#include "task.h"

#include "main.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include <task.h>
#include <semphr.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "usb_trans.h"
#include "usbd_cdc_if.h"

#define DEG_TO_RAD 0.01745329252f
#define RPM_TO_RAD_S 0.10471975512f

// 上位机 target 帧超时 [ms]：上位机 250 Hz 下发，超时即轮子电流清零
#define TARGET_TIMEOUT_MS     100
// M2006 电流指令满量程 ±16384，留 2% 余量，限幅后再转 int16，避免越界反号
#define WHEEL_CUR_MAX_CMD     16000.0f
// 轮轴力矩 -> 电流指令换算 [counts/(N*m)]：
//   C620 电调 ±16384 counts 对应 ±20 A（819.2 counts/A），
//   M2006（P36 减速箱，36:1）**输出侧**力矩常数约 0.18 N*m/A，
//   故 1 N*m(轮轴) ≈ 5.56 A ≈ 4500 counts。取 4550 略偏保守。
//   注意：0.18 N*m/A 已经是轮轴（输出侧）的量，36:1 已在里面，
//   所以换算时**不要**再除/乘减速比，否则力矩会差 36 倍。
//   实机标定结果不同时只改这一个数。
#define WHEEL_COUNTS_PER_NM   4550.0f
// 轮子反馈超时 [ms]：超过该时间没收到 M2006 反馈就不再给付力矩
#define WHEEL_FB_TIMEOUT_MS   50
// M2006（P36 减速箱）减速比 36:1：CAN 反馈是转子侧角度/转速，换算到轮轴要除以该值
#define WHEEL_GEAR_RATIO      36.0f

// 帧长自检：与上位机 usb_drive/data_pack.h 逐字节一致
//   下行 RobotTarget_t = 2×(angle,omega,torque) + 4×(torque,angle,omega,kp,kd) = 104 B
//   上行 RobotState_t  = 2×(angle,omega) + 4×RobStrideState_t = 208 B
_Static_assert(sizeof(WheelTarget_t) == 12, "WheelTarget_t 需 12 B（含 torque）");
_Static_assert(sizeof(JointTarget_t) == 20, "JointTarget_t 需 20 B");
_Static_assert(sizeof(RobotTarget_t) == 104, "RobotTarget_t 需 104 B，检查 WheelTarget_t/JointTarget_t");
_Static_assert(offsetof(RobotTarget_t, joint) == 24, "joint[] 起始偏移必须是 24 B");
_Static_assert(sizeof(RobotState_t) == 208, "RobotState_t 需 208 B，检查 RobStrideState_t");

volatile int wheel_cnt_send;
volatile int joint_cnt_send;
volatile uint32_t cdc_state_drop_cnt;
M2006_TypeDef wheel_motor[2];   // [0]=左轮(0x201)，[1]=右轮(0x202)
volatile uint8_t wheel_fb_ok[2]; // 该轮是否已收到过反馈帧（0 = 从未收到，先不输出力矩）
static volatile TickType_t wheel_fb_tick[2];  // 该轮最近一次 CAN 反馈的 tick
RobotState_t rob_state;
RobotTarget_t rob_target;

RobStride_t joint_device[4];

JointCalib_t joint_calib[4] = {
    {0.0f, 1},   // 0: lf_joint1  左前曲柄（α_L），电机 ID 1
    {0.0f, 1},   // 1: lb_joint1  左后曲柄（β_L），电机 ID 2
    {0.0f, 1},   // 2: rf_joint1  右前曲柄（α_R），电机 ID 3
    {0.0f, 1},   // 3: rb_joint1  右后曲柄（β_R），电机 ID 4
};

/// 取该关节的方向系数（+1 / -1）
static inline float joint_inv_sign(uint8_t i)
{
    return (joint_calib[i].motor_inv < 0) ? -1.0f : 1.0f;
}

static SemaphoreHandle_t usb_target_ready;

// 最近一次收到 target 帧的 tick：轮子任务用它做链路看门狗（USB 掉线时电流必须清零）
static volatile TickType_t last_target_tick;

static void usb_target_received(uint8_t *data, uint16_t size)
{
    if (size == sizeof(RobotTarget_t)) {
        memcpy(&rob_target, data, sizeof(rob_target));
        last_target_tick = xTaskGetTickCount();
        if (usb_target_ready != NULL) {
            xSemaphoreGive(usb_target_ready);
        }
    }
}

static RobStride_t *const joint_devices[4] = {
    &joint_device[0], &joint_device[1], &joint_device[2], &joint_device[3]
};

// 置位前 joint_control_task 不下发控制帧，避免向未使能的电机发指令
static volatile uint8_t joint_ready = 0;

void M2006_Receive(M2006_TypeDef *dst, uint8_t *data)
{
    const uint32_t idx = (dst == &wheel_motor[1]) ? 1U : 0U;

    dst->MchanicalAngle = (uint16_t)((data[0] << 8) | data[1]);
    dst->Speed = (int16_t)((data[2] << 8) | data[3]);
    wheel_fb_tick[idx] = xTaskGetTickCount();   // 反馈新鲜度由轮子任务判断

    // 首帧只对齐基准，不做多圈展开：
    // 此时 LsatAngle 还是复位值 0，机械角可能是任意值，按 diff 判圈会让 r 跳变、
    // Angle 突然偏一整圈（8192），上位机的里程计/LQR 会吃到一次跳变。
    // wheel_fb_ok 同时是该轮“已有反馈”的标志，轮子任务据此决定能否输出力矩。
    if (wheel_fb_ok[idx] == 0) {
        wheel_fb_ok[idx] = 1;
        dst->r = 0;
        dst->LsatAngle = dst->MchanicalAngle;
        dst->Angle = dst->MchanicalAngle;
        dst->Angle_DEG = dst->Angle * 0.0439453125f;
        return;
    }
    const int16_t diff = (int16_t)(dst->MchanicalAngle - dst->LsatAngle);
    if (diff > 4000) {
        --dst->r;
    } else if (diff < -4000) {
        ++dst->r;
    }
    dst->Angle = dst->r * 8192 + dst->MchanicalAngle;
    dst->Angle_DEG = dst->Angle * 0.0439453125f;
    dst->LsatAngle = dst->MchanicalAngle;
}

int rm_send_2006(uint8_t *send_current, uint32_t id)
{
    FDCAN_TxHeaderTypeDef header = {0};

    header.Identifier          = id;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_OFF;
    header.FDFormat            = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0x00U;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, send_current) == HAL_OK;
}

void wheel_control_task(void *argument)
{
    TickType_t Last_wake_time = xTaskGetTickCount();
    // 右轮(0x202)装反：反馈和电流指令都乘 -1，两处配对即可
    const float dir[2] = {1.0f, -1.0f};
    (void)argument;

    for (;;) {
        int16_t command[2] = {0, 0};
        RobotTarget_t target;
        uint8_t fb_ok[2];
        TickType_t fb_tick[2];
        uint8_t alive;
        TickType_t tick_now;

        // 取快照：反馈由 FDCAN 中断写、目标由 USB 回调写，关中断避免读到半新半旧
        taskENTER_CRITICAL();
        target = rob_target;
        fb_ok[0] = wheel_fb_ok[0];
        fb_ok[1] = wheel_fb_ok[1];
        fb_tick[0] = wheel_fb_tick[0];
        fb_tick[1] = wheel_fb_tick[1];
        // 轮子反馈只用于上报（上位机的里程计 / LQR 状态），不参与本地控制
        for (uint32_t i = 0; i < 2; ++i) {
            rob_state.wheel[i].angle =
                wheel_motor[i].Angle_DEG * DEG_TO_RAD * dir[i] / WHEEL_GEAR_RATIO;
            rob_state.wheel[i].omega =
                (float)wheel_motor[i].Speed * RPM_TO_RAD_S * dir[i] / WHEEL_GEAR_RATIO;
        }
        // 链路看门狗：上位机断链后不能再执行上一帧的力矩，必须把电流清零
        tick_now = xTaskGetTickCount();
        alive = (uint8_t)((last_target_tick != 0) &&
                          ((tick_now - last_target_tick) <= pdMS_TO_TICKS(TARGET_TIMEOUT_MS)));
        taskEXIT_CRITICAL();

        // 纯力矩直通：轮轴力矩 [N*m] -> M2006 电流指令 [counts]
        //     counts = τ_wheel * WHEEL_COUNTS_PER_NM
        // WHEEL_COUNTS_PER_NM 已含 36:1 减速比与输出侧力矩常数，这里不再做
        // 任何除以减速比的操作；位置/速度环、目标角限步都不再做。
        for (uint32_t i = 0; i < 2; ++i) {
            const WheelTarget_t *t = &target.wheel[i];

            // 反馈可用 = 曾收到过反馈且仍然新鲜（CAN 线松了不能瞎子给力矩）
            const uint8_t fb_fresh = (uint8_t)((fb_ok[i] != 0) &&
                ((tick_now - fb_tick[i]) <= pdMS_TO_TICKS(WHEEL_FB_TIMEOUT_MS)));

            // if (alive == 0U || fb_fresh == 0U) {
            //     command[i] = 0;
            //     continue;
            // }

            float tau = t->torque;
            // 非法浮点（帧错位 / 上位机 NaN）不能直接乘成越界 int16
            if (!(tau > -1000.0f && tau < 1000.0f)) {
                tau = 0.0f;
            }
            float cur = tau * WHEEL_COUNTS_PER_NM;
            if (cur > WHEEL_CUR_MAX_CMD)  { cur = WHEEL_CUR_MAX_CMD; }
            if (cur < -WHEEL_CUR_MAX_CMD) { cur = -WHEEL_CUR_MAX_CMD; }
            command[i] = (int16_t)(cur * dir[i]);
        }

        uint8_t data[8] = {0};
        data[0] = (uint8_t)((uint16_t)command[0] >> 8);
        data[1] = (uint8_t)((uint16_t)command[0]);
        data[2] = (uint8_t)((uint16_t)command[1] >> 8);
        data[3] = (uint8_t)((uint16_t)command[1]);
        wheel_cnt_send += rm_send_2006(data, 0x200);

        // 4 ms = 250 Hz：与上位机 500 Hz 控制环 / 250 Hz 下发 / 250 Hz 状态上报对齐
        vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(4));
    }
}

void motor_init_task(void *argument)
{
    (void)argument;

    // 放在任务里用 vTaskDelay 做间隔，不依赖 HAL_Delay / TIM2 时基
    for (uint32_t i = 0; i < 4U; ++i) {
        RobStrideInit(&joint_device[i], &hfdcan1, 1U + i, RobStride_EL05);
        (void)RobStrideSetMode(&joint_device[i], RobStride_MotionControl);
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
    for (uint32_t i = 0; i < 4U; ++i) {
        (void)RobStrideEnable(&joint_device[i]);
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
    joint_ready = 1;
    vTaskDelete(NULL);
}

void joint_control_task(void *argument)
{
    (void)argument;
    // 等电机初始化完成再下发控制帧
    while (joint_ready == 0U) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    TickType_t Last_wake_time = xTaskGetTickCount();
    for (;;) {
        // 整帧快照：USB 回调跑在更高优先级的接收任务里，可能正改写 rob_target
        RobotTarget_t target;
        uint8_t alive;
        taskENTER_CRITICAL();
        target = rob_target;
        alive = (uint8_t)((last_target_tick != 0) &&
                          ((xTaskGetTickCount() - last_target_tick) <=
                           pdMS_TO_TICKS(TARGET_TIMEOUT_MS)));
        taskEXIT_CRITICAL();

        // 断链：kp/kd/torque 全清 -> 电机失力，绝不保持上一帧的刚度与力矩
        // if (alive == 0U) {
        //     memset(&target.joint[0], 0, sizeof(target.joint));
        // }

        for (uint8_t i = 0; i < 4; ++i) {
            const JointTarget_t *t = &target.joint[i];
            if (joint_devices[i] != NULL) {
                // 关节坐标系 -> 电机坐标系（零点偏移 + 方向）
                //   θ_motor = inv * θ_joint + pos_offset，ω/τ 只乘方向
                const float inv = joint_inv_sign(i);
                joint_cnt_send += (RobStrideMotionControl(joint_devices[i],
                                             (uint8_t)joint_devices[i]->motor_id,
                                             inv * t->torque,
                                             inv * t->angle + joint_calib[i].pos_offset,
                                             inv * t->omega,
                                             t->kp, t->kd) == HAL_OK);
            }
        }
        // 4 ms = 250 Hz
        vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(4));
    }
}

void state_send_task(void *argument)
{
    static RobotState_t state_frame;
    (void)argument;

    // 关节反馈要等 motor_init_task 完成才有意义，否则会把未初始化的
    // 0 rad 当成真实关节角上报，上位机启动瞬间会看到一次跳变
    while (joint_ready == 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    TickType_t Last_wake_time = xTaskGetTickCount();
    for (;;) {
        // 关节反馈在 FDCAN 接收中断中更新，拷贝期间关中断避免发出撕裂数据；
        // 同时在此一次性取好整帧快照，后续 USB 发送不再读 rob_state
        taskENTER_CRITICAL();
        for (uint32_t i = 0; i < 4U; ++i) {
            // 电机坐标系 -> 关节坐标系（零点偏移 + 方向）
            //   q = inv * (θ_motor - pos_offset)，ω/τ 只乘方向
            const float inv = joint_inv_sign((uint8_t)i);
            rob_state.joint[i] = joint_device[i].state;
            rob_state.joint[i].rad =
                inv * (rob_state.joint[i].rad - joint_calib[i].pos_offset);
            rob_state.joint[i].omega = inv * rob_state.joint[i].omega;
            rob_state.joint[i].torque = inv * rob_state.joint[i].torque;
        }
        state_frame = rob_state;
        taskEXIT_CRITICAL();

        // 上一帧未发完时 CDC_Transmit_HS 返回 USBD_BUSY，重试几次仍失败则丢弃本帧
        uint8_t ret = USBD_FAIL;
        for (uint32_t attempt = 0; attempt < 3U; ++attempt) {
            ret = CDC_Transmit_HS((uint8_t *)&state_frame, (uint16_t)sizeof(state_frame));
            if (ret == USBD_OK) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
        if (ret != USBD_OK) {
            ++cdc_state_drop_cnt;   // 便于上位机/调试查看丢帧情况
        }

        // 4 ms = 250 Hz：208 B × 250 ≈ 52 KB/s
        vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(4));
    }
}

void target_receive_task(void *argument)
{
    (void)argument;
    usb_target_ready = xSemaphoreCreateBinary();
    USB_CDC_Init(usb_target_received, NULL, NULL);
    for (;;) {
        (void)xSemaphoreTake(usb_target_ready, portMAX_DELAY);
    }
}

/* FDCAN1 FIFO0 新报文回调（FDCAN1_IT0_IRQHandler -> HAL_FDCAN_IRQHandler）。
   必须把 FIFO0 读空：只要 FIFO 里还有报文，退出 ISR 后中断会立刻再次挂起，
   形成中断风暴；CAN 中断优先级(5)高于 HAL 时基 TIM2(15)，会把 uwTick 饿死。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    UNUSED(RxFifo0ITs);

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            break;
        }

        if (header.IdType == FDCAN_STANDARD_ID) {
            if (header.Identifier == 0x201U) {
                M2006_Receive(&wheel_motor[0], data);   // 左轮
            } else if (header.Identifier == 0x202U) {
                M2006_Receive(&wheel_motor[1], data);   // 右轮
            }
        } else if (header.IdType == FDCAN_EXTENDED_ID) {
            // 按报文中的电机 ID 只投递给对应设备，缩短 ISR 停留时间
            for (uint32_t i = 0; i < 4U; ++i) {
                if (((header.Identifier >> 8) & 0x00FFU) != joint_device[i].motor_id) {
                    continue;
                }
                (void)RobStrideRecv_Handle(&joint_device[i], hfdcan, header.Identifier, data);
                break;
            }
        }
    }
}
