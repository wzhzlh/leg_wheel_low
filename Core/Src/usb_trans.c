#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "usb_device.h"

_Static_assert(USB_CDC_RECV_BUFFER_SIZE <= APP_RX_DATA_SIZE,
               "USB_CDC_RECV_BUFFER_SIZE 不能超过 APP_RX_DATA_SIZE");
_Static_assert(USB_CDC_MAX_PACKET_SIZE == CDC_DATA_HS_MAX_PACKET_SIZE,
               "USB_CDC_MAX_PACKET_SIZE 必须与 HS bulk OUT 端点包长一致");

extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

QueueHandle_t kUsbRecvQueue;
TaskHandle_t  kUsbRecvTaskHandle;

static Recv_finished_cb_t    Recv_Finished_Cb;
static RecvBufferOverflow_cb_t Recv_Overflow_cb;
static void *kUserData;

/* 把 CDC 的 OUT 端点重新挂到 UserRxBufferHS + offset，并等待下一个包。
   CDC_Receive_HS 只投递长度到队列，不主动 re-arm（见 usbd_cdc_if.c），
   所以组帧和重挂全部由这个任务串行完成，避免 ISR 与任务同时改 RxBuffer。 */
static void USB_RecvTask(void *param)
{
    (void)param;

    for (;;)
    {
        int32_t  buffer_index = 0;
        uint32_t pack_size = 0;
        uint8_t  frame_ready = 1U;

        USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);

        do
        {
            USBD_CDC_ReceivePacket(&hUsbDeviceHS);
            if (xQueueReceive(kUsbRecvQueue, &pack_size, portMAX_DELAY) != pdTRUE)
            {
                frame_ready = 0U;
                break;
            }

            // 端点每次都要按满包长度预置接收，所以按 MAX_PACKET_SIZE 预留空间，
            // 否则最后一个包的 PrepareReceive 会写出 UserRxBufferHS 边界
            if (pack_size > USB_CDC_MAX_PACKET_SIZE ||
                (uint32_t)buffer_index + USB_CDC_MAX_PACKET_SIZE > USB_CDC_RECV_BUFFER_SIZE)
            {
                // 一帧超过组帧缓冲区：丢弃整帧并回到缓冲区起始，避免越界写
                USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
                buffer_index = -1;
                frame_ready = 0U;
                if (Recv_Overflow_cb != NULL)
                {
                    Recv_Overflow_cb(kUserData);
                }
                break;
            }

            buffer_index += (int32_t)pack_size;
            USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS + buffer_index);
        } while (pack_size == USB_CDC_MAX_PACKET_SIZE);

        if (frame_ready && buffer_index > 0 && Recv_Finished_Cb != NULL)
        {
            Recv_Finished_Cb(UserRxBufferHS, (uint16_t)buffer_index);
        }
    }
}

// 初始化 USB CDC 应用层
void USB_CDC_Init(Recv_finished_cb_t recv_cb, RecvBufferOverflow_cb_t recv_overflow_cb, void *user_data)
{
    Recv_Finished_Cb = recv_cb;
    Recv_Overflow_cb = recv_overflow_cb;
    kUserData = user_data;

    kUsbRecvQueue = xQueueCreate(8, sizeof(uint32_t));
    xTaskCreate(USB_RecvTask, "usb_cdc_recv", 128, NULL, 6, &kUsbRecvTaskHandle);
}

/* 在 OTG_HS_IRQHandler -> CDC_Receive_HS 中断上下文调用 */
void CDC_RecvCplt_Handler(uint8_t *Buf, uint32_t *Len)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    (void)Buf;
    if (kUsbRecvQueue == NULL)
    {
        return;
    }
    xQueueSendFromISR(kUsbRecvQueue, Len, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
