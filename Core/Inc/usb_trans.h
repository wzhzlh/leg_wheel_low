#ifndef __USB_TRANS_H__
#define __USB_TRANS_H__

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* USB_OTG_HS（ULPI 高速）bulk OUT 端点最大包长，
   必须与 usbd_cdc.h 的 CDC_DATA_HS_MAX_PACKET_SIZE 一致；
   收到短包（< 512 B）即认为一帧结束。 */
#define USB_CDC_MAX_PACKET_SIZE     512U
/* 组帧缓冲区上限，必须 <= usbd_cdc_if.h 的 APP_RX_DATA_SIZE */
#define USB_CDC_RECV_BUFFER_SIZE    2048U

typedef void (*Recv_finished_cb_t)(uint8_t *src, uint16_t size);       // 数据包接收完成回调
typedef void (*RecvBufferOverflow_cb_t)(void *user_data);               // 接收缓冲区溢出回调

extern QueueHandle_t kUsbRecvQueue;
extern TaskHandle_t  kUsbRecvTaskHandle;

void USB_CDC_Init(Recv_finished_cb_t recv_cb, RecvBufferOverflow_cb_t recv_overflow_cb, void *user_data);
void CDC_RecvCplt_Handler(uint8_t *Buf, uint32_t *Len);

#endif
