#include "oslib_config.h"

#ifdef OSLIB_CAN_MODULE_ENABLED

#include "can/oslib_can.h"
#include "can/oslib_can_dispatch.h"
#ifdef OSLIB_UART_MODULE_ENABLED
#include "uart/oslib_uart.h"
#include "config.h"

#endif

/*-Declaration--------------------------------------------*/

/* 在这里添加CAN回调函数的定义或外部声明 */
#ifdef OSLIB_UART_MODULE_ENABLED
static void CanCallback_EchoID(CAN_ConnMessage *msg)
{
    uprintf("CAN: Callback[%x]\r\n", msg->id);
}
#endif


/* 在这里添加任务对应消息队列的外部声明 */
#if USE_OSLIB_CAN_EXAMPLE
extern osMessageQueueId_t ExampleCanTaskQueue;
#endif

extern osMessageQueueId_t can1ReceiveQueueHandle;
extern osMessageQueueId_t can2ReceiveQueueHandle;
extern osMessageQueueId_t protectReceiveQueueHandle;
/*-IDList--------------------------------------------------*/

/* 
 * 以下IDList数组每一项格式为 {<ID>, <ID类型>, <消息队列地址>, <回调函数>}
 * 注意1: <消息队列地址> 和 <回调函数> 两域有且仅有一个有值, 另一个写为NULL
 * 注意2: 数组中的ID数量有限制, 每个CAN要求
 *      [STD/4]+[EXT(扩展ID)/2]+VESC <= 14
 *      其中STD, EXT和VESC分别代表标准ID, 扩展ID和VESC的ID的数量, 方括号[]代表向上取整
 */ 

#if defined(CAN1_ENABLED)

/* 在这里添加对CAN1的其他ID和对应消息队列的指针或回调函数 */
//can1为电机的接口
CAN_IDRecord_t CAN1_RecordList[] = {
//     {0x1, CAN_IDTYPE_STD, NULL, CanCallback_EchoID},
//    CANx_Record_Callback(0x1, CAN_IDTYPE_STD, CanCallback_EchoID),
    // {0x9, CAN_IDTYPE_STD, NULL, CanCallback_EchoID},
    // {0x2, CAN_IDTYPE_EXT, NULL, CanCallback_EchoID},
    CANx_Record_Queue(0x201, CAN_IDTYPE_STD, &can1ReceiveQueueHandle),
    CANx_Record_Queue(0x202, CAN_IDTYPE_STD, &can1ReceiveQueueHandle),
    CANx_Record_Queue(0x203, CAN_IDTYPE_STD, &can1ReceiveQueueHandle),
    CANx_Record_Queue(0x204,CAN_IDTYPE_STD,&can1ReceiveQueueHandle),
    CANx_Record_Queue(0x205,CAN_IDTYPE_STD,&can1ReceiveQueueHandle),
    CANx_Record_Queue(0x206,CAN_IDTYPE_STD,&can1ReceiveQueueHandle),
    CANx_Record_Queue(0x10,CAN_IDTYPE_EXT,&can1ReceiveQueueHandle),
    CANx_Record_Queue_Mask(0x4E,CAN_IDTYPE_EXT,&can1ReceiveQueueHandle,0xFF),
    CANx_Record_Queue_Mask(0x4F,CAN_IDTYPE_EXT,&can1ReceiveQueueHandle,0xFF),
//    CANx_Record_Queue(0x207,CAN_IDTYPE_STD,&can1ReceiveQueueHandle),
#if USE_OSLIB_CAN_EXAMPLE
    // {0x120, CAN_IDTYPE_STD, &ExampleCanTaskQueue, NULL},
    CANx_Record_Queue(0x120, CAN_IDTYPE_STD, &ExampleCanTaskQueue),
    // {0x58, CAN_IDTYPE_VESC, &ExampleCanTaskQueue, NULL},
    CANx_Record_Queue(0x58, CAN_IDTYPE_VESC, &ExampleCanTaskQueue),
#endif
};

size_t CAN1_RecordListSize = sizeof(CAN1_RecordList) / sizeof(CAN_IDRecord_t);
#endif

#if defined(CAN2_ENABLED)

/* 在这里添加对CAN2的其他ID和对应消息队列的指针或回调函数 */
//必须为奇数个任务，不然无法进入Receive_Func
CAN_IDRecord_t CAN2_RecordList[] = {
    // {0x2, CAN_IDTYPE_STD, NULL, CanCallback_EchoID},
    CANx_Record_Callback(0x200,CAN_IDTYPE_STD,CanCallback_EchoID),
//    CANx_Record_Queue(0x200,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
    CANx_Record_Queue(0x201,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
    CANx_Record_Queue(0x202,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
    CANx_Record_Queue(0x203,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
    CANx_Record_Queue(0x204,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
    CANx_Record_Queue(0x205,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
    CANx_Record_Queue(0x206,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
    CANx_Record_Queue(0x207,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
    CANx_Record_Queue(0x208,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
    CANx_Record_Queue(0x209,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
//    CANx_Record_Queue(0x20A,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
//    CANx_Record_Queue(0x20B,CAN_IDTYPE_STD,&can2ReceiveQueueHandle),
#if USE_OSLIB_CAN_EXAMPLE
    // {0x100, CAN_IDTYPE_STD, &ExampleCanTaskQueue, NULL},
    CANx_Record_Queue(0x100, CAN_IDTYPE_STD, &ExampleCanTaskQueue),
    // {0x15, CAN_IDTYPE_VESC, &ExampleCanTaskQueue, NULL},
    CANx_Record_Queue(0x15, CAN_IDTYPE_VESC, &ExampleCanTaskQueue),
#endif
};
#endif
size_t CAN2_RecordListSize = sizeof(CAN2_RecordList) / sizeof(CAN_IDRecord_t);


#endif // OSLIB_CAN_MODULE_ENABLED
