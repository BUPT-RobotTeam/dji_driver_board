/**
 * @file oslib_can_dispatch.c
 * @author KYzhang
 * @brief OSLIB-CAN接收分配模块
 * @version 0.2
 * @date 2021-01-04
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "oslib_config.h"

#ifdef OSLIB_CAN_MODULE_ENABLED

#include "can/oslib_can.h"
#include "can/oslib_can_dispatch.h"
#include "utils/oslib_hash.h"
#include "debug/oslib_debug.h"

#define OSLIB_CAN_HashKey(id, ide, rtr, mask) ((((id) << 3) | (ide) | (rtr)) & ((mask << 3) | 0x07))
#define Get_CANID(can) ((can == &hcan1) ? 0:1)
/*-CAN任务函数-----------------------------------------------*/
#define VescMask (~(0x1FUL << 8))

uint32_t mask_list[2][2][30];
int mask_list_num[2][2] = {0,0,0,0};

static void CAN_Dispatch_Task(void *argument)
{
    OSLIB_CAN_Dispatch_t *can_dispatch = (OSLIB_CAN_Dispatch_t *)argument;
    OSLIB_CAN_Handle_t *can_handle = can_dispatch->can_handle;
    CAN_ConnMessage message;
    CAN_IDRecord_t *record;
    for (;;)
    {   
        osMessageQueueGet(can_handle->rx_queue, &message, 0, osWaitForever);
        Debug("%s: Rx FIFO%d[0x%x]", can_handle->name, message.fifo, message.id);
        uint8_t can_id = Get_CANID(can_handle->hcan);
        record = (CAN_IDRecord_t *)HashTable_get(
            can_dispatch->table, 
            (const void *)OSLIB_CAN_HashKey(message.id,message.ide,message.rtr,mask_list[can_id][message.fifo][message.index])
        );
        if (NULL != record)
        {
            if (record->queue != NULL)
                osMessageQueuePut(*(record->queue), &message, 0, 0);
            else if (record->callback != NULL)
                record->callback(&message);
        }
    }
}

/*-筛选器配置函数-----------------------------------------*/
#define StdFilterID16(id) ((((id) << 3) | CAN_RTR_DATA | CAN_ID_STD) << 2)
#define StdFilterID32(id) (((id) << (18 + 3)) | CAN_RTR_DATA | CAN_ID_STD)
#define ExtFilterID32(id) (((id) << 3) | CAN_RTR_DATA | CAN_ID_EXT)
#define StdFilterMask16(mask) ((((mask) << 3) | 0x06) << 2) // 0x06代表过滤ide
#define StdFilterMask32(mask) (((mask) << (18 + 3)) | 0x06) // 0x06代表过滤ide
#define ExtFilterMask32(mask) (((mask) << 3) | 0x06)        // 0x06代表过滤ide

#define VescFilterMask ((~(0x1F << (8 + 3))) & 0xFFFFFFFE) // 0x1F代表不参与过滤的5位
/**
 * @brief 以32bit列表方式配置筛选器组
 *
 * @param hcan 待配置CAN的句柄指针
 * @param fifo CAN_FILTER_FIFO0/CAN_FILTER_FIFO1
 * @param bank 筛选器组编号, 范围为0~13
 * @param fid1 筛选器组接受第一个ID
 * @param fid2 筛选器组接受第二个ID
 */
static void OSLIB_CAN_List32FilterConfig(CAN_HandleTypeDef *hcan, uint8_t fifo, uint32_t bank, uint32_t fid1, uint32_t fid2)
{
    uint8_t can_id = Get_CANID(hcan);
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = bank;
    sFilterConfig.FilterFIFOAssignment = fifo;
    sFilterConfig.FilterIdHigh = (uint16_t)((fid1 >> 16) & 0xFFFF);
    sFilterConfig.FilterIdLow = (uint16_t)((fid1)&0xFFFF);
    sFilterConfig.FilterMaskIdHigh = (uint16_t)((fid2 >> 16) & 0xFFFF);
    sFilterConfig.FilterMaskIdLow = (uint16_t)((fid2)&0xFFFF);

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();

    mask_list[can_id][fifo][mask_list_num[can_id][fifo]++] = FULLMASK;
    mask_list[can_id][fifo][mask_list_num[can_id][fifo]++] = FULLMASK;

    Debug("CAN: Filter[%d] = (0x%08x, 0x%08x)", bank, fid1, fid2);
}

/**
 * @brief 以16bit列表方式配置筛选器组
 *
 * @param hcan 待配置CAN的句柄指针
 * @param fifo CAN_FILTER_FIFO0/CAN_FILTER_FIFO1
 * @param bank 筛选器组编号, 范围为0~13
 * @param fid1 筛选器组接受第一个ID
 * @param fid2 筛选器组接受第二个ID
 * @param fid3 筛选器组接受第三个ID
 * @param fid4 筛选器组接受第四个ID
 */
static void OSLIB_CAN_List16FilterConfig(CAN_HandleTypeDef *hcan, uint8_t fifo, uint32_t bank,
                                         uint16_t fid1, uint16_t fid2, uint16_t fid3, uint16_t fid4)
{
    CAN_FilterTypeDef sFilterConfig;
    uint8_t can_id = Get_CANID(hcan);
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = bank;
    sFilterConfig.FilterFIFOAssignment = fifo;
    sFilterConfig.FilterIdHigh = (uint16_t)((fid3)&0xFFFF);
    sFilterConfig.FilterIdLow = (uint16_t)((fid1)&0xFFFF);
    sFilterConfig.FilterMaskIdHigh = (uint16_t)((fid4)&0xFFFF);
    sFilterConfig.FilterMaskIdLow = (uint16_t)((fid2)&0xFFFF);

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();

    mask_list[can_id][fifo][mask_list_num[can_id][fifo]++] = FULLMASK;
    mask_list[can_id][fifo][mask_list_num[can_id][fifo]++] = FULLMASK;
    mask_list[can_id][fifo][mask_list_num[can_id][fifo]++] = FULLMASK;
    mask_list[can_id][fifo][mask_list_num[can_id][fifo]++] = FULLMASK;

    Debug("CAN: Filter[%d] = (0x%04hx, 0x%04hx, 0x%04hx, 0x%04hx)", bank, fid1, fid2, fid3, fid4);
}

/**
 * @brief 以32bit掩码方式配置筛选器组
 *
 * @param hcan 待配置CAN的句柄指针
 * @param fifo CAN_FILTER_FIFO0/CAN_FILTER_FIFO1
 * @param bank 筛选器组编号, 范围为0~13
 * @param id 筛选器组中的参考ID
 * @param mask 筛选器组中的掩码
 */
static void OSLIB_CAN_Mask32FilterConfig(CAN_HandleTypeDef *hcan, uint8_t fifo, uint32_t bank, uint32_t id, uint32_t mask)
{
    CAN_FilterTypeDef sFilterConfig;
    uint8_t can_id = Get_CANID(hcan);
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = bank;
    sFilterConfig.FilterFIFOAssignment = fifo;
    sFilterConfig.FilterIdHigh = (uint16_t)((id >> 16) & 0xFFFF);
    sFilterConfig.FilterIdLow = (uint16_t)((id)&0xFFFF);
    sFilterConfig.FilterMaskIdHigh = (uint16_t)((mask >> 16) & 0xFFFF);
    sFilterConfig.FilterMaskIdLow = (uint16_t)((mask)&0xFFFF);

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();

    mask_list[can_id][fifo][mask_list_num[can_id][fifo]++] = (mask >> 3);

    Debug("CAN: Filter[%d] = (id=0x%08x, mask=0x%08x)", bank, id, mask);
}

/**
 * @brief 以16bit掩码方式配置筛选器组
 *
 * @param hcan 待配置CAN的句柄指针
 * @param fifo CAN_FILTER_FIFO0/CAN_FILTER_FIFO1
 * @param bank 筛选器组编号, 范围为0~13
 * @param id 筛选器组中的参考ID
 * @param mask 筛选器组中的掩码
 */
static void OSLIB_CAN_Mask16FilterConfig(CAN_HandleTypeDef *hcan, uint8_t fifo, uint32_t bank, uint32_t id1, uint32_t id2, uint32_t mask1, uint32_t mask2)
{
    CAN_FilterTypeDef sFilterConfig;
    uint8_t can_id = Get_CANID(hcan);
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = bank;
    sFilterConfig.FilterFIFOAssignment = fifo;
    sFilterConfig.FilterIdHigh = (uint16_t)(mask1 & 0xFFFF);
    sFilterConfig.FilterIdLow = (uint16_t)(id1 & 0xFFFF);
    sFilterConfig.FilterMaskIdHigh = (uint16_t)(mask2 & 0xFFFF);
    sFilterConfig.FilterMaskIdLow = (uint16_t)(id2 & 0xFFFF);

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();

    mask_list[can_id][fifo][mask_list_num[can_id][fifo]++] = (mask1 >> 3);
    mask_list[can_id][fifo][mask_list_num[can_id][fifo]++] = (mask2 >> 3);

    Debug("CAN: Filter[%d] = (id1=0x%04x, mask2=0x%04x) (id2=0x%04x, mask2=0x%04x)", bank, id1, mask1, id2, mask2);
}
// TODO: 插入哈希表前, 检查是否有ID冲突, 并给出警告
// TODO: 检查ExtID是否有被VescMask覆盖掉的部分, 并给出警告
/**
 * @brief 为某个CAN配置分发器
 *
 * @param can_handle 已初始化的CAN
 * @param can_dispatch 分发器
 * @param can_record_list ID记录表
 * @param can_record_list_size ID记录表大小
 */
void OSLIB_CAN_Dispatch_Init(OSLIB_CAN_Dispatch_t *can_dispatch, OSLIB_CAN_Handle_t *can_handle, CAN_IDRecord_t *can_record_list, size_t can_record_list_size)
{
    // 创建CAN处理任务
    static osThreadAttr_t task_attr = {
        .priority = osPriorityAboveNormal7,
        .stack_size = 128 * 4,
    };
    can_handle->plugin = (void *)can_dispatch;
    can_dispatch->task = osThreadNew(CAN_Dispatch_Task, can_dispatch, &task_attr);
    can_dispatch->table = HashTable_create(NULL, NULL, NULL);
    can_dispatch->can_handle = can_handle;
    // 配置CAN筛选器+插入哈希表
    // 扫描列表三遍, 每次处理一种类型的ID
    int bank = 0, index = 0;
    const int bank_offset = (can_handle->hcan->Instance == CAN1) ? 0 : 14;  //CAN2的筛选器组编号从14开始
    CAN_IDRecord_t *store32 = NULL;  // 扩展帧 ID 2个一组 存储每一组的第一个
    int store16_len = 0;
    CAN_IDRecord_t *store16[3];      // 标准帧 ID 4个一组 存储每一组的前三个
    uint32_t and_value = 0xFFFFFFFF; // 用于取所有ID共同的'1'位
    uint32_t or_value = 0x0;         // 用于取所有ID共同的‘0’位
    uint32_t mask = 0xFFFFFFFF;      // 掩码初始值
    // 处理列表过滤模式下的扩展帧ID, 每2个扩展帧ID采用32位列表筛选器
    for (index = 0; index < can_record_list_size; index++)
    {
        if (bank >= 12)
            goto bankout_ext; // 筛选器不够用
        CAN_IDRecord_t *record = &can_record_list[index];
        if (record->idtype != CAN_IDTYPE_EXT ||record->filter != CAN_FILTERMODE_IDLIST)
            continue;
        if (store32 == NULL)
            store32 = record;
        else {
            OSLIB_CAN_List32FilterConfig(can_handle->hcan, CAN_FILTER_FIFO1, bank_offset + bank++,
                                         ExtFilterID32(store32->id), ExtFilterID32(record->id));
            store32 = NULL;
        }
        HashTable_insert(can_dispatch->table,
                         (const void *)OSLIB_CAN_HashKey(record->id, CAN_ID_EXT, CAN_RTR_DATA, FULLMASK), record);
    }
    if (store32 != NULL)
        OSLIB_CAN_List32FilterConfig(can_handle->hcan, CAN_FILTER_FIFO1, bank_offset + bank++,
                                     ExtFilterID32(store32->id), ExtFilterID32(store32->id));
    // 处理列表过滤模式下的标准帧ID, 每4个标准帧ID采用16位列表筛选器
    for (index = 0; index < can_record_list_size; index++)
    {
        if (bank >= 13)
            goto bankout_std;
        CAN_IDRecord_t *record = &can_record_list[index];
        if (record->idtype != CAN_IDTYPE_STD || record->filter != CAN_FILTERMODE_IDLIST)
            continue;
        if (store16_len < 3)
            store16[store16_len++] = record;
        else {
            OSLIB_CAN_List16FilterConfig(can_handle->hcan, CAN_FILTER_FIFO0, bank_offset + bank++,
                                         StdFilterID16(store16[0]->id), StdFilterID16(store16[1]->id),
                                         StdFilterID16(store16[2]->id), StdFilterID16(record->id));
            store16_len = 0;
        }
        HashTable_insert(can_dispatch->table,
                         (const void *)OSLIB_CAN_HashKey(record->id, CAN_ID_STD, CAN_RTR_DATA, FULLMASK), record);
    }
    switch (store16_len)
    {
    case 3:
        OSLIB_CAN_List16FilterConfig(can_handle->hcan, CAN_FILTER_FIFO0, bank_offset + bank++,
                                     StdFilterID16(store16[0]->id), StdFilterID16(store16[1]->id),
                                     StdFilterID16(store16[2]->id), StdFilterID16(store16[0]->id));
        break;
    case 2:
        OSLIB_CAN_List16FilterConfig(can_handle->hcan, CAN_FILTER_FIFO0, bank_offset + bank++,
                                     StdFilterID16(store16[0]->id), StdFilterID16(store16[1]->id),
                                     StdFilterID16(store16[0]->id), StdFilterID16(store16[1]->id));
        break;
    case 1:
        OSLIB_CAN_List16FilterConfig(can_handle->hcan, CAN_FILTER_FIFO0, bank_offset + bank++,
                                     StdFilterID16(store16[0]->id), StdFilterID16(store16[0]->id),
                                     StdFilterID16(store16[0]->id), StdFilterID16(store16[0]->id));
        break;
    case 0:
    default:
        break;
    }

    // 处理掩码过滤模式下的ID,每1个扩展帧ID采用32位列表筛选器
    for (index = 0; index < can_record_list_size; index++)
    {
        if (bank >= 12)
            goto bankout_ext; // 筛选器不够用
        CAN_IDRecord_t *record = &can_record_list[index];
        if (record->filter != CAN_FILTERMODE_IDMASK || record->idtype != CAN_IDTYPE_EXT)
            continue;
        HashTable_insert(can_dispatch->table,
                         (const void *)OSLIB_CAN_HashKey(record->id, CAN_ID_EXT, CAN_RTR_DATA, record->mask), record);
        OSLIB_CAN_Mask32FilterConfig(can_handle->hcan, CAN_FILTER_FIFO1, bank_offset + bank++,
                                   ExtFilterID32(record->id), ExtFilterID32(record->mask));
    }
    store32 = NULL;
    // 处理掩码过滤模式下的ID,每2个标准帧ID采用16位列表筛选器
    for (index = 0; index < can_record_list_size; index++)
    {
        if (bank >= 12)
            goto bankout_std; // 筛选器不够用
        CAN_IDRecord_t *record = &can_record_list[index];
        if (record->idtype != CAN_IDTYPE_STD || record->filter != CAN_FILTERMODE_IDMASK)
            continue;
        if (store32 == NULL)
            store32 = record;
        else {
            OSLIB_CAN_Mask16FilterConfig(can_handle->hcan, CAN_FILTER_FIFO0, bank_offset + bank++,
                                         StdFilterID16(store32->id), StdFilterID16(record->id),StdFilterMask16(store32->mask),StdFilterMask16(record->mask));
            store32 = NULL;
        }
        HashTable_insert(can_dispatch->table,
                         (const void *)OSLIB_CAN_HashKey(record->id, CAN_ID_EXT, CAN_RTR_DATA, record->mask), record);
    }
    if (store32 != NULL)
        OSLIB_CAN_Mask16FilterConfig(can_handle->hcan, CAN_FILTER_FIFO0, bank_offset + bank++,
                                     StdFilterID16(store32->id),StdFilterID16(store32->id),StdFilterMask16(store32->mask),StdFilterMask16(store32->mask));
    return;
    // stm32为每个CAN提供14个筛选器, 如果筛选器不够用, 剩下的ID全部合成掩码
    // 掩码的核心是为所需的ID找到尽可能多的相同的位, 针对这些位进行过滤
    // 分别对所有ID取与和取或, 得到的结果分别可以反应出所有ID共同为1的位和所有ID共同为0的位
bankout_ext:
    for (; index < can_record_list_size; index++)
    {
        CAN_IDRecord_t *record = &can_record_list[index];
        if (record->idtype != CAN_IDTYPE_EXT)
            continue;
        and_value &= record->id;
        or_value |= record->id;
        HashTable_insert(can_dispatch->table,
                         (const void *)OSLIB_CAN_HashKey(record->id, CAN_ID_EXT, CAN_RTR_DATA, FULLMASK), record);
    }
    if (store32 != NULL)
    {
        and_value &= store32->id;
        or_value |= store32->id;
    }
    mask &= (and_value | (~or_value));
    OSLIB_CAN_Mask32FilterConfig(can_handle->hcan, CAN_FILTER_FIFO1, bank_offset + bank++,
                               ExtFilterID32(and_value & or_value), ExtFilterMask32(mask));
    // 重置and_value和or_value用来处理标准帧id, (mask已经不再使用了, 所以不需要重置)
    and_value = 0xFFFFFFFF;
    or_value = 0x0;
    index = 0;
bankout_std:
    for (; index < can_record_list_size; index++)
    {
        CAN_IDRecord_t *record = &can_record_list[index];
        if (record->idtype != CAN_IDTYPE_STD)
            continue;
        and_value &= record->id;
        or_value |= record->id;
        HashTable_insert(can_dispatch->table,
                         (const void *)OSLIB_CAN_HashKey(record->id, CAN_ID_STD, CAN_RTR_DATA, FULLMASK), record);
    }
    for (int i = 0; i < store16_len; i++)
    {
        and_value &= store16[i]->id;
        or_value |= store16[i]->id;
    }
    OSLIB_CAN_Mask32FilterConfig(can_handle->hcan, CAN_FILTER_FIFO0, bank_offset + bank++,
                               StdFilterID32(and_value & or_value), StdFilterMask32(and_value | (~or_value)));
    return;
}
#endif // OSLIB_CAN_MODULE_ENABLED
