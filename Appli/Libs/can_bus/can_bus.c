#include "can_bus.h"
#include <string.h>

static HAL_StatusTypeDef CAN_Bus_SendMessage(FDCAN_HandleTypeDef *hfdcan,
                                             const CAN_BusMessage_t *message);

static uint32_t CAN_Bus_ToFdcanDlc(uint8_t byte_count)
{
    static const uint32_t dlc_map[] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };

    if (byte_count > 8U) {
        return FDCAN_DLC_BYTES_0;
    }

    return dlc_map[byte_count];
}

static uint8_t CAN_Bus_FromFdcanDlc(uint32_t fdcan_dlc)
{
    if (fdcan_dlc > FDCAN_DLC_BYTES_8) {
        return 0U;
    }

    return (uint8_t)fdcan_dlc;
}

static void CAN_Bus_MessageInit(CAN_BusMessage_t *message, uint16_t id)
{
    if (message == NULL) {
        return;
    }

    memset(message, 0, sizeof(*message));
    message->id = id;
}

static void CAN_Bus_BuildU8(CAN_BusMessage_t *message, uint16_t id, uint8_t value)
{
    if (message == NULL) {
        return;
    }

    CAN_Bus_MessageInit(message, id);
    message->dlc = 1;
    message->data[0] = value;
}

static void CAN_Bus_BuildU16(CAN_BusMessage_t *message, uint16_t id, uint16_t value)
{
    if (message == NULL) {
        return;
    }

    CAN_Bus_MessageInit(message, id);
    message->dlc = 2;
    message->data[0] = (uint8_t)(value & 0xFFU);
    message->data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void CAN_Bus_BuildU32(CAN_BusMessage_t *message, uint16_t id, uint32_t value)
{
    if (message == NULL) {
        return;
    }

    CAN_Bus_MessageInit(message, id);
    message->dlc = 4;
    message->data[0] = (uint8_t)(value & 0xFFU);
    message->data[1] = (uint8_t)((value >> 8) & 0xFFU);
    message->data[2] = (uint8_t)((value >> 16) & 0xFFU);
    message->data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

HAL_StatusTypeDef CAN_Bus_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == NULL) {
        return HAL_ERROR;
    }

    /* Match only destination bits 3:0; ignore priority and message ID. */
    /* Target Device ID: 0x0 (Main Controller) */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterIndex = 0;
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x00FU;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Match broadcast destination 0xF using the same lower-four-bit mask. */
    filter.FilterIndex = 1;
    filter.FilterID1 = CAN_DEST_BROADCAST_ID;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef CAN_Bus_SendMessage(FDCAN_HandleTypeDef *hfdcan,
                                             const CAN_BusMessage_t *message)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if ((hfdcan == NULL) || (message == NULL) || (message->dlc > 8U) || (message->id > 0x7FFU)) {
        return HAL_ERROR;
    }

    tx_header.Identifier = message->id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = CAN_Bus_ToFdcanDlc(message->dlc);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)message->data);
}

HAL_StatusTypeDef CAN_Bus_SendU8(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t value)
{
    CAN_BusMessage_t message;

    CAN_Bus_BuildU8(&message, id, value);
    return CAN_Bus_SendMessage(hfdcan, &message);
}

HAL_StatusTypeDef CAN_Bus_SendU16(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint16_t value)
{
    CAN_BusMessage_t message;

    CAN_Bus_BuildU16(&message, id, value);
    return CAN_Bus_SendMessage(hfdcan, &message);
}

HAL_StatusTypeDef CAN_Bus_SendU32(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint32_t value)
{
    CAN_BusMessage_t message;

    CAN_Bus_BuildU32(&message, id, value);
    return CAN_Bus_SendMessage(hfdcan, &message);
}

HAL_StatusTypeDef CAN_Bus_SendRaw(FDCAN_HandleTypeDef *hfdcan, uint16_t id, const uint8_t *data, uint8_t len)
{
    if ((data == NULL) || (len > 8U)) {
        return HAL_ERROR;
    }

    CAN_BusMessage_t message;
    memset(&message, 0, sizeof(message));
    message.id = id;
    message.dlc = len;
    memcpy(message.data, data, len);

    return CAN_Bus_SendMessage(hfdcan, &message);
}

HAL_StatusTypeDef CAN_Bus_Receive(FDCAN_HandleTypeDef *hfdcan, CAN_BusMessage_t *message)
{
    FDCAN_RxHeaderTypeDef rx_header = {0};

    if ((hfdcan == NULL) || (message == NULL)) {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, message->data) != HAL_OK) {
        return HAL_ERROR;
    }

    if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
        (rx_header.RxFrameType != FDCAN_DATA_FRAME) ||
        (rx_header.FDFormat != FDCAN_CLASSIC_CAN) ||
        (rx_header.DataLength > FDCAN_DLC_BYTES_8)) {
        return HAL_ERROR;
    }

    message->id = (uint16_t)rx_header.Identifier;
    message->dlc = CAN_Bus_FromFdcanDlc(rx_header.DataLength);
    return HAL_OK;
}

uint16_t CAN_Bus_ReadU16(const CAN_BusMessage_t *message)
{
    if ((message == NULL) || (message->dlc < 2U)) {
        return 0U;
    }

    return (uint16_t)message->data[0] | ((uint16_t)message->data[1] << 8);
}

uint8_t CAN_Bus_ReadU8(const CAN_BusMessage_t *message)
{
    if ((message == NULL) || (message->dlc < 1U)) {
        return 0U;
    }

    return message->data[0];
}

uint32_t CAN_Bus_ReadU32(const CAN_BusMessage_t *message)
{
    if ((message == NULL) || (message->dlc < 4U)) {
        return 0UL;
    }

    return (uint32_t)message->data[0]
         | ((uint32_t)message->data[1] << 8)
         | ((uint32_t)message->data[2] << 16)
         | ((uint32_t)message->data[3] << 24);
}
