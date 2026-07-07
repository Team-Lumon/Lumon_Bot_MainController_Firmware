#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "main.h"
#include <stdint.h>

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} CAN_BusMessage_t;

typedef enum {
    CAN_ID_HEARTBEAT = 0x100U,
    CAN_ID_STATUS = 0x101U,
    CAN_ID_COMMAND = 0x200U,
    CAN_ID_LED_COMMAND = 0x201U,
    CAN_ID_ADC_REPORT = 0x300U,
    CAN_ID_DEBUG = 0x400U
} CAN_BusMessageId_t;

HAL_StatusTypeDef CAN_Bus_Init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef CAN_Bus_SendU8(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t value);
HAL_StatusTypeDef CAN_Bus_SendU16(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint16_t value);
HAL_StatusTypeDef CAN_Bus_SendU32(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint32_t value);
HAL_StatusTypeDef CAN_Bus_Receive(FDCAN_HandleTypeDef *hfdcan, CAN_BusMessage_t *message);

uint8_t CAN_Bus_ReadU8(const CAN_BusMessage_t *message);
uint16_t CAN_Bus_ReadU16(const CAN_BusMessage_t *message);
uint32_t CAN_Bus_ReadU32(const CAN_BusMessage_t *message);

#endif
