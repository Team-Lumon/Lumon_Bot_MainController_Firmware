/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bus.h"
#include "ik_wrapper.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

XSPI_HandleTypeDef hxspi1;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN PV */
static volatile uint8_t can_send_pending;
uint32_t canValue = 0;
static MotorCommand_t all_commands[151][8];
static int total_steps = 150;
static int current_send_step = 0;
static bool trajectory_ready = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
static void MX_XSPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef SendMotorCommands(const MotorCommand_t *commands) {
  if (commands == NULL)
    return HAL_ERROR;

  static const uint16_t motor_ids[4] = {1, 2, 4, 8};
  HAL_StatusTypeDef overall_status = HAL_OK;

  for (int i = 0; i < 4; i++) {
    // 1. Scale L_total to uint16_t (in 1.0 mm resolution).
    float len_mm = commands[i].L_total * 1000.0f;
    if (len_mm < 0.0f)
      len_mm = 0.0f;
    if (len_mm > 4095.0f)
      len_mm = 4095.0f;
    uint16_t len_u12 = (uint16_t)len_mm;

    // 2. Scale L_dot (velocity) to signed 12-bit int (range -2048 to 2047, 1.0
    // mm/s per LSb).
    float vel_scaled = commands[i].L_dot * 1000.0f;
    if (vel_scaled < -2048.0f)
      vel_scaled = -2048.0f;
    if (vel_scaled > 2047.0f)
      vel_scaled = 2047.0f;
    int16_t vel_s12 = (int16_t)vel_scaled;

    // 3. Scale tau (tension) to 8-bit unsigned (range 0 to 255, 1.0 N per LSb).
    float tension_n = commands[i].tau;
    if (tension_n < 0.0f)
      tension_n = 0.0f;
    if (tension_n > 255.0f)
      tension_n = 255.0f;
    uint8_t tension_u8 = (uint8_t)tension_n;

    // 4. Bit-pack into 32-bit word:
    // Bits 0..11: len_u12
    // Bits 12..23: vel_s12
    // Bits 24..31: tension_u8
    uint32_t packed_val = 0;
    packed_val |= ((uint32_t)(len_u12 & 0xFFFU)) << 0;
    packed_val |= ((uint32_t)(vel_s12 & 0xFFFU)) << 12;
    packed_val |= ((uint32_t)tension_u8) << 24;
    
    // 5. Send the CAN message exactly like the Motor Driver format (4 bytes /
    // 32 bits) The updated CAN library handles the 11-bit ID creation
    // internally based on these arguments!
    HAL_StatusTypeDef status = CAN_Bus_SendU32(
        &CAN, motor_ids[i], CAN_ID_COMMAND, CAN_Priority_HIGH, packed_val);
    uint32_t active_buffer_mask = hfdcan1.LatestTxFifoQRequest;

    // Wait 2ms for the hardware transmission attempt to finish
    HAL_Delay(2);

    // Check if transmission completed successfully (ACKed) via TXBTO register
    bool acked = (hfdcan1.Instance->TXBTO & active_buffer_mask) != 0U;

    printf("  Motor %u: %s (ACK: %s)\r\n", motor_ids[i],
           (status == HAL_OK) ? "Queued OK" : "Queue FULL",
           acked ? "YES" : "NO");

    if (status != HAL_OK) {
      overall_status = status;
    }
  }

  return overall_status;
}

void run_ik_test(void) {
  printf("\r\n=== Running IK Trajectory Test ===\r\n");
  Vector3_t r_start = {0.46f, 0.56f, 0.7f};
  Vector3_t r_end = {0.50f, 0.3f, 0.30f};

  float T_MOVE = 10.0f;
  float FRAME_DT = 0.1f;

  total_steps = (int)(T_MOVE / FRAME_DT);
  if (total_steps > 150)
    total_steps = 150;

  Vector3_t r_out, r_dot_out;

  printf("1. Pre-calculating all trajectory steps...\r\n");

  uint32_t start_tick = HAL_GetTick();
  for (int step = 0; step <= total_steps; ++step) {
    float t = step * FRAME_DT;
    if (t > T_MOVE)
      t = T_MOVE;

    computeFrameTargetsWrapper(t, &r_start, &r_end, all_commands[step], &r_out,
                               &r_dot_out);
  }
  uint32_t elapsed_tick = HAL_GetTick() - start_tick;

  // Print details to the console now that timing is complete
  for (int step = 0; step <= total_steps; ++step) {
    float t = step * FRAME_DT;
    if (t > T_MOVE)
      t = T_MOVE;

    // We already calculated this, but we extract the position and velocity for
    // printing
    computeFrameTargetsWrapper(t, &r_start, &r_end, all_commands[step], &r_out,
                               &r_dot_out);

    printf("---- t = %.2f s (Step %d/%d) ----\r\n", t, step, total_steps);
    printf("Target Pos : [%.4f, %.4f, %.4f] m\r\n", r_out.x, r_out.y, r_out.z);
    printf("Target Vel : [%.4f, %.4f, %.4f] m/s\r\n", r_dot_out.x, r_dot_out.y,
           r_dot_out.z);

    printf("Cable Lens : ");
    for (int i = 0; i < 8; i++) {
      printf("%.4f ", all_commands[step][i].L_total);
    }

    printf("\r\nCable Vels : ");
    for (int i = 0; i < 8; i++) {
      printf("%+.4f ", all_commands[step][i].L_dot);
    }

    printf("\r\nTensions   : ");
    for (int i = 0; i < 8; i++) {
      printf("%.2f ", all_commands[step][i].tau);
    }
    printf("  [feasible=%s]\r\n\n",
           all_commands[step][0].feasible ? "true" : "false");
  }

  printf("===> Pure Kinematics Calculation Time: %lu ms <===\r\n",
         (unsigned long)elapsed_tick);

  printf("2. Calculations complete. Enabling continuous sending in main "
         "loop.\r\n");
  trajectory_ready = true;
}

static void PrintCanMessage(const char *prefix,
                            const CAN_BusMessage_t *message) {
  if ((prefix == NULL) || (message == NULL)) {
    return;
  }

  printf("%s device=0x%X msg=0x%X prio=0x%X dlc=%u data:", prefix,
         message->deviceId, message->messageId, message->priority,
         message->dlc);

  for (uint8_t i = 0; i < message->dlc; i++) {
    printf(" %02X", message->data[i]);
  }

  if (message->dlc >= 4U) {
    printf(" value_u32=%lu", (unsigned long)CAN_Bus_ReadU32(message));
  } else if (message->dlc >= 2U) {
    printf(" value_u16=%u", CAN_Bus_ReadU16(message));
  } else if (message->dlc >= 1U) {
    printf(" value_u8=%u", CAN_Bus_ReadU8(message));
  }

  printf("\r\n");
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *fdcan_handle,
                               uint32_t RxFifo0ITs) {
  if ((fdcan_handle == &CAN) &&
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)) {
    CAN_BusMessage_t message = {0};

    if (CAN_Bus_Receive(&CAN, &message) == HAL_OK) {
      PrintCanMessage("CAN RX", &message);

      switch (message.messageId) {
      case CAN_ID_ADC_REPORT: {
        uint32_t absolute_position = CAN_Bus_ReadU32(&message);
        printf(absolute_position ? "ADC value: %lu\r\n"
                                 : "Failed to read ADC value\r\n",
               (unsigned long)absolute_position);
        break;
      }
      case CAN_ID_STATUS: {
        char status = (char)CAN_Bus_ReadU8(&message);
        switch (status) {
        case 'R':
          printf("Resetting ....\r\n");
          HAL_NVIC_SystemReset();
          break;
        default:
          break;
        }
        break;
      }
      case CAN_ID_DEBUG:
        printf("Debug value: %lu\r\n",
               (unsigned long)CAN_Bus_ReadU32(&message));
        break;
      default:
        printf("Received message with unhandled ID: 0x%03lX\r\n",
               (unsigned long)message.messageId);
        break;
      }
    }
  }
}

void REQUEST_SEND_CAN(void) { can_send_pending = 1U; }

void SEND_CAN(void) {
  if (CAN_Bus_SendU32(&CAN, 0x0F, CAN_ID_DEBUG, CAN_Priority_LOW, canValue) !=
      HAL_OK) {
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    (void)HAL_FDCAN_GetProtocolStatus(&CAN, &protocol_status);
    printf("Failed to send CAN message err=0x%08lX lec=%lu bus_off=%lu\r\n",
           (unsigned long)HAL_FDCAN_GetError(&CAN),
           (unsigned long)protocol_status.LastErrorCode,
           (unsigned long)protocol_status.BusOff);
  } else {
    printf("Sent CAN message with value: %lu\r\n", (unsigned long)canValue);
    canValue++;
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Update SystemCoreClock variable according to RCC registers values. */
  SystemCoreClockUpdate();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  // Note: MX_USART1_UART_Init and MX_USART3_UART_Init must run first to enable
  // debugging output
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  printf("\r\n--- Main Application Starting ---\r\n");

  /* Print Clock Frequencies */
  printf("HSE Value (Assumed): %lu Hz\r\n", HSE_VALUE);
  printf("System Clock (SYSCLK): %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
  printf("AHB Clock (HCLK):      %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
  printf("APB1 Clock (PCLK1):    %lu Hz\r\n", HAL_RCC_GetPCLK1Freq());
  printf("APB2 Clock (PCLK2):    %lu Hz\r\n", HAL_RCC_GetPCLK2Freq());

  uint32_t sysclk_source = __HAL_RCC_GET_SYSCLK_SOURCE();
  if (sysclk_source == RCC_SYSCLKSOURCE_STATUS_HSI) {
    printf("SYSCLK Source: Internal HSI (64 MHz)\r\n");
  } else if (sysclk_source == RCC_SYSCLKSOURCE_STATUS_HSE) {
    printf("SYSCLK Source: External HSE (24 MHz)\r\n");
  } else if (sysclk_source == RCC_SYSCLKSOURCE_STATUS_PLLCLK) {
    uint32_t pll_src = READ_BIT(RCC->PLLCKSELR, RCC_PLLCKSELR_PLLSRC);
    if (pll_src == RCC_PLLSOURCE_HSI) {
      printf("SYSCLK Source: PLL1 (Sourced from Internal HSI)\r\n");
    } else if (pll_src == RCC_PLLSOURCE_HSE) {
      printf("SYSCLK Source: PLL1 (Sourced from External HSE)\r\n");
    } else {
      printf("SYSCLK Source: PLL1 (Sourced from HSI48/CSI)\r\n");
    }
  } else {
    printf("SYSCLK Source: Other\r\n");
  }

  printf("Initializing FDCAN1...\r\n");
  MX_FDCAN1_Init();
  printf("FDCAN1 Initialized.\r\n");

  /* printf("Initializing USB OTG HS...\r\n");
  MX_USB_OTG_HS_PCD_Init();
  printf("USB OTG HS Initialized.\r\n"); */

  printf("Initializing USB Device Stack (Full Speed)...\r\n");
  MX_USB_DEVICE_Init();
  printf("USB Device Stack Initialized.\r\n");

  /* USER CODE BEGIN 2 */
  printf("All peripherals initialized. Entering main loop...\r\n");
  printf("CAN init : ");
  printf(CAN_Bus_Init(&CAN, 0x00) ? "Failed\r\n" : "Success\r\n");

  /*
  printf("Initializing IK Geometry...\r\n");
  initGeometry();
  printf("IK Geometry Initialized.\r\n");

  run_ik_test();
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf(
      "\r\n=== UART Bridge Mode Active: PC (UART3) <-> ESP32 (UART1) ===\r\n");

  uint32_t last_test_send_tick = 0;
  MotorCommand_t dummy_cmds[4];
  for (int i = 0; i < 4; i++) {
    dummy_cmds[i].L_total = 1.0f + (i * 0.1f); // 1.0m, 1.1m, 1.2m, 1.3m
    dummy_cmds[i].L_dot = 0.0f;                // 0.0 m/s velocity
    dummy_cmds[i].tau = 10.0f * (i + 1);       // 10N, 20N, 30N, 40N
  }

  while (1) {
    /* Send test commands to the 4 motors periodically */
    if (HAL_GetTick() - last_test_send_tick >= 500) {
      last_test_send_tick = HAL_GetTick();
      HAL_StatusTypeDef status = SendMotorCommands(dummy_cmds);
      if (status == HAL_OK) {
        printf("Sent test commands to motors 1, 2, 4, 8 successfully\r\n");
      } else {
        FDCAN_ProtocolStatusTypeDef protocol_status = {0};
        (void)HAL_FDCAN_GetProtocolStatus(&CAN, &protocol_status);
        printf(
            "Failed to send test commands! err=0x%08lX lec=%lu bus_off=%lu\r\n",
            (unsigned long)HAL_FDCAN_GetError(&CAN),
            (unsigned long)protocol_status.LastErrorCode,
            (unsigned long)protocol_status.BusOff);
      }
    }

    /* Clear any UART errors on USART1 (ESP32) */
    if (USART1->ISR &
        (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
      USART1->ICR =
          USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
    }

    /* Clear any UART errors on USART3 (PC) */
    if (USART3->ISR &
        (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
      USART3->ICR =
          USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
    }

    /* Forward data from PC (USART3) to ESP32 (USART1) */
    if (USART3->ISR & USART_ISR_RXNE_RXFNE) {
      uint8_t byte = USART3->RDR;
      while (!(USART1->ISR & USART_ISR_TXE_TXFNF))
        ;
      USART1->TDR = byte;
    }

    /* Forward data from ESP32 (USART1) to PC (USART3) */
    if (USART1->ISR & USART_ISR_RXNE_RXFNE) {
      uint8_t byte = USART1->RDR;
      while (!(USART3->ISR & USART_ISR_TXE_TXFNF))
        ;
      USART3->TDR = byte;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void) {

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 3;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief XSPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_XSPI1_Init(void) {

  /* USER CODE BEGIN XSPI1_Init 0 */

  /* USER CODE END XSPI1_Init 0 */

  XSPIM_CfgTypeDef sXspiManagerCfg = {0};

  /* USER CODE BEGIN XSPI1_Init 1 */

  /* USER CODE END XSPI1_Init 1 */
  /* XSPI1 parameter configuration*/
  hxspi1.Instance = XSPI1;
  hxspi1.Init.FifoThresholdByte = 1;
  hxspi1.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi1.Init.MemoryType = HAL_XSPI_MEMTYPE_MICRON;
  hxspi1.Init.MemorySize = HAL_XSPI_SIZE_64MB;
  hxspi1.Init.ChipSelectHighTimeCycle = 1;
  hxspi1.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi1.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi1.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi1.Init.ClockPrescaler = 0;
  hxspi1.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_HALFCYCLE;
  hxspi1.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
  hxspi1.Init.MaxTran = 0;
  hxspi1.Init.Refresh = 0;
  hxspi1.Init.MemorySelect = HAL_XSPI_CSSEL_NCS1;
  if (HAL_XSPI_Init(&hxspi1) != HAL_OK) {
    Error_Handler();
  }
  sXspiManagerCfg.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
  sXspiManagerCfg.IOPort = HAL_XSPIM_IOPORT_1;
  sXspiManagerCfg.Req2AckTime = 1;
  if (HAL_XSPIM_Config(&hxspi1, &sXspiManagerCfg,
                       HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN XSPI1_Init 2 */

  /* USER CODE END XSPI1_Init 2 */
}

/**
 * @brief USB_OTG_HS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_HS_PCD_Init(void) {

  /* USER CODE BEGIN USB_OTG_HS_PCD_Init 0 */

  /* USER CODE END USB_OTG_HS_PCD_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_PCD_Init 1 */

  /* USER CODE END USB_OTG_HS_PCD_Init 1 */
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_HIGH;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
  hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  HAL_StatusTypeDef status = HAL_PCD_Init(&hpcd_USB_OTG_HS);
  if (status != HAL_OK) {
    printf("HAL_PCD_Init failed! Status: %d\r\n", status);
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_HS_PCD_Init 2 */

  /* USER CODE END USB_OTG_HS_PCD_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOP_CLK_ENABLE();
  __HAL_RCC_GPIOO_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(debugLED_GPIO_Port, debugLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : debugLED_Pin */
  GPIO_InitStruct.Pin = debugLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(debugLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EStop_Pin */
  GPIO_InitStruct.Pin = EStop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EStop_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("\r\n!!! ERROR_HANDLER CALLED - SYSTEM HALTING !!!\r\n");
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
