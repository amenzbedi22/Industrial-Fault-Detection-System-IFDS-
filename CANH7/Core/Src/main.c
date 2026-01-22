/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#define START_MARKER_1 0xAA
#define START_MARKER_2 0x55
#define END_MARKER_1   0x55
#define END_MARKER_2   0xAA
#define PACKET_SIZE    156
#define PAYLOAD_SIZE   152


#define OD_TEMP_INDEX        0x2000
#define OD_VIB_INDEX         0x2001
#define OD_PRESS_INDEX       0x2002
#define OD_FLOW_INDEX        0x2003
#define OD_CURRENT_INDEX     0x2004
#define OD_VOLTAGE_INDEX     0x2005

#define OD_FFT_TEMP_INDEX    0x2010
#define OD_FFT_VIB_INDEX     0x2011
#define OD_FFT_PRESS_INDEX  0x2012
#define OD_FFT_FLOW_INDEX   0x2013
#define SDO_INIT_UPLOAD     0x40
#define SDO_EXPEDITED_4B   0x43
#define SDO_EXPEDITED_NB   0x4F   // example for arrays (custom use)

#define NODE_ID_SERVER   0x01
#define NODE_ID_CLIENT   0x01

uint16_t index_glob = 0;
uint32_t offset = 0;


#pragma pack(push, 1)
typedef struct {
  float Temperature;
  float Vibration;
  float Pressure;
  float Flow_Rate;
  float Current;
  float Voltage;
  float FFT_Temperature[8];
  float FFT_Vibration[8];
  float FFT_Pressure[8];
  float FFT_Flow[8];
} DataPacket;
#pragma pack(pop)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t bytesToDLC(uint32_t size)
{
    if (size <= 8)  return size << 16;
    if (size <= 12) return FDCAN_DLC_BYTES_12;
    if (size <= 16) return FDCAN_DLC_BYTES_16;
    if (size <= 20) return FDCAN_DLC_BYTES_20;
    if (size <= 24) return FDCAN_DLC_BYTES_24;
    if (size <= 32) return FDCAN_DLC_BYTES_32;
    if (size <= 48) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch);
void processPacket(void);
void CANopen_SDO_Send_Float(uint8_t clientId, uint16_t index, uint8_t sub, float value);
void CANopen_SDO_Send_CHUNK(uint8_t clientId, float *array, uint32_t count);
void CANopen_SDO_Send_Array(uint8_t clientId, uint16_t index, uint8_t sub, float *array, uint32_t count);
void handle_CAN_request(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
/* USER CODE BEGIN PFP */
uint8_t rxBuffer[PACKET_SIZE];
DataPacket packet;
HAL_StatusTypeDef flag = HAL_ERROR ;
uint32_t totalSize = 156;
uint32_t maxFrameSize = 64;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_FDCAN2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  /* Configure Tx buffer message */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart2, rxBuffer, sizeof(rxBuffer));

  printf("UART interrupt ready. Waiting for data...\r\n");
    TxHeader.Identifier = 0x581;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_64;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0x0; // Ignore because FDCAN_NO_TX_EVENTS

    /* Configure standard ID reception filter to Rx buffer 0 */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
  #if 0
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL; // Ignore because FDCAN_FILTER_TO_RXBUFFER
  #endif
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
    sFilterConfig.FilterID1 = 0x601; // ID of Node1
  #if 0
    sFilterConfig.FilterID2 = 0x0; // Ignore because FDCAN_FILTER_TO_RXBUFFER
  #endif
    sFilterConfig.RxBufferIndex = 0;
    if((HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig)) != HAL_OK)
    {
      Error_Handler();
    }

    /* Start the FDCAN module */
    if((HAL_FDCAN_Start(&hfdcan2)) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
       {
         /* Notification Error */
         Error_Handler();
       }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 13;
  hfdcan2.Init.NominalTimeSeg1 = 86;
  hfdcan2.Init.NominalTimeSeg2 = 13;
  hfdcan2.Init.DataPrescaler = 2;
  hfdcan2.Init.DataSyncJumpWidth = 12;
  hfdcan2.Init.DataTimeSeg1 = 12;
  hfdcan2.Init.DataTimeSeg2 = 12;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 1;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_12;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void processPacket(void)
{
    // Check start/end markers
    if (rxBuffer[0] == START_MARKER_1 && rxBuffer[1] == START_MARKER_2 &&
        rxBuffer[154] == END_MARKER_1 && rxBuffer[155] == END_MARKER_2)
    {
        // Copy payload (152 bytes) into struct
        memcpy(&packet, &rxBuffer[2], PAYLOAD_SIZE);
        printf("Received %d bytes:\r\n", sizeof(packet));
        uint8_t *p = (uint8_t *)&packet;
        for (int i = 0; i < sizeof(packet); i++)
        {
            printf("%02X ", p[i]);
        }
        printf("\r\n");

        // Now you can access the data directly
        printf("Temperature: %.6f\r\n", packet.Temperature);
        printf("Vibration: %.6f\r\n", packet.Vibration);
        printf("Pressure: %.6f\r\n", packet.Pressure);
        printf("Flow Rate: %.6f\r\n", packet.Flow_Rate);
        printf("Current: %.6f\r\n", packet.Current);
        printf("Voltage: %.6f\r\n", packet.Voltage);

        printf("FFT_Temperature: ");
        for (int i = 0; i < 8; i++) printf("%.6f ", packet.FFT_Temperature[i]);
        printf("\r\n");

        printf("FFT_Vibration: ");
        for (int i = 0; i < 8; i++) printf("%.6f ", packet.FFT_Vibration[i]);
        printf("\r\n");

        printf("FFT_Pressure: ");
        for (int i = 0; i < 8; i++) printf("%.6f ", packet.FFT_Pressure[i]);
        printf("\r\n");

        printf("FFT_Flow: ");
        for (int i = 0; i < 8; i++) printf("%.6f ", packet.FFT_Flow[i]);
        printf("\r\n");
    }
    else
    {
        printf("Invalid markers!\r\n");
    }
}


void CANopen_SDO_Send_Float(uint8_t clientId, uint16_t index, uint8_t sub, float value)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t data[8] = {0};

    data[0] = SDO_EXPEDITED_4B;
    data[1] = index & 0xFF;
    data[2] = index >> 8;
    data[3] = sub;
    memcpy(&data[4], &value, sizeof(float));

    tx.Identifier = 0x580 + clientId;  // server → client
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.FDFormat = FDCAN_FD_CAN;
    tx.BitRateSwitch = FDCAN_BRS_ON;
    tx.DataLength = FDCAN_DLC_BYTES_8;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, data);
}


void CANopen_SDO_Send_CHUNK(uint8_t clientId, float *array, uint32_t count)
{
    uint8_t txData[8];
    FDCAN_TxHeaderTypeDef tx = {0};

    // CAN Tx configuration
    tx.Identifier = 0x580 + clientId;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.FDFormat = FDCAN_FD_CAN;
    tx.BitRateSwitch = FDCAN_BRS_ON;

    // Total bytes in array

        uint8_t n = count - offset;
        if (n > 7) n = 7; // max 7 bytes per segment

        txData[0] = 0x4F; // Initiate upload array

        memcpy(&txData[1],((uint8_t *)array) + offset, n);

        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, txData);

        offset += n;
        if (n < 7)
        {
            offset = 0 ;

        }
}


void CANopen_SDO_Send_Array(uint8_t clientId, uint16_t index, uint8_t sub,float *array, uint32_t count)
{
	index_glob = index;
	uint8_t data[8];
    FDCAN_TxHeaderTypeDef tx = {0};

    // ---- 1) Initiate upload response ----
    memset(data, 0, 8);
    data[0] = 0x41;             // Initiate upload array
    data[1] = index & 0xFF;
    data[2] = index >> 8;
    data[3] = sub;
    memcpy(&data[4], &count, 4);  // total number of bytes in array

    tx.Identifier = 0x580 + clientId; // SDO response ID
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.FDFormat = FDCAN_FD_CAN;
    tx.BitRateSwitch = FDCAN_BRS_ON;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, data);
}

void handle_CAN_request(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    uint8_t  cs  = rxData[0];
    uint16_t idx = rxData[1] | (rxData[2] << 8);
    uint8_t  sub = rxData[3];

    if (cs == SDO_INIT_UPLOAD)   // only SDO read
    {
    switch (idx)
    {
        // ---- Scalars ----
        case OD_TEMP_INDEX:
            CANopen_SDO_Send_Float(NODE_ID_CLIENT, idx, sub, packet.Temperature);
            break;

        case OD_VIB_INDEX:
            CANopen_SDO_Send_Float(NODE_ID_CLIENT, idx, sub, packet.Vibration);
            break;

        case OD_PRESS_INDEX:
            CANopen_SDO_Send_Float(NODE_ID_CLIENT, idx, sub, packet.Pressure);
            break;

        case OD_FLOW_INDEX:
            CANopen_SDO_Send_Float(NODE_ID_CLIENT, idx, sub, packet.Flow_Rate);
            break;

        case OD_CURRENT_INDEX:
            CANopen_SDO_Send_Float(NODE_ID_CLIENT, idx, sub, packet.Current);
            break;

        case OD_VOLTAGE_INDEX:
            CANopen_SDO_Send_Float(NODE_ID_CLIENT, idx, sub, packet.Voltage);
            break;

        // ---- FFT Temperature ----
        case OD_FFT_TEMP_INDEX:
            CANopen_SDO_Send_Array(NODE_ID_CLIENT, idx, sub,
                                   packet.FFT_Temperature,
                                   sizeof(packet.FFT_Temperature));

            break;

        // ---- FFT Vibration ----
        case OD_FFT_VIB_INDEX:
            CANopen_SDO_Send_Array(NODE_ID_CLIENT, idx, sub,
                                   packet.FFT_Vibration,
                                   sizeof(packet.FFT_Vibration));
            break;

        // ---- FFT Pressure ----
        case OD_FFT_PRESS_INDEX:
            CANopen_SDO_Send_Array(NODE_ID_CLIENT, idx, sub,
                                   packet.FFT_Pressure,
                                   sizeof(packet.FFT_Pressure));
            break;

        // ---- FFT Flow Rate ----
        case OD_FFT_FLOW_INDEX:
        	CANopen_SDO_Send_Array(NODE_ID_CLIENT, idx, sub,
                                   packet.FFT_Flow,
								   sizeof(packet.FFT_Flow));
            break;
        default:
            printf("Unknown SDO index 0x%04X\r\n", idx);
            break;
    }
   }
    else if ( cs == 0x4E){

    	switch (index_glob)
    	    {    	        // ---- FFT Temperature ----
    	        case OD_FFT_TEMP_INDEX:
    	            CANopen_SDO_Send_CHUNK(NODE_ID_CLIENT,
    	                                   packet.FFT_Temperature,
    	                                   sizeof(packet.FFT_Temperature));
    	            break;

    	        // ---- FFT Vibration ----
    	        case OD_FFT_VIB_INDEX:
    	        	CANopen_SDO_Send_CHUNK(NODE_ID_CLIENT,
    	                                   packet.FFT_Vibration,
    	                                   sizeof(packet.FFT_Vibration));
    	            break;

    	        // ---- FFT Pressure ----
    	        case OD_FFT_PRESS_INDEX:
    	        	CANopen_SDO_Send_CHUNK(NODE_ID_CLIENT,
    	                                   packet.FFT_Pressure,
    	                                   sizeof(packet.FFT_Pressure));
    	            break;
        	        // ---- FFT Flow Rate ----
        	    case OD_FFT_FLOW_INDEX:
        	    	CANopen_SDO_Send_CHUNK(NODE_ID_CLIENT,
        	                                packet.FFT_Flow,
        	                                sizeof(packet.FFT_Flow));
        	        break;

    	        default:
    	            printf("Unknown SDO index 0x%04X\r\n", idx);
    	            break;
    	    }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Print received data to PuTTY
        printf("Received %d bytes:\r\n", sizeof(rxBuffer));
        for (int i = 0; i < sizeof(rxBuffer); i++)
        {
            printf("%02X ", rxBuffer[i]);
        }
        printf("\r\n");
        processPacket();
        // Flush buffer and restart interrupt
        HAL_UART_Receive_IT(&huart2, rxBuffer, sizeof(rxBuffer));
    }
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t rxData[8]={};
        for (int i = 0; i < sizeof(rxData); i++)
         {
             printf("%02X ", rxData[i]);
         }
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rxData) == HAL_OK)
        {
            for (int i = 0; i < sizeof(rxData); i++)
             {
                 printf("%02X ", rxData[i]);
             }

            handle_CAN_request(&RxHeader, rxData);
            }
        }
    }
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
