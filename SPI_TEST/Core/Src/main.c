#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;
/* DMA handle declarations */
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;
/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
void debug_send(const char *message);
void spi_write_chopconf(uint32_t data);
void spi_read_chopconf(void);
void select_tmc5160(uint8_t select);
static void MX_DMA_Init(void);
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();  // Initialize DMA
    MX_SPI2_Init();
    MX_USART2_UART_Init();

    while (1)
    {
        // Write a configuration value to the CHOPCONF register
        uint32_t chopconf_value = 0x00100000;  // Example configuration value
        debug_send("Writing CHOPCONF register...\n");
        spi_write_chopconf(chopconf_value);

        HAL_Delay(1000);  // Short delay before reading back

        // Read the CHOPCONF register
        debug_send("Reading CHOPCONF register...\n");
        spi_read_chopconf();

        HAL_Delay(1000);  // Wait before the next transaction
    }
}
/* DMA Initialization */
static void MX_DMA_Init(void)
{
    /* Enable DMA1 clock */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Configure DMA for SPI2 TX */
    hdma_spi2_tx.Instance = DMA1_Channel5;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode = DMA_NORMAL;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure DMA for SPI2 RX */
    hdma_spi2_rx.Instance = DMA1_Channel4;
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_rx.Init.Mode = DMA_NORMAL;
    hdma_spi2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    /* Configure DMA for SPI2 RX */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    /* Link the DMA handles with SPI2 */
    __HAL_LINKDMA(&hspi2, hdmatx, hdma_spi2_tx);
    __HAL_LINKDMA(&hspi2, hdmarx, hdma_spi2_rx);
}
/* SPI Write CHOPCONF Function (Single Transaction) */
void spi_write_chopconf(uint32_t data)
{
    uint8_t txData[5] = {0xEC, (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF};
    uint8_t rxData[5] = {0};

    select_tmc5160(1);  // Select TMC5160 (CS1)

    if (HAL_SPI_TransmitReceive_DMA(&hspi2, txData, rxData, sizeof(txData)) != HAL_OK)
    {
        Error_Handler();
    }

    /* Wait for the SPI transmission to complete */
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {}

    select_tmc5160(0);  // Deselect TMC5160

    char debug_msg[50];
    snprintf(debug_msg, sizeof(debug_msg), "Status Byte from Write: 0x%02X\n", rxData[0]);
    debug_send(debug_msg);
}


/* SPI Read CHOPCONF Function (Two-Transaction Read) */
void spi_read_chopconf(void)
{
    uint8_t txData[5] = {0x6C, 0x00, 0x00, 0x00, 0x00};  // Read command
    uint8_t dummyData[5] = {0x00, 0x00, 0x00, 0x00, 0x00};  // Dummy data
    uint8_t rxData[5] = {0};

    // First transaction
    select_tmc5160(1);
    if (HAL_SPI_TransmitReceive_DMA(&hspi2, txData, rxData, sizeof(txData)) != HAL_OK)
    {
        Error_Handler();
    }
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {}
    select_tmc5160(0);

    char debug_msg[50];
    snprintf(debug_msg, sizeof(debug_msg), "Status Byte: 0x%02X\n", rxData[0]);
    debug_send(debug_msg);

    HAL_Delay(10);

    // Second transaction (read actual data)
    select_tmc5160(1);
    if (HAL_SPI_TransmitReceive_DMA(&hspi2, dummyData, rxData, sizeof(dummyData)) != HAL_OK)
    {
        Error_Handler();
    }
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {}
    select_tmc5160(0);

    snprintf(debug_msg, sizeof(debug_msg), "Received CHOPCONF: 0x%02X%02X%02X%02X\n",
             rxData[1], rxData[2], rxData[3], rxData[4]);
    debug_send(debug_msg);
}


/* Function to select/deselect TMC5160 (CS1) */
void select_tmc5160(uint8_t select)
{
    if (select) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);  // Select (CS low)
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);    // Deselect (CS high)
    }
}

/* Debug Send Function */
void debug_send(const char *message)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

/* SPI2 Initialization */
static void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;  // CPOL = 1
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;       // CPHA = 1
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;

    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART2 Initialization */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

    /*Configure GPIO pin : CS1 Pin (GPIOC4) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Error Handler */
void Error_Handler(void)
{
    while (1)
    {
        // Stay here if there's an error
    }
}
