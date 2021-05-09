#include "Trajectory.hpp"
#include "HexController.hpp"

#include "mbed.h"
#include "stm32f4xx.h"

// Memory requirements
// Buffer:
// 250Hz * 2s * 18joints * 2B/joint * 2buffers = 36kB

#define BUFFER_SIZE 40000
#define SERVO_FREQ 250
// #define SERVO_FREQ 1
#define JOINTS 18
// #define JOINTS 1
#define TRAJECTORY_TIME 1
// #define TRAJECTORY_TIME 1
#define HEADER_SIZE 3
#define JOINT_CMD_SIZE 2
#define TOTAL_CMD_NUMBER (TRAJECTORY_TIME * SERVO_FREQ)
#define SINGLE_CMD_SIZE (HEADER_SIZE + JOINTS * JOINT_CMD_SIZE)
#define TRAJECTORY_LENGTH (TOTAL_CMD_NUMBER * SINGLE_CMD_SIZE)

#define CMD_TRAJECTORY_NOW 42
#define CMD_TRAJECTORY_LOOP 43
#define CMD_CURRENT_STEP 23

#define WAIT_TIME_MS 1000 
DigitalOut led1(LED1);

TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef dma_uart6_tx;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef dma_spi3_rx;

volatile uint8_t maestroInit = 0xAA;
volatile uint8_t buffers[2][BUFFER_SIZE];
volatile uint8_t bufferUsed = 0;
volatile uint8_t bufferChanged = 0;
volatile uint32_t cmdCnt = 0;
volatile uint8_t changeBuffer = 0;
volatile uint8_t spiCmd;
volatile uint8_t receivingTrajectory = 0;

HexController hexController;

void MX_TIM6_Init() {
    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance = TIM6;
    htim6.Init.Period = 40 - 1;
    htim6.Init.Prescaler = 9000 - 1;
    htim6.Init.ClockDivision = 0;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.RepetitionCounter = 0;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6);

    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);
}

static void MX_USART6_UART_Init(void) {
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /**USART6 GPIO Configuration
    PC7      ------> USART6_RX
    PC6      ------> USART6_TX
    */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);

    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    NVIC_EnableIRQ(USART6_IRQn);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    __HAL_RCC_DMA2_CLK_ENABLE();

    dma_uart6_tx.Instance = DMA2_Stream6;
    dma_uart6_tx.Init.Channel = DMA_CHANNEL_5;
    dma_uart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_uart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    dma_uart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_uart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma_uart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma_uart6_tx.Init.Mode = DMA_NORMAL;
    dma_uart6_tx.Init.Priority = DMA_PRIORITY_HIGH;
    dma_uart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&dma_uart6_tx);

    __HAL_LINKDMA(&huart6, hdmatx, dma_uart6_tx);

    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

void MX_SPI3_Init() {
    // PB10 - SCK
    // PC1  - MOSI
    // PC2  - MISO
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_SLAVE;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi3);

    HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    static DMA_HandleTypeDef hdma_tx;
    static DMA_HandleTypeDef hdma_rx;

    GPIO_InitTypeDef  GPIO_InitStruct;

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /**SPI3 GPIO Configuration
    PA4      ------> SPI3_NSS
    PB0      ------> SPI3_MOSI
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*##-3- Configure the DMA streams ##########################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance                 = DMA1_Stream5;
    hdma_tx.Init.Channel             = DMA_CHANNEL_0;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
    hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdma_tx);   

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(hspi, hdmatx, hdma_tx);

    /* Configure the DMA handler for Transmission process */
    hdma_rx.Instance                 = DMA1_Stream2;
    hdma_rx.Init.Channel             = DMA_CHANNEL_0;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_NORMAL;
    hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
    hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4; 

    HAL_DMA_Init(&hdma_rx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(hspi, hdmarx, hdma_rx);

    /*##-4- Configure the NVIC for DMA #########################################*/ 
    /* NVIC configuration for DMA transfer complete interrupt (SPI3_TX) */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (SPI3_RX) */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);   
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    /*##-5- Configure the NVIC for SPI #########################################*/
    HAL_NVIC_SetPriority(SPI3_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

extern "C" {
void USART6_IRQHandler() {
    HAL_UART_IRQHandler(&huart6);
}

void DMA2_Stream6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma_uart6_tx);
}

void TIM6_DAC_IRQHandler() {
    HAL_TIM_IRQHandler(&htim6);
}

void SPI3_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi3);
}

void DMA1_Stream2_IRQHandler(void) {
    HAL_DMA_IRQHandler(hspi3.hdmarx);
}

void DMA1_Stream5_IRQHandler(void) {
    HAL_DMA_IRQHandler(hspi3.hdmatx);
}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (bufferChanged) {
        bufferUsed = (bufferUsed + 1) & 1;
        bufferChanged = 0;
        cmdCnt = 0;
    }

    volatile uint8_t *cmdPtr = buffers[bufferUsed] + cmdCnt * SINGLE_CMD_SIZE;
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *) cmdPtr, SINGLE_CMD_SIZE);

    if (cmdCnt + 1 < TOTAL_CMD_NUMBER) {
        ++cmdCnt;
    } else if (changeBuffer) {
        bufferUsed = (bufferUsed + 1) & 1;
        changeBuffer = 0;
        cmdCnt = 0;
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    led1 = !led1;
    if (spiCmd == CMD_CURRENT_STEP) {
        HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *) &cmdCnt, 1);
    }
    if (spiCmd == CMD_TRAJECTORY_NOW) {
        if (!receivingTrajectory) {
            volatile uint8_t *buffer = buffers[(bufferUsed + 1) & 1];
            receivingTrajectory = 1;
            HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) buffer, TRAJECTORY_LENGTH);
        } else {
            receivingTrajectory = 0;
            bufferChanged = 1;  // TODO make this interrupt higher priority than timer
            spiCmd = 0;
            HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) &spiCmd, 1);
        }
    }
    if (spiCmd == CMD_TRAJECTORY_LOOP) {
        if (!receivingTrajectory) {
            volatile uint8_t *buffer = buffers[(bufferUsed + 1) & 1];
            receivingTrajectory = 1;
            HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) buffer, TRAJECTORY_LENGTH);
        } else {
            receivingTrajectory = 0;
            changeBuffer = 1;
            spiCmd = 0;
            HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) &spiCmd, 1);
        }
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (spiCmd == CMD_CURRENT_STEP) {
        spiCmd = 0;
        HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) &spiCmd, 1);
    }
}

int main() {
    MX_USART6_UART_Init();

    // TODO set constant speed
    HAL_UART_Transmit_DMA(&huart6, (uint8_t*) &maestroInit, 1);

    MX_TIM6_Init();
    MX_SPI3_Init();

    HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) &spiCmd, 1);

    while (true) {
        thread_sleep_for(WAIT_TIME_MS);
    }
}
