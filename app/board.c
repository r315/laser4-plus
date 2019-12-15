
#include "board.h"

static SPI_HandleTypeDef hspi;
volatile uint32_t ticks;

static void spiInit(void);
static void timInit(void);

void Error_Handler(char * file, int line){
  while(1){
  }
}

void BOARD_Init(void){
    GPIO_ENABLE;
    DBG_LED_INIT;
    CC25_CS_INIT;    
    spiInit();
    timInit();
}
}

void SPI_Write(uint8_t data){
  HAL_SPI_Transmit(&hspi, &data, 1, 10);
} 

uint8_t SPI_Read(void){
    return 0;
}

void BOARD_GPO_Init(GPIO_TypeDef *port, uint8_t pin) {
    if(pin <  8){ 
        port->CRL = (port->CRL & ~(15 << (pin << 2))) | (GPIO_MODE_OUT << (pin << 2));
    }else{ 
        port->CRH = (port->CRH & ~(15 << ((pin - 8) << 2))) | (GPIO_MODE_OUT << ((pin - 8) << 2)); 
    }
}

/**
 * 
 */
static void spiInit(){

    __HAL_RCC_SPI2_CLK_ENABLE();
    hspi.Instance = SPI2;
    hspi.Init.Mode = SPI_MODE_MASTER;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    //hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    //hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi.Init.NSS = SPI_NSS_SOFT;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi) != HAL_OK){
        Error_Handler(__FILE__, __LINE__);
    }

    SPI_PINS_INIT;
}

/**
 * @brief Initialyze 1ms general purpose time base
 * 
 * */
static void timInit(void){
TIM_TypeDef *tim = TIM4;
    ticks = 0;
    
    RCC->APB1ENR  |= RCC_APB1ENR_TIM4EN;    // Enable Timer 4
    RCC->APB1RSTR |= RCC_APB1ENR_TIM4EN;    // Reset timer registers
    RCC->APB1RSTR &= ~RCC_APB1ENR_TIM4EN;

    tim->CR1 = 0; //TIM_CR1_DIR;
    tim->PSC = (SystemCoreClock/1000000) - 1; // Set Timer clock
    tim->ARR = 1000 - 1;
    tim->DIER = TIM_DIER_UIE;
    tim->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM4_IRQn);

}

uint32_t HAL_GetTick(void){
//uint32_t BOARD_GetTick(void){    
    return ticks;    
}

void TIM4_IRQHandler(void){
    TIM4->SR = ~TIM4->SR;
    ticks++;
    //DBG_PIN_TOGGLE;
}

