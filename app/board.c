
#include "board.h"
#include "stm32f1xx_hal.h"
#include <stdout.h>

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
    HW_BIND_BUTTON_INIT;
    spiInit();
    timInit();
#ifdef ENABLE_USART
    usart_init();
#endif    
}

void SPI_Write(uint8_t data){
  HAL_SPI_Transmit(&hspi, &data, 1, 10);
} 

uint8_t SPI_Read(void){
uint8_t data;
    HAL_SPI_Receive(&hspi, &data, 1, 10);
    return data;
}

void BOARD_GPIO_Init(GPIO_TypeDef *port, uint8_t pin, uint8_t mode) {

    mode &= 0x0f;

    if(pin <  8){ 
        port->CRL = (port->CRL & ~(15 << (pin << 2))) | (mode << (pin << 2));
    }else{ 
        port->CRH = (port->CRH & ~(15 << ((pin - 8) << 2))) | (mode << ((pin - 8) << 2)); 
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
 *          using timer4
 * 
 * */
static void timInit(void){

    ticks = 0;
    
    RCC->APB1ENR    |= RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR    |= RCC_APB2ENR_TIM1EN;
    RCC->APB1RSTR   |= RCC_APB1RSTR_TIM4RST | RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR   &= ~(RCC_APB1RSTR_TIM4RST | RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST);
    RCC->APB2RSTR   |= RCC_APB2RSTR_TIM1RST;
    RCC->APB2RSTR   &= ~RCC_APB2RSTR_TIM1RST;
    /* Configure 1ms intervals with TIM4 fot HAL  */
    TIM4->PSC = (SystemCoreClock/1000000) - 1; // Set Timer clock
    TIM4->ARR = 1000 - 1;
    TIM4->DIER = TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);
    /* Configure 0.5us time base with TIM1 for multiprotocol */
	TIM1->PSC = 35;								// 36-1;for 72 MHZ /0.5sec/(35+1)
	TIM1->ARR = 0xFFFF;							// Count until 0xFFFF
	TIM1->CCMR1 = (1<<4);	                    // Main scheduler
	TIM1->SR = 0x1E5F & ~TIM_SR_CC2IF;			// Clear Timer/Comp2 interrupt flag
	TIM1->DIER &= ~TIM_DIER_CC2IE;				// Disable Timer/Comp2 interrupt
	TIM1->EGR |= TIM_EGR_UG;					// Refresh the timer's count, prescale, and overflow
	TIM1->CR1 |= TIM_CR1_CEN;

#ifdef MOCK_PPM
    // PPM Mock timer
    TIM3->PSC = (SystemCoreClock/1000000) - 1;  // Set Timer clock
    TIM3->ARR = 6000 - 1;                       // PPM 4ch*1500us 
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);
#endif
}

void BOARD_DelayMs(uint32_t ms){
uint32_t timeout = ticks + ms;
    while(ticks < timeout){        
    }
}

uint32_t BOARD_GetTick(void){    
    return ticks;    
}

void TIM4_IRQHandler(void){
    TIM4->SR = ~TIM4->SR;
    ticks++;
    //DBG_PIN_TOGGLE;
}

uint32_t HAL_GetTick(void){ return BOARD_GetTick(); }

/**
 * @brief Flash write functions for EEPROM emulation
 */
uint32_t flash_write(uint8_t *dst, uint8_t *data, uint16_t count){
uint16_t *src = (uint16_t*)data;
uint32_t res, address = (uint32_t)dst;

    res = HAL_FLASH_Unlock();
    if( res == HAL_OK){    
        for (uint16_t i = 0; i < count; i+= 2, src++){
            res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, *src);
            if(res != HAL_OK){
                break; 
            }
        }
    }
    HAL_FLASH_Lock();
    return res;
}