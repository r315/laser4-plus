
#include "board.h"

static SPI_HandleTypeDef hspi;
volatile uint32_t ticks;

static void spiInit(void);
static void timInit(void);
static void randomInit(void);

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
    randomInit();
}

void SPI_Write(uint8_t data){
  HAL_SPI_Transmit(&hspi, &data, 1, 10);
} 

uint8_t SPI_Read(void){
uint8_t data;
    HAL_SPI_Receive(&hspi, &data, 1, 10);
    return data;
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
 *          using timer4
 * 
 * */
static void timInit(void){

    ticks = 0;
    
    RCC->APB1ENR  |= RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN;
    RCC->APB1RSTR |= RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN;
    RCC->APB1RSTR &= ~(RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN);

    TIM4->PSC = (SystemCoreClock/1000000) - 1; // Set Timer clock
    TIM4->ARR = 1000 - 1;
    TIM4->DIER = TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);

	TIM2->PSC = 35;								// 36-1;for 72 MHZ /0.5sec/(35+1)
	TIM2->ARR = 0xFFFF;							// Count until 0xFFFF
	TIM2->CCMR1 = (1<<4);	                    // Main scheduler
	TIM2->SR = 0x1E5F & ~TIM_SR_CC2IF;			// Clear Timer2/Comp2 interrupt flag
	TIM2->DIER &= ~TIM_DIER_CC2IE;				// Disable Timer2/Comp2 interrupt
	TIM2->EGR |= TIM_EGR_UG;					// Refresh the timer's count, prescale, and overflow
	TIM2->CR1 |= TIM_CR1_CEN;

    // PPM Mock timer
    TIM3->PSC = (SystemCoreClock/1000000) - 1;  // Set Timer clock
    TIM3->ARR = 6000 - 1;                       // PPM 4ch*1500us 
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);
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
 * @brief Pseudo random number generator initialization
 * */
static void randomInit(void){
    /*uint32_t seed=0;
		for(uint8_t i=0;i<4;i++)
			seed=(seed<<8) | (analogRead(PB0)& 0xFF);
		randomSeed(seed); */
}

uint8_t eeprom_data[256];

uint16_t BOARD_EEPROM_Write(uint16_t address, uint8_t *data, uint16_t len){
    for (uint16_t i = 0; i < len; i++)
    {
        eeprom_data[address + i] = data[i];        
    }    
    return len;
}

uint16_t BOARD_EEPROM_Read(uint16_t address, uint8_t *data, uint16_t len){
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = eeprom_data[address + i];
    }
    
    return len;
}