
#include "board.h"
#include "stm32f1xx_hal.h"
#include <stdout.h>

static SPI_HandleTypeDef hspi;
static volatile uint32_t ticks;
static volatile uint16_t *ppm_frame_data;

static void spiInit(void);
static void timInit(void);
static void (*ppmFrameReady)(void);

void Error_Handler(char * file, int line){
  while(1){
  }
}

void laser4Init(void){
    spiInit();
    timInit();
}

void SPI_Write(uint8_t data){
  HAL_SPI_Transmit(&hspi, &data, 1, 10);
} 

uint8_t SPI_Read(void){
uint8_t data;
    HAL_SPI_Receive(&hspi, &data, 1, 10);
    return data;
}

void gpioInit(GPIO_TypeDef *port, uint8_t pin, uint8_t mode) {

    
    if(mode == GPI_PD){
        port->ODR &= ~(1 << pin);
    }

    if(mode == GPI_PU){
        port->ODR |= (1 << pin);
    }    

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
    /* Configure 0.5us time base with TIM1 for multiprotocol */
	TIM1->PSC = 35;								// 36-1;for 72 MHZ /0.5sec/(35+1)
	TIM1->ARR = 0xFFFF;							// Count until 0xFFFF
	TIM1->CCMR1 = (1<<4);	                    // Main scheduler
	TIM1->SR = 0x1E5F & ~TIM_SR_CC2IF;			// Clear Timer/Comp2 interrupt flag
	TIM1->DIER &= ~TIM_DIER_CC2IE;				// Disable Timer/Comp2 interrupt
	TIM1->EGR |= TIM_EGR_UG;					// Refresh the timer's count, prescale, and overflow
	TIM1->CR1 |= TIM_CR1_CEN;
    /* Configure 1ms intervals with TIM4 fot HAL  */
    TIM4->PSC = (SystemCoreClock/1000000) - 1; // Set Timer clock
    TIM4->ARR = 1000 - 1;
    TIM4->DIER = TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);
   
    /* Configure PPM input pin PB5*/
    gpioInit(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN, GPI_PU);
    /* Partial remap for TIM3 (PB5 -> TI2) */
    AFIO->MAPR &= ~(3 << 10);      // CLR TIM3_REMAP[1:0]
    AFIO->MAPR |= (2 << 10);       // Partial remap
    /* Configure TIM3 CH1 TI2 */
    PPM_TIM->CR1 = 0;              // Stop counter
    PPM_TIM->PSC = (SystemCoreClock/2000000) - 1;      // 0.5us    
    PPM_TIM->CCMR1 = (2 << 0);     // Map IC1 to TI2
    PPM_TIM->CCER  = (3 << 0);     // CH1 as input capture on falling edge;
    PPM_TIM->DIER = TIM_DIER_CC1IE;// Enable interrupt for CH1    
    NVIC_EnableIRQ(PPM_TIM_IRQn);  // Enable timer interupt
}

void delayMs(uint32_t ms){
uint32_t timeout = ticks + ms;
    while(ticks < timeout){        
    }
}

uint32_t getTick(void){    
    return ticks;    
}

uint32_t HAL_GetTick(void){ return getTick(); }

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

void setTimer(uint32_t interval, void(*cb)(void)){
    // PPM Mock timer
    TIM3->PSC = (SystemCoreClock/1000000) - 1;  // Set Timer clock 1MHz
    TIM3->ARR = interval - 1;
    TIM3->DIER = TIM_DIER_UIE;
    timer_callback = cb;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;
}

void stopTimer(){
    TIM3->CR1 &= ~TIM_CR1_CEN;
    timer_callback = NULL;
}


void enableWatchDog(uint32_t interval){
uint32_t timeout = 4096;
uint8_t pres = 0;

    interval *= 10;

    if(interval > 0xFFFF){
        interval = 0xFFFF;
    }    

    while( interval > timeout){
        timeout <<= 1;
        pres++;
    }

    if(IWDG->SR != 0){
        // other update is in progress
        return;
    }

    IWDG->KR = 0x5555; // Enable access to PR and RLR registers
    IWDG->PR = pres;
    IWDG->RLR = (interval * 0xFFFF) / timeout;
    IWDG->KR = 0xAAAA;  // Reload
    IWDG->KR = 0xCCCC;  // Start IWDG
}

void reloadWatchDog(void){
    IWDG->KR = 0xAAAA; // Reload RLR on counter
}

void ppmSetReadyAction(volatile uint16_t *buf, void(*cb)(void)){
    if(cb == NULL || buf == NULL ){
        return;
    }
    PPM_TIM->CR1 &= ~TIM_CR1_CEN;   // Stop counter
    ppmFrameReady = cb;
    ppm_frame_data = buf;
    PPM_TIM->CR1 |= TIM_CR1_CEN;   // Start counter
    return;
    // attach interrupt
    AFIO->EXTICR[1] = ( 1 << 4);        // PB5 -> EXTI5
    EXTI->IMR = ( 1 << 5);              // MR5
    EXTI->FTSR = (1 << 5);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
 * @brief Interrupts handlers
 * */
void EXTI9_5_IRQHandler(void){
uint32_t pr = EXTI->PR;
    if((pr & EXTI_PR_PR5) != 0){        
        ppmFrameReady();
    }
    EXTI->PR = pr;
}
void TIM4_IRQHandler(void){
    TIM4->SR = ~TIM4->SR;
    ticks++;
    //DBG_PIN_TOGGLE;
}
/**
 * This code is based on multiprotocol PPM_Decode function, but instead of interrups 
 * it uses the timer capture module.
 * The capture is performed in every falling edge of the pin
 * and measured the pulse duration.
 * 
 * _______    _________    ______________
 *        |  |         |  |
 *        |  |         |  |
 *         --           --
 *        ^            ^
  * */
RAM_CODE void PPM_TIM_IRQHandler(void){
static uint8_t channel_counter = 0, frame_ok = 0;
static uint16_t last_time = 0;
uint16_t cur_time;

    if(PPM_TIM->SR & TIM_SR_CC1IF){
        PPM_TIM->SR &= ~(TIM_SR_CC1IF);
        cur_time = PPM_TIM->CCR1 - last_time;
//DBG_PIN_TOGGLE;
        if(cur_time < 1600){ //PPM_MIN_PULSE){
            frame_ok = 0;
        }else if(cur_time > 4400){ //PPM_MAX_PULSE){
            // Start of frame
            if(channel_counter >= MIN_RADIO_CHANNELS){
                ppmFrameReady();
            }
            channel_counter = 0;
            frame_ok = 1;
        }else if(frame_ok){
            ppm_frame_data[channel_counter] = cur_time;
            if(channel_counter++ >= PPM_MAX_CHANNELS){
                frame_ok = 0;
            }
        }        
        last_time += cur_time;
        }
    }

