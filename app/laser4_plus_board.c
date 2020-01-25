
#include "board.h"
#include "stm32f1xx_hal.h"
#include <stdout.h>


static SPI_HandleTypeDef hspi;
static volatile uint32_t ticks;
static void (*pinIntCB)(void);
static volatile uint16_t adc_result;

static void spiInit(void);
static void timInit(void);
static void adcInit(void);
static void encInit(void);
static void ppmOutInit(void);

void Error_Handler(char * file, int line){
  while(1){
  }
}

void laser4Init(void){
    GPIO_ENABLE;
    AFIO->MAPR = (2 << 24); // SW-DP Enabled
    DBG_PIN_INIT;
    CC25_CS_INIT;
    HW_BIND_BUTTON_INIT;
    LED_INIT;
    HW_SW_INIT;
    HW_TX_35MHZ_EN_INIT;
    
    spiInit();
    timInit();
    adcInit();
    encInit();
    ppmOutInit();
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
        port->BRR = (1 << pin);
    }

    if(mode == GPI_PU){
        port->BSRR = (1 << pin);
    }    

    mode &= 0x0f;

    if(pin <  8){ 
        port->CRL = (port->CRL & ~(15 << (pin << 2))) | (mode << (pin << 2));
    }else{ 
        port->CRH = (port->CRH & ~(15 << ((pin - 8) << 2))) | (mode << ((pin - 8) << 2)); 
    }
}

void gpioAttachInterrupt(GPIO_TypeDef *port, uint8_t pin, uint8_t edge, void(*cb)(void)){
    if(cb == NULL){
        return;
    }
    pinIntCB = cb;
    AFIO->EXTICR[1] = ( 1 << 4);        // PB5 -> EXTI5
    EXTI->IMR  = ( 1 << 5);             // MR5
    EXTI->FTSR = (1 << 5);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void gpioRemoveInterrupt(GPIO_TypeDef *port, uint8_t pin){
    NVIC_DisableIRQ(EXTI9_5_IRQn);
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
    /* Configure 1ms timer*/
#if NO_SYS_TICK
    TIM4->PSC = (SystemCoreClock/1000000) - 1; // Set Timer clock
    TIM4->ARR = 1000 - 1;
    TIM4->DIER = TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);
#else
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);
#endif   
    /* Configure 0.5us time base for multiprotocol */
    TIMER_BASE->CR1 = 0;                               // Stop counter
    TIMER_BASE->PSC = (SystemCoreClock/2000000) - 1;	// 36-1;for 72 MHZ /0.5sec/(35+1)
    TIMER_BASE->ARR = 0xFFFF;							// Count until 0xFFFF
    TIMER_BASE->CCMR1 = (1<<4);	                    // Main scheduler
	TIMER_BASE->SR = 0x1E5F & ~TIM_SR_CC1IF;			// Clear Timer/Comp2 interrupt flag
    TIMER_BASE->DIER = 0;               				// Disable Timer/Comp2 interrupts
    TIMER_BASE->EGR |= TIM_EGR_UG;					    // Refresh the timer's count, prescale, and overflow
    TIMER_BASE->CR1 |= TIM_CR1_CEN;                    // Enable counter
}

void delayMs(uint32_t ms){
uint32_t timeout = ticks + ms;
    while(ticks < timeout){        
    }
}

uint32_t getTick(void){ return ticks; }
uint32_t HAL_GetTick(void){ return getTick(); }

/**
 * @brief Flash write functions for EEPROM emulation
 */
uint32_t flashWrite(uint8_t *dst, uint8_t *data, uint16_t count){
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

/**
 * @brief Erase 1k selctor on flash
 * 
 * @param address:  start address for erasing
 * @return : 0 on fail
 * */
uint32_t flashPageErase(uint32_t address){
uint32_t res;

    res = HAL_FLASH_Unlock();

    if( res == HAL_OK){
        FLASH_PageErase(address);
    }
    
    HAL_FLASH_Lock();
    return 1;
}

/**
 * @brief Configure watchdog timer according a given interval
 *  in wich the timer will expire and a system reset is performed
 * 
 * @param interval : Interval in wich the watchdog will perform a system reset
 * */
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
/**
 * @brief Watchdog reset that mus be called before the interval
 *          specified on configuration
 * 
 * */
void reloadWatchDog(void){
    IWDG->KR = 0xAAAA; // Reload RLR on counter
}

/**
 * @brief Read digital switches values
 * 
 * @return : bitmask with active switches
 * */
uint32_t readSwitches(void){
uint16_t state = 0;

    state = (HW_SW_AUX1_VAL << 0) | (HW_SW_AUX2_VAL << 1) | (HW_SW_AUX3_VAL << 2);

    return state;
}
/**
 * @brief Configure ADC for a defined channel in interrupt mode and initiates a convertion.
 *  After convertion the result is stored locally through interrupt
 * 
 * 
 * @param ch : Channel to perform convertion
 * */
static void adcInit(void){
    RCC->APB2ENR  |= RCC_APB2ENR_ADC1EN;     // Enable Adc1
    RCC->APB2RSTR |= RCC_APB2ENR_ADC1EN;
    RCC->APB2RSTR &= ~RCC_APB2ENR_ADC1EN;

    ADC1->CR2 = (15 << 17) | ADC_CR2_ADON;     // Enable ADC, trigger by software
    ADC1->CR1 = ADC_CR1_EOCIE;    // Enable end of convertion interrupt
    ADC1->SMPR2 = (4 << (3 * HW_VBAT_CHANNEL));   // Sample time = 41.5 cycles, AN9-0
    NVIC_EnableIRQ(ADC1_IRQn);
    ADC1->SQR3 = HW_VBAT_CHANNEL;   // Configure channel for first conversion on sequence 
    ADC1->CR2 |= ADC_CR2_SWSTART; 

    gpioInit(GPIOA, HW_VBAT_CHANNEL, GPI_ANALOG);
}

/**
 * @brief Return the value os the last convertion and
 * initiates a new convertion.
 * 
 * */
uint16_t readAdcChannel(void){
    return 0;
}

/**
 * @brief Read battery voltage, a new battery 
 *      measurement is performed on function call exit
 * 
 * @return : battery voltage in mV
 * */
uint32_t readBatteryVoltage(void){
uint32_t result;
    // Calculate voltage
    result = adc_result * (4096.0 / ADC_VREF);
    // Start a new convertion
    ADC1->CR2 |= ADC_CR2_SWSTART;
    return result;
}

/**
 * @brief Rotary encorder init
 *  Configures a timer as pulse counter, the counter is incremented/decremented
 *  on edges from the two signals from the encoder
 * 
 * Using TIM2 CH1 and CH2 as TI1 and TI2, also filter is configured
 * */
void encInit(void){
    gpioInit(GPIOB, 3, GPI_PU);
    gpioInit(GPIOA, 15, GPI_PU);
    AFIO->MAPR = (AFIO->MAPR & ~(3 << 8)) | (1 << 8);       // Partial remap for TIM2; PA15 -> CH1, PB3 -> CH2 

    ENC_TIM->CR2 = 
    ENC_TIM->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;        // External clock, Encoder mode 3
    ENC_TIM->CCMR1 = (15 << 12) | (15 << 4)                 // Map TIxFP1 to TIx,
                  | TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0     // and max length if input filter
                  | TIM_CCMR1_IC2PSC_1 | TIM_CCMR1_IC1PSC_1;
    ENC_TIM->CCER = 0;                                      // Falling polarity
    ENC_TIM->CR1 = TIM_CR1_CEN;
    ENC_TIM->SR = 0;
    
}

/**
 * @brief Generates PPM signal for 6 channels.
 * One extra channel is required in order to produce the last
 * pulse of the last channel. Another channel is added just to avoid wasting
 * cpu cycles waiting for the transmission of the last channel, this way the DMA
 * transfer complete interrupt doesn't stop the timer in the middle of the transmission.
 * 
 * @param data : pointer to the six channels data
 * 
 * */
void ppmOut(uint16_t *data){
static uint16_t ppm_data[MAX_PPM_CHANNELS + 2];
    // Copy channel data to temp buffer
    for (uint16_t i = 0; i < MAX_PPM_CHANNELS; i++){
        ppm_data[i] = data[i];
    }
    // Set extra channels to end ppm signal
    ppm_data[MAX_PPM_CHANNELS] = PPM_MAX_PERIOD;
    ppm_data[MAX_PPM_CHANNELS + 1] = PPM_MAX_PERIOD;
    // Force counter update and DMA request, setting the period here
    // will produce a rising edge, but as the ppm line should be high
    // we get only the initial ppm low pulse.
    PPM_TIM->ARR = ppm_data[0];
    PPM_TIM->EGR = TIM_EGR_UG;
    // Configure DMA transfer, the first transfer will have the value on ppm_data[0]
    // since this value was transferred to produce initial pulse, it is necessary to send it again
    // to generate the channel time.
    DMA1_Channel7->CMAR = (uint32_t)(ppm_data);
    DMA1_Channel7->CNDTR = MAX_PPM_CHANNELS + 2;
    DMA1_Channel7->CCR |= DMA_CCR_EN;   
    // Resume timer
    PPM_TIM->CR1 |= TIM_CR1_CEN;
    DBG_PIN_HIGH;
}

/**
 * @brief PPM output generation init
 * */
void ppmOutInit(void){
    gpioInit(GPIOB, 7, GPO_AF | GPO_2MHZ);

     /* Configure DMA Channel1*/
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;               // Enable DMA1
    DMA1_Channel7->CPAR = (uint32_t)&PPM_TIM->ARR;  // Destination peripheral
    DMA1_Channel7->CCR =
            DMA_CCR_PL |                            // Highest priority
            DMA_CCR_MSIZE_0 |                       // 16bit Dst size
            DMA_CCR_PSIZE_0 |                       // 16bit src size
            DMA_CCR_DIR |                           // Read from memory
            DMA_CCR_MINC |                          // increment memory pointer after transference
            DMA_CCR_TCIE;                           // Enable end of transfer interrupt
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    PPM_TIM->CR1 =  TIM_CR1_DIR | TIM_CR1_ARPE;
    PPM_TIM->PSC = (SystemCoreClock/2000000) - 1;	// 36-1;for 72 MHZ /0.5sec/(35+1)
    PPM_TIM->CCMR1 = (7 << 12);                     // PWM mode 2
    PPM_TIM->CCER = TIM_CCER_CC2E;                  // Enable channel
    // Force high state
    PPM_TIM->ARR = PPM_MAX_PERIOD;
    PPM_TIM->CNT = PPM_MAX_PERIOD;
    PPM_TIM->CCR2 = PPM_PULSE_WIDTH;
    // Enable DMA Request 
    PPM_TIM->DIER |= TIM_DIER_UDE; 
}

/**
 * @brief Interrupts handlers
 * */
void EXTI9_5_IRQHandler(void){
uint32_t pr = EXTI->PR;
    if((pr & EXTI_PR_PR5) != 0){        
        pinIntCB();
    }
    EXTI->PR = pr;
}

#ifdef NO_SYS_TICK
void TIM4_IRQHandler(void){
    TIM4->SR = ~TIM4->SR;
    ticks++;
    //DBG_PIN_TOGGLE;
}
#else
void SysTick_Handler(void){
    ticks++;
}
#endif

void ADC1_IRQHandler(void){
    adc_result = (uint16_t)ADC1->DR;
}

void DMA1_Channel7_IRQHandler(void){
    if(DMA1->ISR & DMA_ISR_TCIF7){
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;
        // As two extra channels were send,
        // we end up here when transfering the first extra channel.
        // As the timer is stoped we get only a rising edge due to update event
        // and cancel the last channel transmission.
        PPM_TIM->CR1 &= ~TIM_CR1_CEN;
        DBG_PIN_LOW;        
    }
    DMA1->IFCR |= DMA_IFCR_CGIF7;  // Clear DMA Flags TODO: ADD DMA Error handling ?
}