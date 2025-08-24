
#include "board.h"
#include "nvdata.h"
#include "stm32f1xx_hal.h"
#include "usbd_conf.h"
#include "debug.h"

#ifdef ENABLE_DEBUG_BOARD
#define DBG_TAG     "[BOARD]: "
#define DBG_BOARD_INF DBG_INF
#define DBG_BOARD_WRN DBG_WRN
#define DBG_BOARD_ERR DBG_ERR
#else
#define DBG_TAG     ""
#define DBG_BOARD_INF(...)
#define DBG_BOARD_WRN(...)
#define DBG_BOARD_ERR(...)
#endif

typedef struct {
    volatile uint16_t status;
    uint16_t result[2];
    uint32_t calibration_code;
    float resolution;
    float vdiv_racio;
    float sense_resistor;
    uint32_t battery_voltage;
    uint32_t battery_current;
}adc_t;


typedef struct {
        uint32_t time;
        uint32_t count;
        uint32_t status;
        void (*action)(void);
}swtimer_t;

swtimer_t timers[SWTIM_NUM];

typedef struct {
    tone_t *ptone;
    tone_t tone;
    volatile uint32_t status;
}sound_t;


// Private variables
static SPI_HandleTypeDef hspi;
static volatile uint32_t ticks;
static adc_t hadc;
static void (*pinIntCB)(void);

#ifdef ENABLE_DISPLAY
I2C_HandleTypeDef hi2c2;
static void i2cInit(void);
#endif

// Private functions
static void spiInit(void);
static void timInit(void);
static void adcInit(void);
static void encInit(void);
static void crcInit(void);
#ifdef ENABLE_PPM
static void ppmOutInit(void);
#endif

#ifdef ENABLE_BUZZER
static sound_t hbuz;
static void buzInit(void);
#endif

#ifdef ENABLE_USART
static serialbus_t uartbus;

int serial_available(void) { return UART_Available(&uartbus); }
int serial_read(char *buf, int len) { return UART_Read(&uartbus, (uint8_t*)buf, len); }
int serial_write(const char *buf, int len) { return UART_Write(&uartbus, (const uint8_t*)buf, len); }

stdinout_t pcom = {
    .available = serial_available,
    .read = serial_read,
    .write = serial_write
};

#endif

extern void setup(void);
extern void loop(void);

int main(void)
{
    laser4Init();

    setup();

    while(1){
        loop();
    }

    return 0;
}

// Functions implemenation
void Error_Handler(char * file, int line)
{
    while(1){
    }
}

void laser4Init(void){
    GPIO_ENABLE;
    AFIO->MAPR = (2 << 24); // SW-DP Enabled
    CC25_CS_INIT;
    LED_INIT;
    HW_SW_INIT;
    HW_TX_35MHZ_EN_INIT;

    spiInit();
    timInit();
    adcInit();
    encInit();
#ifdef ENABLE_PPM
    ppmOutInit();
#endif

#ifdef ENABLE_BUZZER
    buzInit();
#endif
    crcInit();
#ifdef ENABLE_USART
    uartbus.bus = UART_BUS0;
    uartbus.speed = 115200;
    UART_Init(&uartbus);
#endif

#ifdef ENABLE_DISPLAY
    i2cInit();
    LCD_Init();
    //LCD_Fill(0, 0, 128, 32, BLACK);
    //LCD_Update();
#endif
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


void SPI_Write(uint8_t data){
    HAL_SPI_Transmit(&hspi, &data, 1, 10);
}

uint8_t SPI_Read(void){
uint8_t data;
    HAL_SPI_Receive(&hspi, &data, 1, 10);
    return data;
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

#ifdef ENABLE_DISPLAY
/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void i2cInit(void){
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }
}
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};
uint32_t HAL_RCC_GetPCLK1Freq(void){
  return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* PB10     ------> I2C2_SCL
* PB11     ------> I2C2_SDA
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c){
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpioInit(GPIOB, 10, GPO_10MHZ | GPO_AF | GPO_OD);
    gpioInit(GPIOB, 11, GPO_10MHZ | GPO_AF | GPO_OD);
    __HAL_RCC_I2C2_CLK_ENABLE();
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

void I2C2_EV_IRQHandler(void){
    HAL_I2C_EV_IRQHandler(&hi2c2);
}

void I2C2_ER_IRQHandler(void){
    HAL_I2C_ER_IRQHandler(&hi2c2);
}

static volatile uint8_t lcd_busy;
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    lcd_busy = 0;
}

void I2C_WriteBlock(uint16_t address, uint8_t *data, uint16_t size){
#if 0
    uint32_t retry = 100;
    while(retry--){
        if(HAL_I2C_Master_Transmit_IT(&hi2c2, address << 1, data, size) == HAL_OK){
            break;
        }
    }
#else
    if(lcd_busy == 0){
        lcd_busy = 1;
        HAL_I2C_Master_Transmit_IT(&hi2c2, address << 1, data, size);
    }
#endif
}
/**
 * @brief request lcd update
 * @return : 0 if lcd is busy, otherwise success
 * */
uint8_t requestLcdUpdate(void){
    if(!lcd_busy){
        LCD_Update();
        return 1;
    }
    return 0;
}

#endif

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
    TIMER_BASE->CR1 = 0;                                // Stop counter
    TIMER_BASE->PSC = (SystemCoreClock/2000000) - 1;    // 36-1;for 72 MHZ /0.5sec/(35+1)
    TIMER_BASE->ARR = 0xFFFF;                           // Count until 0xFFFF
    TIMER_BASE->CCMR1 = (1<<4);                         // Main scheduler
    TIMER_BASE->SR = 0x1E5F & ~TIM_SR_CC1IF;            // Clear Timer/Comp2 interrupt flag
    TIMER_BASE->DIER = 0;                               // Disable Timer/Comp2 interrupts
    TIMER_BASE->EGR |= TIM_EGR_UG;                      // Refresh the timer's count, prescale, and overflow
    TIMER_BASE->CR1 |= TIM_CR1_CEN;                     // Enable counter
}

void delayMs(uint32_t ms)
{
    uint32_t timeout = ticks + ms;
    while(ticks < timeout){ }
}

uint32_t getTick(void){ return ticks; }
uint32_t HAL_GetTick(void){ return getTick(); }

/**
 * @brief Flash write functions for EEPROM emulation
 */
static void flashWrite(uint32_t address, const uint8_t *data, uint32_t count)
{
    uint16_t *psrc = (uint16_t*)data;

    HAL_StatusTypeDef res = HAL_FLASH_Unlock();

    if( res == HAL_OK){
        for (uint16_t i = 0; i < count; i+= 2, psrc++){
            res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, *psrc);
            if(res != HAL_OK){
                break;
            }
        }
    }

    HAL_FLASH_Lock();
}

/**
 * @brief Erase 1k selctor on flash
 *
 * @param address:  start address for erasing
 * @return : 0 on fail
 * */
void flashPageErase(uint32_t address)
{
    uint32_t res;
    extern void FLASH_PageErase(uint32_t PageAddress);

    res = HAL_FLASH_Unlock();

    if( res == HAL_OK){
        FLASH_PageErase(address);
    }
    FLASH->CR = 0;

    HAL_FLASH_Lock();
}

void flashRead (uint32_t addr, uint8_t *dst, uint32_t len)
{
    memcpy(dst, (void*)addr, len);
}

extern uint32_t _seeprom, _eeeprom;     //declared on linker script
static uint8_t eep_buf[32];

static nvdata_t laser4_plus_nvdata = {
    .sector = {
        .start = (uint32_t)&_seeprom,
        .end = (uint32_t)&_eeeprom,
        .init = NULL,
        .read = flashRead,
        .write = flashWrite,
        .erase = flashPageErase
    }
};

uint8_t* EEPROM_Init(uint16_t size)
{
    if(!laser4_plus_nvdata.nvb.size){
        // For this data has to be aligned to 16bit
        if(size & 1){
            size++;
        }

        if (size > 32){
            size = 32;
            DBG_BOARD_WRN("eep_buf should be bigger");
        }

        laser4_plus_nvdata.nvb.data = eep_buf;
        laser4_plus_nvdata.nvb.size = size;

        NV_Init(&laser4_plus_nvdata);
    }

    return laser4_plus_nvdata.nvb.data;
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
 * @brief Configure sample time for one adc channel
 *
 * */
static void adcSampleTime(uint16_t ch, uint16_t time){
    if(ch > 17){  // Max 17 channels
        return;
    }

    if(ch < 10){
        ADC1->SMPR2 =  (ADC1->SMPR2 & (7 << (3 * ch))) | (time << (3 * ch));   // Sample time for channels AN9-0
    }else{
        ADC1->SMPR1 =  (ADC1->SMPR1 & (7 << (3 * (ch % 10)))) | (time << (3 * (ch % 10)));   // Sample time for channels AN17-10
    }
}

/**
 * @brief calibrate ADC and get resolution based
 * on 1.20V internal reference
 * */
uint32_t adcCalibrate(void){
uint32_t bsqr3;

    ADC1->CR2 |= ADC_CR2_CAL;                     // Perform ADC calibration
    while(ADC1->CR2 & ADC_CR2_CAL){
        asm volatile("nop");
    }

    hadc.calibration_code = ADC1->DR;
    // Set calibration flag
    hadc.status |= ADC_CAL;
    // select VREFINT channel for first conversion
    bsqr3 = ADC1->SQR3;
    ADC1->SQR3 = (HW_VREFINT_CHANNEL << 0);
    // wake up Vrefint
    ADC1->CR2 |= ADC_CR2_TSVREFE;
    delayMs(5);
    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while(!(ADC1->SR & ADC_SR_EOC)){
        asm volatile("nop");
    }
    // Compute resolution
    hadc.resolution = VREFINT_VALUE / ADC1->DR;
    // power down VREFINT
    ADC1->CR2 &= ~ADC_CR2_TSVREFE;
    ADC1->SQR3 = bsqr3;
    // Set resolution flag
    hadc.status |= ADC_RES;
    return 1;
}

/**
 * @brief Set voltage divider racio, used to measure battery voltage
 *
 * @param r : Racio = R2/(R1+R2)
 * */
void adcSetVdivRacio(float r){
    hadc.vdiv_racio = r;
    hadc.status |= ADC_DIV;
}

/**
 * @brief Set current sense resistor value for current calculation
 *
 * @param rs : Sense resistor value in ohms
 * */
void adcSetSenseResistor(float rs){
    hadc.sense_resistor = rs * ISENSE_GAIN;
}

/**
 * @brief get current voltage divider racio, used to measure battery voltage
 *
 * @return : R2/(R1+R2)
 * */
float adcGetVdivRacio(void){
    return hadc.vdiv_racio;
}

/**
 * @brief Get current sense resistor value for current calculation
 *
 * @return : Sense resistor value in ohms
 * */
float adcGetSenseResistor(void){
    return hadc.sense_resistor / ISENSE_GAIN;
}


/**
 * @brief Configure ADC for a HW_VBAT_CHANNEL channel in interrupt mode and initiates a convertion.
 *  After convertion the result is stored locally through the interrupt
 *
 * PA0/AN0 is the default channel
 * */
static void adcInit(void){
    HW_VBAT_CH_INIT;

    RCC->APB2ENR  |= RCC_APB2ENR_ADC1EN;        // Enable and reset ADC1
    RCC->APB2RSTR |= RCC_APB2ENR_ADC1EN;
    RCC->APB2RSTR &= ~RCC_APB2ENR_ADC1EN;

    ADC1->CR2 = ADC_CR2_EXTSEL_SWSTART |        // Select software trigger,
                ADC_CR2_ADON;                   // Enable ADC
    delayMs(20);
    // Configure Sample time for the used channles
    adcSampleTime(HW_VBAT_CHANNEL, 6);          // Sample time, 6 => 71.5 cycles.
    adcSampleTime(HW_ISENSE_CHANNEL, 6);        // Sample time, 6 => 71.5 cycles.
    adcSampleTime(HW_VREFINT_CHANNEL, 3);       // Sample time 3 => 28.5 cycles.
    // Perform start up calibration
    adcCalibrate();
    // Configure channels to be converted and enable scan mode
    ADC1->SQR3 = (HW_ISENSE_CHANNEL << 5) | (HW_VBAT_CHANNEL << 0);
    ADC1->SQR1 = (1 << 20);                     // Two channels on sequence
    ADC1->CR1 |= ADC_CR1_SCAN;
     // Configure DMA
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;               // Enable DMA1
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;      // Source peripheral
    DMA1_Channel1->CCR =
            DMA_CCR_MSIZE_0 |                       // 16bit Dst size
            DMA_CCR_PSIZE_0 |                       // 16bit src size
            //DMA_CCR_DIR |                           // Read from memory
            DMA_CCR_MINC |                          // Memory increment
            DMA_CCR_TCIE;                           // Enable end of transfer interrupt
    ADC1->CR2 |= ADC_CR2_DMA;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    hadc.status = 0;
}

/**
 * @brief
 * */
static void adcStartConversion(void){
    hadc.status &= ~ADC_RDY;
    // Destination memory
    DMA1_Channel1->CMAR = (uint32_t)&hadc.result[0];
    // ADC sequence length
    DMA1_Channel1->CNDTR = ADC_SEQ_LEN;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    // Start
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/**
 * @brief Get current adc resolution (mV/step)
 * */
float adcGetResolution(void){
    return hadc.resolution;
}

/**
 * @brief Get battery voltage by start a convertion and wait for it to end
 *
 * @return : battery voltage in mV
 * */
uint32_t batteryGetVoltage(void){
    adcStartConversion();
    while((hadc.status & ADC_RDY) == 0 );
    return  hadc.battery_voltage;
}

/**
 * @brief Read battery voltage, if a battery measurement is ready, starts a new
 * conversion and return the last measured value. If a measurement is not available return
 *
 * @param dst : Pointer to place measured value
 *
 * @return : 0 if no measure is available (don't change dst), != 0 on success
 * */
uint32_t batteryReadVoltage(uint32_t *dst){
    if(hadc.status & ADC_RDY){
        *dst = hadc.battery_voltage;
        adcStartConversion();
        return 1;
    }
    return 0;
}

/**
 * @brief Get instant current consumption (mA)
 * */
uint32_t batteryGetCurrent(void){
    adcStartConversion();
    while((hadc.status & ADC_RDY) == 0 );
    return  hadc.battery_current;
}

/**
 * @brief Read current consumption, if measurement is ready, starts a new
 * conversion and return the last measured value. If a measurement is not available return
 *
 * @param dst : Pointer to place measured value
 *
 * @return : 0 if no measure is available (don't change dst), != 0 on success
 * */
uint32_t batteryReadCurrent(uint32_t *dst){
    if(hadc.status & ADC_RDY){
        *dst = hadc.battery_current;
        adcStartConversion();
        return 1;
    }
    return 0;
}

/**
 * @brief Read current consumption and vattery voltage, if measurement is ready, starts a new
 * conversion and return the last measured value. If a measurement is not available return
 *
 * @param dst : Pointer to place measured value
 *
 * @return : 0 if no measure is available (don't change dst), != 0 on success
 * */
uint32_t batteryReadVI(vires_t *dst){
    if(hadc.status & ADC_RDY){
        dst->vbat = hadc.battery_voltage;
        dst->cur = hadc.battery_current;
        adcStartConversion();
        return 1;
    }
    return 0;
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

    ENC_TIM->CR2 = 0;
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
#if defined(ENABLE_PPM)
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
    //DBG_PIN_HIGH;
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
    PPM_TIM->PSC = (SystemCoreClock/2000000) - 1;   // 36-1;for 72 MHZ /0.5sec/(35+1)
    PPM_TIM->CCMR1 = (7 << 12);                     // PWM mode 2
    PPM_TIM->CCER = TIM_CCER_CC2E;                  // Enable channel
    // Force high state
    PPM_TIM->ARR = PPM_MAX_PERIOD;
    PPM_TIM->CNT = PPM_MAX_PERIOD;
    PPM_TIM->CCR2 = PPM_PULSE_WIDTH;
    // Enable DMA Request
    PPM_TIM->DIER |= TIM_DIER_UDE;
}
#endif

#ifdef ENABLE_BUZZER
/**
 * @brief Basic tone generation on pin PA8 using TIM1_CH1
 * and DMA
 *
 * Buzzer timer is configured as PWM mode1 in downcount mode.
 * The counter starts from ARR register (top) that defines the frequency perioud in us,
 * and counts down, when matches CCR1 the output is set to high.
 * When the counter reaches zero, set the output to low and request a DMA transfer to ARR register.
 * On the last DMA transfer an interrupt is issued, that will configure the next tone periout to be
 * loaded to ARR or stop the melody.
 *
 * */
void buzInit(void){
    gpioInit(GPIOA, 8, GPO_AF | GPO_2MHZ);
    // Configure DMA
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;               // Enable DMA1
    DMA1_Channel5->CPAR = (uint32_t)&BUZ_TIM->ARR;  // Destination peripheral
    DMA1_Channel5->CCR =
            DMA_CCR_MSIZE_0 |                       // 16bit Dst size
            DMA_CCR_PSIZE_0 |                       // 16bit src size
            DMA_CCR_DIR |                           // Read from memory
            DMA_CCR_TCIE;                           // Enable end of transfer interrupt
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    // Configure timer
#ifdef BUZ_IDLE_HIGH
    BUZ_TIM->CR1 = 0;
    BUZ_TIM->CCMR1 = (7 << 4);                      // PWM mode 2
#else
    BUZ_TIM->CR1 = TIM_CR1_DIR;
    BUZ_TIM->CCMR1 = (6 << 4);                      // PWM mode 1
#endif
    BUZ_TIM->PSC = (SystemCoreClock/1000000) - 1;   // 72-1;for 72 MHZ (1us clock)
    BUZ_TIM->CCER = TIM_CCER_CC1E;                  // Enable channel
    BUZ_TIM->BDTR |= TIM_BDTR_MOE;                  // Necessary for TIM1
    // Force idle state
    BUZ_TIM->CCR1 = BUZ_DEFAULT_VOLUME;             // Low volume level
    BUZ_TIM->ARR = 0xFFF;
    BUZ_TIM->EGR |= TIM_EGR_UG;
    // Enable DMA Request
    BUZ_TIM->DIER |= TIM_DIER_UDE;
}

/**
 * @brief Private helper to initiate tone generation
 *
 * @param tone : pointer to first tone to be played
 * */
static void buzStartTone(tone_t *tone){
    DMA1_Channel5->CMAR = (uint32_t)(&tone->f);
    DMA1_Channel5->CNDTR = tone->d;
    DMA1_Channel5->CCR |= DMA_CCR_EN;
    BUZ_TIM->EGR = TIM_EGR_UG;
    BUZ_TIM->CR1 |=  TIM_CR1_CEN;
    hbuz.status |= BUZ_PLAYING;
}

/**
 * @brief Plays a single tone for a given time
 *
 * @param freq     : Tone fundamental frequency
 * @param duration : duration of tone in ms
 * */
void buzPlayTone(uint16_t freq, uint16_t duration){
uint32_t d = duration * 1000UL;    // Convert to us
    hbuz.tone.f = FREQ_TO_US(freq) - BUZ_TIM->CCR1; // Subtract volume pulse
    hbuz.tone.d = d / hbuz.tone.f;
    hbuz.ptone = &hbuz.tone;
    buzStartTone(hbuz.ptone);
    hbuz.ptone->d = 0;       //force tone ending
}

/**
 * @brief Plays a melody composed of multiple tones.
 * The last tone on melody must have duration of zero
 *
 * @param tones : pointer to tones array.
 * */
void buzPlay(tone_t *tones){
tone_t *pt = tones;

    // Convert each tone frequency to time in us
    while(pt->d > 0){
        uint32_t d = pt->d * 1000UL;
        pt->f = FREQ_TO_US(pt->f) - BUZ_TIM->CCR1;
        pt->d = d / pt->f;
        pt++;
    }

    // Set next tone
    hbuz.ptone = tones + 1;
    // Play first tone
    buzStartTone(tones);
}

/**
 * @brief Change tone volume by changing
 * duty cycle
 *
 * @param level : Tone volume 0 to tone frequency period
 *
 * @return : Current tone volume
 *
 * */
uint16_t buzSetLevel(uint16_t level){
    level -= 1;
    if(level < BUZ_TIM->ARR){
        BUZ_TIM->CCR1 = level;
    }

    return BUZ_TIM->CCR1 + 1;
}

/**
 * @brief Waits for the end of tone(s)
 * Blocking call duh..
 * */
void buzWaitEnd(void)
{
    uint32_t timeout = 0x8000;
    do{
        timeout--;
    }while(hbuz.status & BUZ_PLAYING && timeout);
}
#endif

/**
 * @brief Enable CRC unit
 * */
void crcInit(void){
    RCC->AHBENR |= RCC_AHBENR_CRCEN;
    CRC->CR = 1;
}

/**
 * @brief Start a software timer
 *
 * @param time : Timer duration
 * @param flags : Extra flags for continuous mode, 0 for single time
 * @param cb : callback function when timer expires
 *
 * @return : Assigned timer index
 * */
uint32_t startTimer(uint32_t time, uint32_t flags, void (*cb)(void)){

    for(uint32_t i = 0; i < SWTIM_NUM; i++){
        if((timers[i].status & SWTIM_RUNNING) == 0){
            timers[i].time = time;
            timers[i].count = 0;
            timers[i].action = cb;
            timers[i].status = flags | SWTIM_RUNNING;
            return i;
        }
    }
    return SWTIM_NUM;
}

/**
 * */
void stopTimer(uint32_t tim){
    timers[tim].status = 0;
}

/**
 * @brief Check if timers have expired and execute correspondent action
 *
 * */
void processTimers(void){
static uint32_t last_tick = 0;
uint32_t diff = ticks - last_tick;
swtimer_t *tim = timers;
    for(uint32_t i = 0; i < SWTIM_NUM; i++, tim++){
        if(tim->status & SWTIM_RUNNING){
            tim->count += diff;
            if(tim->count >= tim->time){
                tim->action();
                if(tim->status & SWTIM_AUTO_RELOAD){
                    tim->count = 0;
                }else{
                    tim->status &= ~(SWTIM_RUNNING);
                }
            }
        }
    }

    last_tick = ticks;
}

/**
 * @brief meh close enougth
 *
 * @return : CRC'd number with timer
 * */
uint32_t xrand(void){
    CRC->DR = TIMER_BASE->CNT;
    return CRC->DR;
}

/**
 * @brief Interrupts handlers
 * */
/**
  * @brief  USB Interrupt handler
  * @param  none
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler(void){
#ifdef ENABLE_VCP
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
#endif
}

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
// ADC1 DMA request
void DMA1_Channel1_IRQHandler(void){
    //if(DMA1->ISR & DMA_ISR_TCIF1){
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        hadc.battery_voltage = (float)(hadc.result[0] * hadc.resolution) / hadc.vdiv_racio;
        hadc.battery_current = (hadc.result[1] * hadc.resolution)/hadc.sense_resistor;
        hadc.status |= ADC_RDY;
    //}
    DMA1->IFCR |= DMA_IFCR_CGIF1;
}

// TIM1 DMA request
void DMA1_Channel5_IRQHandler(void){
    if(DMA1->ISR & DMA_ISR_TCIF5){
        DMA1_Channel5->CCR &= ~DMA_CCR_EN;
        if(hbuz.ptone->d != 0){
            // Load next tone
            DMA1_Channel5->CMAR = (uint32_t)(&hbuz.ptone->f);
            DMA1_Channel5->CNDTR = hbuz.ptone->d;
            DMA1_Channel5->CCR |= DMA_CCR_EN;
            hbuz.ptone++;
        }else{
            // Tone ended, stop tone generation
            BUZ_TIM->CR1 &= ~TIM_CR1_CEN;
            BUZ_TIM->ARR = 0xFFF;
            BUZ_TIM->EGR = TIM_EGR_UG;
            hbuz.status &= ~BUZ_PLAYING;
        }
    }
    DMA1->IFCR |= DMA_IFCR_CGIF5;
}
// TIM4 DMA request
void DMA1_Channel7_IRQHandler(void){
    if(DMA1->ISR & DMA_ISR_TCIF7){
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;
        // As two extra channels were send,
        // we end up here when transfering the first extra channel.
        // As the timer is stoped we get only a rising edge due to update event
        // and cancel the last channel transmission.
        PPM_TIM->CR1 &= ~TIM_CR1_CEN;
        //DBG_PIN_LOW;
    }
    DMA1->IFCR |= DMA_IFCR_CGIF7;  // Clear DMA Flags TODO: ADD DMA Error handling ?
}
