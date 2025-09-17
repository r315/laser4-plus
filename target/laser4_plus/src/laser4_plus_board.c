
#include "board.h"
#include "nvdata.h"
#include "stm32f1xx_hal.h"
#include "dma_stm32f1xx.h"
#include "tone_stm32f1xx.h"
#include "gpio_stm32f1xx.h"
#include "usbd_conf.h"
#include "debug.h"
#include "dma.h"
#include "tone.h"
#include "serial.h"

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

typedef struct adc{
    volatile uint16_t status;
    uint16_t result[2];
    uint32_t calibration_code;
    float resolution;
    float vdiv_racio;
    float sense_resistor;
    uint32_t battery_voltage;
    uint32_t battery_current;
}adc_t;

typedef struct swtimer{
        uint32_t time;
        uint32_t count;
        uint32_t status;
        void (*action)(void);
}swtimer_t;

// Private variables
static swtimer_t timers[SWTIM_NUM];
static volatile uint32_t systicks;
static void (*gpio_int_handler)(void);

// Private functions prototypes
static void systicksInit(void);
static void crcInit(void);

extern void setup(void);
extern void loop(void);

// Optional private varables and function prototypes
#ifdef CC2500_INSTALLED
static SPI_HandleTypeDef hspi;
#endif

#ifdef ENABLE_BATTERY_MONITOR
static adc_t hadc;
static dmatype_t adcdma;
static void adcInit(void);
#endif

#ifdef ENABLE_DISPLAY
static I2C_HandleTypeDef hi2c2;
static drvlcdi2c_t drvlcdi2c;
static volatile uint8_t lcd_busy;
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};
static void i2cInit(i2cbus_t *i2cbus);
#endif

#ifdef CC2500_INSTALLED
static void spiInit(void);
#endif

#ifdef ENABLE_AUX_ENCODER
static uint16_t enc_count;
static void auxEncoderInit(void);
#endif

#ifdef ENABLE_PPM_OUTPUT
static dmatype_t ppmdma;
static uint16_t ppm_data[MAX_PPM_CHANNELS + 2];
static void ppmOutInit(void);
#endif

#ifdef ENABLE_UART
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


// Functions implemenation

static void laser4Init(void)
{
    GPIO_ENABLE;
    AFIO->MAPR = (2 << 24); // SW-DP Enabled
    LED_INIT;
    HW_SW_INIT;
    HW_TX_35MHZ_EN_INIT;

    systicksInit();

#ifdef CC2500_INSTALLED
    CC25_CS_INIT;
    spiInit();
#endif

#ifdef ENABLE_BATTERY_MONITOR
    adcInit();
#endif

#ifdef ENABLE_AUX_ENCODER
    auxEncoderInit();
#endif

#ifdef ENABLE_PPM
    gpioInit(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN, GPI_PU);
#endif

#ifdef ENABLE_PPM_OUTPUT
    ppmOutInit();
#endif

#ifdef ENABLE_BUZZER
    tone_pwm_init_t tone_init;
    tone_init.ch = BUZ_TIM_CH;
    tone_init.pin = BUZ_PIN;
    tone_init.pin_idle = BUZ_PIN_IDLE;
    tone_init.tim = BUZ_TIM;
    tone_init.volume = BUZ_DEFAULT_VOLUME;

    TONE_PwmInit(&tone_init);
#endif
    crcInit();
#ifdef ENABLE_UART
    uartbus.bus = UART_BUS0;
    uartbus.speed = 115200;
    UART_Init(&uartbus);
#endif

#ifdef ENABLE_DISPLAY
    i2cInit(&drvlcdi2c.i2cdev);
    drvlcdi2c.w = DISPLAY_W;
    drvlcdi2c.h = DISPLAY_H;
    LCD_Init(&drvlcdi2c);
    LCD_SetComPin(0);   // This display does not interleave rows
    LCD_SetOrientation(LCD_REVERSE_LANDSCAPE);
#endif
}

int main(void)
{
    laser4Init();

    setup();

    while(1){
        loop();
    }

    return 0;
}

void gpioInit(GPIO_TypeDef *port, uint8_t pin, uint8_t mode) {

    mode &= 0x0f;

    if(mode == GPI_PD){
        port->BRR = (1 << pin);
    }

    if(mode == GPI_PU){
        port->BSRR = (1 << pin);
        mode = GPIO_IOF_PD; // Clear CNF bit 0
    }

    if(pin <  8){
        port->CRL = (port->CRL & ~(15 << (pin << 2))) | (mode << (pin << 2));
    }else{
        port->CRH = (port->CRH & ~(15 << ((pin - 8) << 2))) | (mode << ((pin - 8) << 2));
    }
}

void gpioAttachInterrupt(GPIO_TypeDef *port, uint8_t pin, uint8_t edge, void(*cb)(void))
{
    (void)pin;
    (void)port;
    (void)edge;

    if(cb == NULL){
        return;
    }

    gpio_int_handler = cb;
    AFIO->EXTICR[1] = ( 1 << 4);        // PB5 -> EXTI5
    EXTI->IMR  = ( 1 << 5);             // MR5
    EXTI->FTSR = (1 << 5);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void gpioRemoveInterrupt(GPIO_TypeDef *port, uint8_t pin)
{
    (void)pin;
    (void)port;

    NVIC_DisableIRQ(EXTI9_5_IRQn);
}

#ifdef CC2500_INSTALLED
void SPI_Write(uint8_t data)
{
    HAL_SPI_Transmit(&hspi, &data, 1, 10);
}

uint8_t SPI_Read(void)
{
    uint8_t data;
    HAL_SPI_Receive(&hspi, &data, 1, 10);
    return data;
}
/**
 *
 */
static void spiInit()
{
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
        DBG_BOARD_ERR("SPI Init fail");
    }

    SPI_PINS_INIT;
}
#endif

#ifdef ENABLE_DISPLAY
/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void i2cInit(i2cbus_t *i2cbus)
{
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
        DBG_BOARD_ERR("I2C Init fail");
    }

    i2cbus->addr = hi2c2.Init.OwnAddress1;
    i2cbus->peripheral = &hi2c2;
    i2cbus->speed = hi2c2.Init.ClockSpeed;
    i2cbus->bus_num = I2C_BUS1;
}

uint32_t HAL_RCC_GetPCLK1Freq(void)
{
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
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpioInit(GPIOB, 10, GPO_MS_AF_OD);
    gpioInit(GPIOB, 11, GPO_MS_AF_OD);
    __HAL_RCC_I2C2_CLK_ENABLE();
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c2);
}

void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c2);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    lcd_busy = 0;
}

void i2cWriteBlock(uint16_t address, uint8_t *data, uint16_t size)
{
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

uint16_t I2C_Write(i2cbus_t *i2c, uint8_t addr, const uint8_t *data, uint16_t size)
{
    HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)i2c->peripheral, addr << 1, (uint8_t*)data, size, 100);
    return size;
}

uint16_t I2C_Read(i2cbus_t *i2c, uint8_t addr, uint8_t *data, uint16_t size)
{
    // Not used
    (void)i2c;
    (void)addr;
    (void)data;

    return size;
}

/**
 * @brief request lcd update
 * @return : 0 if lcd is busy, otherwise success
 * */
uint8_t requestLcdUpdate(void)
{
    if(!lcd_busy){
        LCD_Update();
        return 1;
    }
    return 0;
}

#endif

/**
 * @brief Initialyze system time bases
 * */
static void systicksInit(void)
{
    /** 1ms time base using cortex-M systick
     * for software timers
     */
    systicks = 0;
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Configure 0.5us time base for multiprotocol */
    RCC->APB1ENR    |= RCC_APB1ENR_TIM3EN;

    TIME_BASE->CR1 = 0;                                // Stop counter
    TIME_BASE->PSC = (SystemCoreClock/2000000) - 1;    // 36-1;for 72 MHZ /0.5sec/(35+1)
    TIME_BASE->ARR = 0xFFFF;                           // Count until 0xFFFF
    TIME_BASE->CCMR1 = (1<<4);                         // Compare mode, CCR1 is used for precise timming
    TIME_BASE->SR = 0;                                 // Clear all flags
    TIME_BASE->DIER = 0;                               // Disable Timer/Comp2 interrupts
    TIME_BASE->EGR |= TIM_EGR_UG;                      // Refresh the timer's count, prescale, and overflow
    TIME_BASE->CR1 = TIM_CR1_CEN;                      // Enable counter
}

/**
 * @brief Get current timer tick.
 * @param
 * @return Current timer tick
 */
uint16_t ticksGet(void)
{
    return TIME_BASE->CNT;
}

/**
 * @brief Get how many ticks has elapsed from a given start
 * tick.
 * @param start Start tick
 * @return number of elapsed from given start
 */
uint16_t ticksGetElapsed(uint16_t start)
{
    return TIME_BASE->CNT - start;
}

/**
 * @brief Confgures timer to set a flag when
 * given interval has passed.
 * @param interval in timer ticks
 */
void ticksSetInterval(uint16_t interval)
{
    TIME_BASE->CCR1 += interval;
    TIME_BASE->SR = 0;
}

/**
 * @brief Resets configured interval.
 * No flag is set/reset
 *
 */
void ticksResetInterval(void)
{
    TIME_BASE->CCR1 = TIME_BASE->CNT;
}

/**
 * @brief Get ticks remaining until timeout.
 *
 * @return  Return ticks to timeout, Negative value represents
 * how many ticks have elapsed from timeout
 */
int16_t ticksGetIntervalRemaining(void)
{
    return TIME_BASE->CCR1 - TIME_BASE->CNT;
}
/**
 * @brief checks timer flags for interval time out.
 * @param
 * @return 1: if interval has timed out/expired, 0 otherwise
 */
uint16_t ticksIsIntervalTimedout(void)
{
    return TIME_BASE->SR & TIM_SR_CC1IF;
}

void DelayMs(uint32_t ms)
{
    uint32_t timeout = systicks + ms;
    while(systicks < timeout){ }
}

uint32_t HAL_GetTick(void)
{
    return systicks;
}

/**
 * @brief Unlock the FLASH control register access
 *
 * @return 1: success, 0: otherwise
 */
static uint8_t flashUnlock(void)
{
    if(FLASH->CR & FLASH_CR_LOCK){
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
    return !(FLASH->CR & FLASH_CR_LOCK);
}

/**
 * @brief Lock the FLASH control register access
 *
 */
static void flashLock(void)
{
    FLASH->CR = FLASH_CR_LOCK;
}

/**
 * @brief Wait for flash operation to end
 *
 * @return 1: operation done, 0: timeout
 */
static uint8_t flashWaitBusy(void)
{
    uint32_t tickstart;
    uint32_t Timeout = 50000;
    tickstart = systicks;

    while(FLASH->SR & FLASH_FLAG_BSY) {
        if((systicks - tickstart) > Timeout) {
            return 0;
        }
    }

    return 1;
}
/**
  * @brief  Program halfword (16-bit) at a specified address
  * @note   The function flashUnlock() should be called before to unlock the FLASH interface
  *         The function flashLock() should be called after to lock the FLASH interface
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @note   FLASH should be previously erased before new programmation (only exception to this
  *         is when 0x0000 is programmed)
  *
  * @param  Address:      Specifies the address to be programmed.
  * @param  Data:         Specifies the data to be programmed
  *
  * @return 1: success, 0: timeout
  */
 static uint8_t flashProgramHalfWord(uint32_t Address, uint16_t Data)
 {
    FLASH->CR |= FLASH_CR_PG;
    *(__IO uint16_t*)Address = Data;

    return flashWaitBusy();
 }
/**
 * @brief Flash write functions for EEPROM emulation
 */
static void flashWrite(uint32_t address, const uint8_t *data, uint32_t count)
{
    uint16_t *psrc = (uint16_t*)data;

    if(flashUnlock()){
        for (uint16_t i = 0; i < count; i+= 2, psrc++){
            if(!flashProgramHalfWord(address + i, *psrc)){
                break;
            }
        }
    }

    flashLock();
}

/**
 * @brief Erase 1k selctor on flash
 *
 * @param address:  start address for erasing
 * @return : 0 on fail
 * */
void flashPageErase(uint32_t address)
{
    if(flashUnlock()){
        /* Proceed to erase the page */
        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR = address;
        FLASH->CR |= FLASH_CR_STRT;
    }

    flashWaitBusy();

    FLASH->CR = 0;

    flashLock();
}

void flashRead (uint32_t addr, uint8_t *dst, uint32_t len)
{
    memcpy(dst, (void*)addr, len);
}

extern uint32_t _seeprom, _eeeprom;     //declared on linker script
static uint8_t eep_buf[L4P_EEPROM_SZ];

static nvdata_t laser4_plus_nvdata = {
    .sector = {
        .start = (uint32_t)&_seeprom,
        .end = (uint32_t)&_eeeprom,
        .init = NULL,
        .read = flashRead,
        .write = flashWrite,
        .erase = flashPageErase
    },
    .nvb = {
        .size = 0,
        .next = NULL,
        .data = NULL
    }
};

/**
 * @brief Initializes eeprom data structures
 *
 * @param size  number of bytes requested by
 *              application for eeprom
 * @return      Pointer to eeprom data bytes
 */
uint8_t* EEPROM_Init(uint16_t size)
{
    if(!laser4_plus_nvdata.nvb.size){
        laser4_plus_nvdata.nvb.data = eep_buf;
        laser4_plus_nvdata.nvb.size = NVDATA_BLK_SZ(size);

        if (laser4_plus_nvdata.nvb.size > L4P_EEPROM_SZ){
            laser4_plus_nvdata.nvb.size = L4P_EEPROM_SZ;
            DBG_BOARD_WRN("eep_buf should be bigger");
        }

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


#ifdef ENABLE_BATTERY_MONITOR
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
    DelayMs(5);
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

static void adcEocHandler(void)
{
    DMA_Cancel(&adcdma);
    hadc.battery_voltage = (float)(hadc.result[0] * hadc.resolution) / hadc.vdiv_racio;
    hadc.battery_current = (hadc.result[1] * hadc.resolution)/hadc.sense_resistor;
    hadc.status |= ADC_RDY;
}

/**
 * @brief Configure ADC for a HW_VBAT_CHANNEL channel in interrupt mode and initiates a convertion.
 *  After convertion the result is stored locally through the interrupt
 *
 * PA0/AN0 is the default channel
 * */
static void adcInit(void)
{
    HW_VBAT_CH_INIT;

    RCC->APB2ENR  |= RCC_APB2ENR_ADC1EN;        // Enable and reset ADC1
    RCC->APB2RSTR |= RCC_APB2ENR_ADC1EN;
    RCC->APB2RSTR &= ~RCC_APB2ENR_ADC1EN;

    ADC1->CR2 = ADC_CR2_EXTSEL_SWSTART |        // Select software trigger,
                ADC_CR2_ADON;                   // Enable ADC
    DelayMs(20);
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
    adcdma.dir = DMA_DIR_P2M;
    adcdma.ssize = DMA_CCR_PSIZE_16;
    adcdma.dsize = DMA_CCR_MSIZE_16;
    adcdma.src = (void*)&ADC1->DR;
    adcdma.dst = (void*)&hadc.result[0];
    adcdma.eot = adcEocHandler;
    adcdma.len = ADC_SEQ_LEN;

    DMA_Config(&adcdma, DMA1_REQ_ADC1);

    ADC1->CR2 |= ADC_CR2_DMA;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    hadc.status = 0;
}

/**
 * @brief
 * */
static void adcStartConversion(void){
    DMA_Start(&adcdma);
    hadc.status &= ~ADC_RDY;
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
uint32_t batteryGetVoltage(void)
{
    uint32_t timeout = 0x100;
    adcStartConversion();
    do{
        timeout--;
    }while((hadc.status & ADC_RDY) == 0 && timeout);
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
 * @brief Read current consumption and battery voltage, if measurement is ready, starts a new
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
#endif

#ifdef ENABLE_AUX_ENCODER
/**
 * @brief Rotary encorder init
 *  Configures a timer as pulse counter, the counter is incremented/decremented
 *  on edges from the two signals from the encoder
 *
 * Using TIM2 CH1 and CH2 as TI1 and TI2, also filter is configured
 * */
void auxEncoderInit(void)
{
    RCC->APB1ENR    |= RCC_APB1ENR_TIM2EN;

    gpioInit(GPIOB, 3, GPI_PU);
    gpioInit(GPIOA, 15, GPI_PU);
    AFIO->MAPR = (AFIO->MAPR & ~(3 << 8)) | (1 << 8);       // Partial remap for TIM2; PA15 -> CH1, PB3 -> CH2

    ENC_TIM->CR2 = 0;
    ENC_TIM->SMCR = TIM_SMCR_SMS_1;                         // Encoder mode 2, count only on TI1 edges
    ENC_TIM->CCMR1 = (15 << 12) | (15 << 4) |               // Max length if input filter on TI1, TI2
                  TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0;      // Map TIxFP1 to TIx,
    ENC_TIM->CCER = 0;                                      // Falling polarity
    ENC_TIM->SR = 0;
    ENC_TIM->CR1 = TIM_CR1_CEN;
}

uint16_t auxGetEncoderCount(void)
{
    return ENC_TIM->CNT;
}

/**
 * @brief Returns hom much counter has changes since last call
 *
 * @param
 * @return delta from last time called
 */
int16_t auxGetEncoder(void)
{
    int16_t diff = ENC_TIM->CNT - enc_count;
    enc_count += diff;
    return diff;
}
#endif

#ifdef ENABLE_AUX_SWITCHES
uint32_t auxGetSwitches(void)
{
    return (IS_HW_SW_AUX1_PRESSED << 0) |
            (IS_HW_SW_AUX2_PRESSED << 1) |
            (IS_HW_SW_AUX3_PRESSED << 2);
}
#endif

#ifdef ENABLE_PPM_OUTPUT
static void ppmEotHandler(void)
{
    // As two extra channels were send,
    // we end up here when transfering the first extra channel.
    // As the timer is stoped we get only a rising edge due to update event
    // and cancel the last channel transmission.
    DMA_Cancel(&ppmdma);
    PPM_TIM->CR1 &= ~TIM_CR1_CEN;
}

/**
 * @brief Generates a single PPM frame with 6 channels.
 * it should be called every 20ms with new data.
 *
 * @param data : pointer to servo data in us unit
 * @param nch : Number of channels in servo data,
 * has a maximum of MAX_PPM_CHANNELS
 *
 * */
void ppmOut(const uint16_t *data, uint8_t nch)
{
    if(nch > MAX_PPM_CHANNELS){
        nch = MAX_PPM_CHANNELS;
    }
    // Copy channel data to local buffer allows
    // application to change buffer right after this call
    for (uint16_t i = 0; i < nch; i++){
        ppm_data[i] = data[i];
    }
    // Set extra channels with max width at end ppm signal
    ppm_data[nch]     = PPM_MAX_PERIOD;
    ppm_data[nch + 1] = PPM_MAX_PERIOD;
    // Force counter update and DMA request, setting the period here
    // will produce a rising edge, but as the ppm line should be high
    // we get only the initial ppm low pulse.
    PPM_TIM->ARR = ppm_data[0];
    PPM_TIM->EGR = TIM_EGR_UG;
    // Configure DMA transfer, the first transfer will have the value on ppm_data[0]
    // since this value was transferred to produce initial pulse, it is necessary to send it again
    // to generate the channel time.
    ppmdma.len = nch + 2;
    DMA_Start(&ppmdma);
    // Resume timer
    PPM_TIM->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief PPM output generation.
 *
 * PPM signal can be divided in frames transmitted every 20ms.
 * Each frame can have up to 10 channels and a singe channel can vary it's period
 * betwen 1ms and 2ms but have a fixed pulse width of 0.3ms
 *
 * This implementation generates an inverted ppm siganl with 6 channels.
 *
 * ---+ +----------+ +-------+ +----+ +-------+ +-------+ +-------+ +---------
 *    |_|          |_|       |_|    |_|       |_|       |_|       |_|
 *    ^            ^         ^      ^         ^         ^         ^
 *    |    CH1     |   CH2   |  CH3 |   CH4   |   CH5   |   CH6   |
 *
 * A timer is configured to count downwards and a compare channel is configured in PMW mode 2.
 * In this configuration, the output is high while counter value is greater than
 * channel's value, and low otherwise.
 * When counter reaches zero, it triggers a dma transfer to update ARR, which is then loaded
 * back int to counter.
 * Since the new counter value is again greater than compare value, the output becomes high,
 * and cycle repeats.
 *
 * Two extra channels are used to generate the pulse for last channel.
 * EOT handler is called after last dma transfer and while timer counter is
 * greater than compare value. At this point dma is canceled and timer stopped,
 * leaving just the final pulse. With this cpu cycles aren't wasted waiting
 * for the end to generate the last pulse.
 *
 * */
void ppmOutInit(void)
{
    RCC->APB1ENR    |= RCC_APB1ENR_TIM4EN;
    RCC->APB1RSTR   |= RCC_APB1RSTR_TIM4RST;
    RCC->APB1RSTR   &= ~RCC_APB1RSTR_TIM4RST;

    gpioInit(GPIOB, 7, GPO_LS_AF);
    // Configure DMA
    ppmdma.dir = DMA_DIR_M2P;
    ppmdma.ssize = DMA_CCR_PSIZE_16;
    ppmdma.dsize = DMA_CCR_MSIZE_16;
    ppmdma.src = (void*)&ppm_data[0];
    ppmdma.dst = (void*)&PPM_TIM->ARR;
    ppmdma.eot = ppmEotHandler;
    ppmdma.len = MAX_PPM_CHANNELS + 2;
    DMA_Config(&ppmdma, PPM_DMA_REQ);

    PPM_TIM->CR1 =  TIM_CR1_DIR | TIM_CR1_ARPE;     // Down count and buffered ARR
    PPM_TIM->PSC = (SystemCoreClock/1000000) - 1;   // 1MHz clock, counts units of 1us
    PPM_TIM->CCMR1 = (7 << 12);                     // PWM mode 2
    PPM_TIM->CCER = TIM_CCER_CC2E;                  // Enable channel
    // Force high state by having CNT > CCRx
    PPM_TIM->ARR = PPM_MAX_PERIOD;
    PPM_TIM->CNT = PPM_MAX_PERIOD;
    PPM_TIM->CCR2 = PPM_PULSE_WIDTH;
    // Enable DMA Request
    PPM_TIM->DIER |= TIM_DIER_UDE;
}
#endif

#if defined(ENABLE_PWM)
/* Not sure if I need this code, probably pwm output was more useful */
void pwmOut(const uint16_t *data, uint8_t nch)
{

}

void pwmOutInit(void)
{
    RCC->APB1ENR |= (1 << 0);      // TIM2EN

    PWM_TIM->CR1 = 0;              // Stop counter

    PWM_TIM->PSC = SystemCoreClock/1000000;      // 1Mhz
    PWM_TIM->CCMR1 = 0x0101;       // CC1-2S = 01, ch1-2 mapped to TI1,TI2
    PWM_TIM->CCMR2 = 0x0101;       // CC3-4S = 01, map to TI1
    PWM_TIM->CCER  = 0x1111;       // CC1E, CC1; Configure all channels as input, Capture on rising edge

    PWM_TIM->DIER = (0x0f << 1);   // Enable interrupt on capture for all channels
    NVIC_EnableIRQ(TIM2_IRQn);     // Enable timer 2 interupt
    PWM_TIM->CR1 |= (1 << 0);      // Start counter
}

/**
 * Handles a radio channel with the correspondent captured value from interrupt.
 * Hw capture flag is cleared when the correspondent regiter is read.
 * This is a synchronous algorithm, since the ready flag is set only if all channels were
 * measured.
 * */
RAM_CODE void HandleChannel(volatile uint16_t *dst, volatile uint16_t *ccr, uint8_t ch){

    // check if overcapture occurred
    if(PWM_TIM->SR & (1 << (8 + ch))){
        // if set ignore capture and set capture for rising edge
        PWM_TIM->CCER &= ~(TIM_CAP_POL(ch));
        PWM_TIM->SR &= ~((1 << (8 + ch)) | (1 << ch));
        return;
    }

    // If interrupt was from a rising edge, save capture value on the remote channel
    // and change the polarity of the capturing edge for the give channel
    if(!(PWM_TIM->CCER & TIM_CAP_POL(ch))){
        // First capture, save it on buffer
        *dst =  *ccr;
        PWM_TIM->CCER |= TIM_CAP_POL(ch);
        if(ch == 1) ready = 0;
    }else{
        // If caused by falling edge, calculate the pulse width and save it on the
        // remote channel.
        *dst =  (*ccr > *dst) ? *ccr - *dst : (0xFFFF - *dst) + *ccr;
        PWM_TIM->CCER &= ~(TIM_CAP_POL(ch));
        // The calculated pulse value is stored on the lower half of the buffer
        *(dst-4) = *dst;
        if(ch == 4) ready = 1;
    }
}

/**
 * Every capture of any channel will generate an interrupt on capture event (rising or falling edge),
 * then the handler will evaluate which channel caused the interrupt and call the channel handler
 * with the correspondent radio channel buffer.
 * As each remote channel requires two captures we send the address of
 * the position to store the first captured value, then tha handler decides were to store
 * the pulse value
 * */
RAM_CODE void TIM2_IRQHandler(void){
    if(TIM2->SR & (1 << 1)){
        HandleChannel(&remote_channels[4], (uint16_t*)&TIM2->CCR1, 1);
    }

    if(TIM2->SR & (1 << 2)){
        HandleChannel(&remote_channels[5], (uint16_t*)&TIM2->CCR2, 2);
    }

    if(TIM2->SR & (1 << 3)){
        HandleChannel(&remote_channels[6], (uint16_t*)&TIM2->CCR3, 3);
    }

    if(TIM2->SR & (1 << 4)){
        HandleChannel(&remote_channels[7], (uint16_t*)&TIM2->CCR4, 4);
    }
}
#endif /* ENABLE_PWM */

#ifdef ENABLE_BUZZER
void buzPlayTone(uint16_t freq, uint16_t duration)
{
    TONE_Start(freq, duration);
}

void buzPlay(tone_t *tones)
{
    TONE_Play(tones);
}

uint16_t buzSetLevel(uint16_t level)
{
    return TONE_Volume(level);
}

void buzWaitEnd(void)
{
    while(TONE_Status() == TONE_PLAYNG){}
}
#endif

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
void processTimer(void)
{
    static uint32_t last_tick = 0;
    uint32_t diff = systicks - last_tick;
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

    last_tick = systicks;
}

/**
 * @brief Enable CRC unit
 * */
void crcInit(void){
    RCC->AHBENR |= RCC_AHBENR_CRCEN;
    CRC->CR = 1;
}

/**
 * @brief meh close enougth
 *
 * @return : CRC'd number with timer
 * */
uint32_t xrand(void)
{
    CRC->DR = TIME_BASE->CNT;
    return CRC->DR;
}

/**
 * @brief
 *
 * @param
 * @return
 */
uint32_t cpuGetId(void)
{
    uint32_t *cpuid = (uint32_t*)UID_BASE;
    return cpuid[0] ^ cpuid[1] ^ cpuid[2];
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
#if defined(ENABLE_VCP) || defined(ENABLE_GAME_CONTROLLER)
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
#endif
}

void EXTI9_5_IRQHandler(void){
uint32_t pr = EXTI->PR;
    if((pr & EXTI_PR_PR5) != 0){
        gpio_int_handler();
    }
    EXTI->PR = pr;
}

void SysTick_Handler(void){
    systicks++;
}
