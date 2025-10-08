#include <stdio.h>
#include "at32f4xx.h"
#include "board.h"
#include "stdinout.h"
#include "debug.h"
#include "uart.h"
#include "flash.h"
#include "dma.h"
#include "i2c.h"

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

#ifndef USE_STDPERIPH_DRIVER
#define RCC_AHBPERIPH_CRC           ((uint32_t)0x00000040)
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

static uint32_t systicks;
static void (*gpio_int_handler)(void);
static union{
    uint32_t all;
    struct {
        uint32_t wdt : 8;
        uint32_t spi : 1;
        uint32_t i2c : 1;
    };
}init_flags;

extern uint32_t _seeprom, _eeeprom;     //declared on linker script
static uint8_t eep_buf[L4P_EEPROM_SZ];

static nvdata_t dk415_nvdata = {
    .sector = {
        .start = (uint32_t)&_seeprom,
        .end = (uint32_t)&_eeeprom,
        .init = NULL,
        .read = FLASH_Read,
        .write = FLASH_Write,
        .erase = Flash_EraseSector
    },
    .nvb = {
        .size = 0,
        .next = NULL,
        .data = NULL
    }
};

extern void setup(void);
extern void loop(void);

#ifdef ENABLE_UART
static serialbus_t uartbus;
int serial_available(void) { return UART_Available(&uartbus); }
int serial_read(char *buf, int len) { return UART_Read(&uartbus, (uint8_t*)buf, len); }
int serial_write(const char *buf, int len) { return UART_Write(&uartbus, (const uint8_t*)buf, len); }
#endif

#ifdef CC2500_INSTALLED
static spibus_t hspi;
#endif

#ifdef ENABLE_PPM_OUTPUT
static dmatype_t ppmdma;
static uint16_t ppm_data[PPM_CH_OUT_NUM + 2];
static void ppmOutInit(void);
#endif

#ifdef ENABLE_AUX_ENCODER
static uint16_t enc_count;
static void auxEncoderInit(void);
#endif

#ifdef ENABLE_I2C
i2cbus_t i2cbus;
#endif

#ifdef ENABLE_BATTERY_MONITOR
#define ADC_RDY                 (1 << 0)
#define ADC_DIV                 (1 << 1)
#define ADC_CAL                 (1 << 2)
#define ADC_RES                 (1 << 3)
#define ADC_CR2_EXTSEL_SWSTART  (15 << 17)
#define ADC_SEQ_LEN             2

static adc_t hadc;
static dmatype_t adcdma;
static void adcInit(void);
#endif

#ifdef ENABLE_DISPLAY
static drvlcdi2c_t drvlcdi2c;
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
    RCC->APB1EN |= RCC_APB1EN_TMR3EN;

    TIME_BASE->CTRL1 = 0;                              // Stop counter
    TIME_BASE->DIV = (SystemCoreClock/2000000) - 1;    // 36-1;for 72 MHZ /0.5sec/(35+1)
    TIME_BASE->AR = 0xFFFF;                            // Count until 0xFFFF
    TIME_BASE->CCM1 = (1<<4);                          // Compare mode, CCR1 is used for precise timming
    TIME_BASE->STS = 0;                                // Clear all flags
    TIME_BASE->DIE = 0;                                // Disable Timer/Comp2 interrupts
    TIME_BASE->EVEG |= TMR_EVEG_UEVG;                  // Refresh the timer's count, prescale, and overflow
    TIME_BASE->CTRL1 = TMR_CTRL1_CNTEN;                // Enable counter */
}

static void dk415Init(void)
{
    RCC->APB2EN |= RCC_APB2EN_GPIOAEN | RCC_APB2EN_GPIOBEN |
                    RCC_APB2EN_GPIOCEN | RCC_APB2EN_AFIOEN;

    LED_INIT;
    HW_TX_35MHZ_EN_INIT;
#ifdef ENABLE_DEBUG
    DBG_PIN_INIT;
#endif

    SystemInit();
    systicksInit();

    /* CRC Init */
    RCC->AHBEN |= RCC_AHBPERIPH_CRC;
    CRC->CTRL = 1;

#ifdef ENABLE_UART
    uartbus.bus = UART_BUS0;
    uartbus.speed = 115200;
    UART_Init(&uartbus);
#endif

#ifdef ENABLE_PPM_OUTPUT
    ppmOutInit();
#endif

#ifdef ENABLE_AUX_ENCODER
    auxEncoderInit();
#endif

#ifdef ENABLE_BUZZER
    tone_pwm_init_t tone_init;
    tone_init.ch = BUZ_TIM_CH;
    tone_init.pin = BUZ_PIN;
    tone_init.pin_idle = BUZ_PIN_IDLE;
    tone_init.tmr = BUZ_TIM;
    tone_init.volume = BUZ_DEFAULT_VOLUME;

    TONE_PwmInit(&tone_init);
#endif

#ifdef CC2500_INSTALLED
    hspi.bus = SPI_BUS1;
    hspi.freq = 4000;
    hspi.cfg = SPI_MODE0;

    if (SPI_Init(&hspi) != SPI_OK){
        DBG_BOARD_ERR("SPI Init fail");
    }else{
        SPI_PINS_INIT;
    }

#endif

#ifdef ENABLE_AUX_ENCODER
    auxEncoderInit();
#endif

#ifdef ENABLE_BATTERY_MONITOR
    adcInit();
#endif

#ifdef ENABLE_I2C
    i2cbus.addr = 0;
    i2cbus.speed = 100;
    i2cbus.bus_num = I2C_BUS1;
    i2cbus.cfg = I2C_CFG_DMA;

    if (I2C_Init(&i2cbus) == I2C_SUCCESS){
        GPIO_Config(PB_10, GPO_MS_AF_OD);
        GPIO_Config(PB_11, GPO_MS_AF_OD);
    }else{
        init_flags.i2c = 1;
    }
#endif

    if(RCC->CTRLSTS){
        init_flags.wdt = RCC->CTRLSTS >> 24;
        RCC->CTRLSTS = RCC_CTRLSTS_RSTFC;
    }
}

int main(void)
{
    init_flags.all = 0;

    dk415Init();

    setup();

    /* Log init errors here, because dbg output is
       configured on setup
    */
    if(init_flags.i2c){
        DBG_BOARD_ERR("I2C Init fail");
    }

    if(init_flags.wdt){
        DBG_BOARD_WRN("Reset reason: 0x%x", init_flags.wdt);
    }

    while (1) {
        loop();
    }

    return 0;
}

uint32_t millis(void)
{
    return systicks;
}

void DelayMs(uint32_t ms)
{
    uint32_t timeout = systicks + ms;
    while(systicks < timeout){ }
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
    TIME_BASE->CC1 += interval;
    TIME_BASE->STS = 0;
}

/**
 * @brief Resets configured interval.
 * No flag is set/reset
 *
 */
void ticksResetInterval(void)
{
    TIME_BASE->CC1 = TIME_BASE->CNT;
}

/**
 * @brief Get ticks remaining until timeout.
 *
 * @return  Return ticks to timeout, Negative value represents
 * how many ticks have elapsed from timeout
 */
int16_t ticksGetIntervalRemaining(void)
{
    return TIME_BASE->CC1 - TIME_BASE->CNT;
}
/**
 * @brief checks timer flags for interval time out.
 * @param
 * @return 1: if interval has timed out/expired, 0 otherwise
 */
uint16_t ticksIsIntervalTimedout(void)
{
    return TIME_BASE->STS & TMR_STS_C1IF;
}

/**
 * @brief
 * @param name
 * @param edge
 * @param cb
 */
void gpioAttachInterrupt(uint32_t name, uint8_t edge, void(*cb)(void))
{
    (void)name;
    (void)edge;

    if(cb == NULL){
        return;
    }

    gpio_int_handler = cb;
    AFIO->EXTIC[1] = ( 1 << 4);        // PB5 -> EXTI5
    EXTI->INTEN  = ( 1 << 5);             // MR5
    EXTI->FTRSEL = (1 << 5);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
 * @brief
 * @param name
 */
void gpioRemoveInterrupt(uint32_t name)
{
    (void)name;

    NVIC_DisableIRQ(EXTI9_5_IRQn);
}

/**
 * @brief meh close enougth
 *
 * @return : CRC'd number with timer
 * */
uint32_t xrand(void)
{
    CRC->DT = systicks;
    return CRC->DT;
}

/**
 * @brief
 *
 * @param
 * @return
 */
uint32_t cpuGetId(void)
{
    uint32_t *cpuid = (uint32_t*)&MCUDBG->IDCR;
    return cpuid[0] ^ cpuid[1] ^ cpuid[2];
}

/**
 * @brief Initializes eeprom data structures
 *
 * @param size  number of bytes requested by
 *              application for eeprom
 * @return      Pointer to eeprom data bytes
 */
uint8_t* EEPROM_Init(uint16_t size)
{
    if(!dk415_nvdata.nvb.size){
        dk415_nvdata.nvb.data = eep_buf;
        dk415_nvdata.nvb.size = NVDATA_BLK_SZ(size);

        if (dk415_nvdata.nvb.size > L4P_EEPROM_SZ){
            dk415_nvdata.nvb.size = L4P_EEPROM_SZ;
            DBG_BOARD_WRN("eep_buf should be bigger");
        }

        NV_Init(&dk415_nvdata);
    }

    return dk415_nvdata.nvb.data;
}

#ifdef CC2500_INSTALLED
/**
 * @brief
 * @param data
 */
void SPI_Write(uint8_t data)
{
    SPI_Xchg(&hspi, &data);
}

/**
 * @brief
 * @param
 * @return
 */
uint8_t SPI_Read(void)
{
    uint8_t data = 0xFF;
    return SPI_Xchg(&hspi, &data);
}
#endif

#ifdef ENABLE_AUX_SWITCHES
/**
 * @brief Read digital switches values
 *
 * @return : bitmask with active switches
 * */
uint32_t auxGetSwitches(void)
{
    return (IS_HW_SW_AUX1_PRESSED << 0) | (IS_HW_SW_AUX1_PRESSED << 1) | (IS_HW_SW_AUX1_PRESSED << 2);
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
    RCC->APB1EN |= RCC_APB1EN_TMR2EN;

    GPIO_Config(PB_3, GPI_PU);
    GPIO_Config(PA_15, GPI_PU);

    AFIO->MAP = (AFIO->MAP & ~(3 << 8)) | (1 << 8);         // Partial remap for TIM2; PA15 -> CH1, PB3 -> CH2

    ENC_TIM->CTRL1 = 0;
    ENC_TIM->CTRL2 = 0;
    ENC_TIM->SMC = TMR_SMC_SMSEL_1;                         // Encoder mode 2, count only on TI1 edges
    ENC_TIM->CCM1 = (15 << 12) | (15 << 4) |                // Max length if input filter on TI1, TI2
                    (1 << 8) | (1 << 0);                    // Map TIxFP1 to TIx,
    ENC_TIM->CCE = 0;                                       // Falling polarity
    ENC_TIM->STS = 0;
    ENC_TIM->CTRL1 = TMR_CTRL1_CNTEN;
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

#ifdef ENABLE_PPM_OUTPUT
static void ppmEotHandler(void)
{
    // As two extra channels were send,
    // we end up here when transfering the first extra channel.
    // As the timer is stoped we get only a rising edge due to update event
    // and cancel the last channel transmission.
    DMA_Cancel(&ppmdma);
    PPM_TIM->CTRL1 &= ~TMR_CTRL1_CNTEN;
}

/**
 * @brief Generates a single PPM frame with 6 channels.
 * it should be called every 20ms with new data.
 *
 * @param data : pointer to servo data in us unit
 * @param nch : Number of channels in servo data,
 * has a maximum of PPM_CH_OUT_NUM
 *
 * */
void ppmOut(const uint16_t *data, uint8_t nch)
{
    if(nch > PPM_CH_OUT_NUM){
        nch = PPM_CH_OUT_NUM;
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
    PPM_TIM->AR = ppm_data[0];
    PPM_TIM->EVEG = TMR_EVEG_UEVG;
    // Configure DMA transfer, the first transfer will have the value on ppm_data[0]
    // since this value was transferred to produce initial pulse, it is necessary to send it again
    // to generate the channel time.
    ppmdma.len = nch + 2;
    DMA_Start(&ppmdma);
    // Resume timer
    PPM_TIM->CTRL1 |= TMR_CTRL1_CNTEN;
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
    RCC->APB1EN |= RCC_APB1EN_TMR4EN;

    GPIO_Config(HW_PPM_OUTPUT, GPO_LS_AF);

    // Configure DMA
    ppmdma.dir = DMA_DIR_M2P;
    ppmdma.ssize = DMA_DATA_SIZE_16;
    ppmdma.dsize = DMA_DATA_SIZE_16;
    ppmdma.src = (void*)&ppm_data[0];
    ppmdma.dst = (void*)&PPM_TIM->AR;
    ppmdma.eot = ppmEotHandler;
    ppmdma.len = PPM_CH_OUT_NUM + 2;
    DMA_Config(&ppmdma, PPM_DMA_REQ);

    PPM_TIM->CTRL1 = TMR_CTRL1_DIR | TMR_CTRL1_ARPEN;     // Down count and buffered ARR
    PPM_TIM->DIV = (SystemCoreClock/1000000) - 1;   // 1MHz clock, counts units of 1us
    PPM_TIM->CCM1 = (7 << 12);                      // PWM mode 2 on channel 2
    PPM_TIM->CCE = TMR_CCE_C2EN;                    // Enable channel
    // Force high state by having CNT > CCRx
    PPM_TIM->AR = PPM_MAX_PERIOD;
    PPM_TIM->CNT = PPM_MAX_PERIOD;
    PPM_TIM->CC2 = PPM_PULSE_WIDTH;
    // Enable DMA Request
    PPM_TIM->DIE |= TMR_DIE_UEVDE;
}
#endif


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
        ADC1->SMPT2 =  (ADC1->SMPT2 & (7 << (3 * ch))) | (time << (3 * ch));   // Sample time for channels AN9-0
    }else{
        ADC1->SMPT1 =  (ADC1->SMPT1 & (7 << (3 * (ch % 10)))) | (time << (3 * (ch % 10)));   // Sample time for channels AN17-10
    }
}

/**
 * @brief calibrate ADC and get resolution based
 * on 1.20V internal reference
 * */
uint32_t adcCalibrate(void)
{
    uint32_t bsqr3;

    ADC1->CTRL2 |= ADC_CTRL2_CAL;                     // Perform ADC calibration

    while(ADC1->CTRL2 & ADC_CTRL2_CAL){
        asm volatile("nop");
    }

    hadc.calibration_code = ADC1->RDOR;
    // Set calibration flag
    hadc.status |= ADC_CAL;
    // select VREFINT channel for first conversion
    bsqr3 = ADC1->RSQ3;
    ADC1->RSQ3 = (HW_VREFINT_CHANNEL << 0);
    // wake up Vrefint
    ADC1->CTRL2 |= ADC_CTRL2_TSREF;
    DelayMs(5);
    // Start conversion
    ADC1->CTRL2 |= ADC_CTRL2_SWSTR;
    while(!(ADC1->STS & ADC_STS_EC)){
        asm volatile("nop");
    }
    // Compute resolution
    hadc.resolution = VREFINT_VALUE / ADC1->RDOR;
    // power down VREFINT
    ADC1->CTRL2 &= ~ADC_CTRL2_TSREF;
    ADC1->RSQ3 = bsqr3;
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

    RCC->APB2EN  |= RCC_APB2EN_ADC1EN;          // Enable and reset ADC1
    RCC->APB2RST |= RCC_APB2EN_ADC1EN;
    RCC->APB2RST &= ~RCC_APB2EN_ADC1EN;

    ADC1->CTRL2 = ADC_CR2_EXTSEL_SWSTART |      // Select software trigger,
                  ADC_CTRL2_ADON;               // Enable ADC
    DelayMs(20);
    // Configure Sample time for the used channles
    adcSampleTime(HW_VBAT_CHANNEL, 6);          // Sample time, 6 => 71.5 cycles.
    adcSampleTime(HW_ISENSE_CHANNEL, 6);        // Sample time, 6 => 71.5 cycles.
    adcSampleTime(HW_VREFINT_CHANNEL, 3);       // Sample time 3 => 28.5 cycles.
    // Perform start up calibration
    adcCalibrate();
    // Configure channels to be converted and enable scan mode
    ADC1->RSQ3 = (HW_ISENSE_CHANNEL << 5) | (HW_VBAT_CHANNEL << 0);
    ADC1->RSQ1 = (1 << 20);                     // Two channels on sequence
    ADC1->CTRL1 |= ADC_CTRL1_SCN;
    // Configure DMA
    adcdma.dir = DMA_DIR_P2M;
    adcdma.ssize = DMA_DATA_SIZE_16;
    adcdma.dsize = DMA_DATA_SIZE_16;
    adcdma.src = (void*)&ADC1->RDOR;
    adcdma.dst = (void*)&hadc.result[0];
    adcdma.eot = adcEocHandler;
    adcdma.len = ADC_SEQ_LEN;

    DMA_Config(&adcdma, DMA1_REQ_ADC1);

    ADC1->CTRL2 |= ADC_CTRL2_DMAEN;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    hadc.status = 0;
}

/**
 * @brief
 * */
static void adcStartConversion(void){
    DMA_Start(&adcdma);
    hadc.status &= ~ADC_RDY;
    ADC1->CTRL2 |= ADC_CTRL2_SWSTR;
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

#ifdef ENABLE_DISPLAY
/**
 * @brief Initializes display
 *
 * @param
 * @return 1: on success
 */
uint32_t displayInit(void)
{
    drvlcdi2c.i2cdev = &i2cbus;
    drvlcdi2c.w = DISPLAY_W;
    drvlcdi2c.h = DISPLAY_H;

    if(!LCD_Init(&drvlcdi2c)){
        DBG_BOARD_ERR("Display Init fail");
        return 0;
    }
#ifdef BOARD_LASER4PLUS
    LCD_SetComPin(0);   // This display does not interleave rows
#endif
    LCD_SetOrientation(LCD_REVERSE_LANDSCAPE);

    return 1;
}
#endif
/**
 * Interrupt handlers
 */

void SysTick_Handler(void)
{
    systicks++;
}

