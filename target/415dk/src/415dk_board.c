#include <stdio.h>
#include "board.h"
#include "stdinout.h"
#include "uart.h"

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

static uint32_t ticks;

extern void setup(void);
extern void loop(void);
static void BOARD_Init(void);

int main(void)
{
    BOARD_Init();

    //setup();

    pcom.write("hello\n", 6);

    while (1) {
        //loop();
    }

    return 0;
}

void DelayMs(uint32_t ms)
{
    uint32_t timeout = ticks + ms;
    while(ticks < timeout){ }
}

/**
 * @brief
 * @param data
 */
void SPI_Write(uint8_t data)
{

}

/**
 * @brief
 * @param
 * @return
 */
uint8_t SPI_Read(void)
{
    return 0;
}


/**
 * @brief Read digital switches values
 *
 * @return : bitmask with active switches
 * */
uint32_t readSwitches(void){
    uint16_t state = 0;

        state = (IS_HW_SW_AUX1_PRESSED << 0) | (IS_HW_SW_AUX1_PRESSED << 1) | (IS_HW_SW_AUX1_PRESSED << 2);

        return state;
    }
static void BOARD_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2EN_GPIOAEN, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2EN_GPIOBEN, ENABLE);
    //RCC_APB2PeriphResetCmd(RCC_APB2EN_GPIOAEN, DISABLE);

    LED_INIT;

//    spiInit();
//    timInit();
//    adcInit();
//    encInit();
//    crcInit();

#ifdef ENABLE_PPM_OUTPUT
    ppmOutInit();
#endif

#ifdef ENABLE_BUZZER
    buzInit();
#endif

#ifdef ENABLE_UART
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

