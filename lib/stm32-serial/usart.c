#include "board.h"
#include "app.h"

#ifdef ENABLE_USART

#define USART_QUEUE_ITEM_SIZE   1
#define USART_RX_DATA_SIZE      128
#define USART_TX_DATA_SIZE      512

static QueueHandle_t usart_tx_queue;
static QueueHandle_t usart_rx_queue;

static void usart_putchar(char c){
    //USART1->DR = c;
    //while((USART1->SR & USART_SR_TC) == 0);
    xQueueSend(usart_tx_queue, &c, 0);
    USART1->CR1 |= USART_CR1_TXEIE;		          // enable TX interrupt
}

static void usart_puts(const char* str){
    while(*str){
        //usart_putchar(*str++);
        xQueueSend(usart_tx_queue, str++, 0);
    }
    USART1->CR1 |= USART_CR1_TXEIE;
}  

static char usart_getchar(void){
    char c;
    xQueueReceive(usart_rx_queue, &c, portMAX_DELAY);
    return c;
}

static uint8_t usart_getCharNonBlocking(char *c){    
    return xQueueReceive(usart_rx_queue, c, 0) == pdPASS;
}

static uint8_t usart_kbhit(void){
    return USART_RX_DATA_SIZE - uxQueueSpacesAvailable(usart_rx_queue);
}

void usart_init(void){
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    asm("nop");
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    asm("nop");
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

    BOARD_GPIO_Init(GPIOA, 9, GPIO_CNF_AF | GPIO_MODE_O2MHZ);  // TX
    BOARD_GPIO_Init(GPIOA, 10, GPIO_CNF_AF);  // RX

    USART1->BRR = 0x271;        //115200
    USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    //USART1->CR1 |= USART_CR1_UE;
    //while((USART1->SR & USART_SR_TC) == 0);

    usart_rx_queue = xQueueCreate( USART_RX_DATA_SIZE, USART_QUEUE_ITEM_SIZE );
    configASSERT( (usart_rx_queue != NULL) );
    
    usart_tx_queue = xQueueCreate( USART_TX_DATA_SIZE, USART_QUEUE_ITEM_SIZE );
    configASSERT( (usart_tx_queue != NULL) );

    NVIC_SetPriority(USART1_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 2); 
    NVIC_EnableIRQ(USART1_IRQn);
}

stdout_t pcom = {
    .init = usart_init,
    .xgetchar = usart_getchar,
    .xputchar = usart_putchar,
    .xputs = usart_puts,
    .getCharNonBlocking = usart_getCharNonBlocking,
    .kbhit = usart_kbhit
};

void USART1_IRQHandler(void){
volatile uint32_t status = USART1->SR;

    // Data received
    if (status & USART_SR_RXNE) {
        USART1->SR &= ~USART_SR_RXNE;
        xQueueSendFromISR(usart_rx_queue, (void*)&USART1->DR, 0);
    }
    
    // Check if data transmiter if empty 
    if (status & USART_SR_TXE) {
        USART1->SR &= ~USART_SR_TXE;	          // clear interrupt
        // Check if data is available to send
        if(uxQueueMessagesWaitingFromISR(usart_tx_queue) > 0){
            uint32_t data;
            if(xQueueReceiveFromISR(usart_tx_queue, &data, 0) == pdPASS){
                USART1->DR = data;
            }
        }else{
               // No more data, disable interrupt
            USART1->CR1 &= ~USART_CR1_TXEIE;		      // disable TX interrupt if nothing to send
        }
    }    
}
#endif