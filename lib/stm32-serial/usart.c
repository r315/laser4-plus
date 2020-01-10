#include "board.h"
#include "app.h"

#ifdef ENABLE_USART

static fifo_t usart_tx_fifo;
static fifo_t usart_rx_fifo;

static void usart_putchar(char c){
    //USART1->DR = c;
    //while((USART1->SR & USART_SR_TC) == 0);
    if(fifo_put(&usart_tx_fifo, c))
    USART1->CR1 |= USART_CR1_TXEIE;		          // enable TX interrupt
}

static void usart_puts(const char* str){
    while(*str){
        //usart_putchar(*str++);
        fifo_put(&usart_tx_fifo, *(uint8_t*)str++);
    }
    USART1->CR1 |= USART_CR1_TXEIE;
}  

static char usart_getchar(void){
    uint8_t c;
    fifo_get(&usart_rx_fifo, &c);
    return (char)c;
}

static uint8_t usart_getCharNonBlocking(char *c){    
    if(fifo_avail(&usart_rx_fifo))
        return fifo_get(&usart_rx_fifo, (uint8_t*)c);
    return 0;
}

static uint8_t usart_kbhit(void){
    return fifo_avail(&usart_rx_fifo);
}

void usart_init(void){
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    asm("nop");
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    asm("nop");
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

    gpioInit(GPIOA, 9, GPO_AF | GPO_2MHZ);  // TX
    gpioInit(GPIOA, 10, GPI_PU);            // RX

    USART1->BRR = 0x271;        //115200
    USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    //USART1->CR1 |= USART_CR1_UE;
    //while((USART1->SR & USART_SR_TC) == 0);

    fifo_init(&usart_rx_fifo);
    fifo_init(&usart_tx_fifo);

    NVIC_EnableIRQ(USART1_IRQn);
}

stdout_t pcom = {
    .init = usart_init,
    .xgetchar = usart_getchar,
    .xputchar = usart_putchar,
    .xputs = usart_puts,
    .getCharNonBlocking = usart_getCharNonBlocking,
    .kbhit = usart_kbhit,
    .user_ctx = NULL
};

void USART1_IRQHandler(void){
volatile uint32_t status = USART1->SR;

    // Data received
    if (status & USART_SR_RXNE) {
        USART1->SR &= ~USART_SR_RXNE;
        fifo_put(&usart_rx_fifo, (uint8_t)USART1->DR);
    }
    
    // Check if data transmiter if empty 
    if (status & USART_SR_TXE) {
        USART1->SR &= ~USART_SR_TXE;	          // clear interrupt
        // Check if data is available to send
        if(fifo_avail(&usart_tx_fifo) > 0){
            uint8_t data;
            fifo_get(&usart_tx_fifo, &data);
            USART1->DR = data;            
        }else{
               // No more data, disable interrupt
            USART1->CR1 &= ~USART_CR1_TXEIE;		      // disable TX interrupt if nothing to send
        }
    }    
}
#endif