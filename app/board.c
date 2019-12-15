
#include "board.h"

static SPI_HandleTypeDef hspi;

static void spiInit(void);

void Error_Handler(char * file, int line){
  while(1){
  }
}

void BOARD_Init(void){
    spiInit();
}

void SPI_Write(uint8_t data){
  HAL_SPI_Transmit(&hspi, &data, 1, 10);
} 

uint8_t SPI_Read(void){
    return 0;
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
    hspi.Instance = SPI1;
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
}