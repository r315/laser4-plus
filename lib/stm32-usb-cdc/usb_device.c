#include "usbd_desc.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

USBD_HandleTypeDef hUsbDeviceFS;

/**
* Vile hack to reenumerate, physically _drag_ d+ low.
* (need at least 2.5us to trigger usb disconnect)
* @retval None
* */
static void USB_DEVICE_Reenumerate(void){
    USB->CNTR = USB_CNTR_PDWN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRH = (GPIOA->CRH & ~(0x0F << 16)) | (2 << 16);
    GPIOA->BRR = (1 << 12); //GPIO_PIN_12;
    asm volatile( 
                  "mov r0,#2048 \n"
                  "loop:"
                  "nop\n"
                  "subs r0,#1 \n"
                  "bne loop"
    );
    GPIOA->CRH = (GPIOA->CRH & ~(0x0F << 16)) | (4 << 16);
}

/**
 * @brief CDC_Init
 *        Initialyse  Comunications Device Class interface
 *  
 */ 
void USB_DEVICE_Init(void){
USBD_HandleTypeDef *dev = &hUsbDeviceFS;
    
    //vcom_queue = xQueueCreate( APP_RX_DATA_SIZE, VC_QUEUE_ITEM_SIZE );
    //configASSERT( (vcom_queue != NULL) );

    USB_DEVICE_Reenumerate();

    USBD_Init(dev, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(dev, USBD_CDC_CLASS);
    USBD_CDC_RegisterInterface(dev, &USBD_Interface_fops_FS);
    USBD_Start(dev);
}

void USB_DEVICE_RegisterCallback(HAL_PCD_CallbackIDTypeDef id, void(*cb)(void*), void *ptr){

}

uint8_t USB_DEVICE_SendReport(uint8_t *report, uint16_t len){
    
}