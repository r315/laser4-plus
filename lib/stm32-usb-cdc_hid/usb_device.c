/**
  ******************************************************************************
  * @file           : USB_DEVICE
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_hid.h"
#include "usbd_composite.h"

/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;

typedef struct callback{
    void *param;
    void (*cb)(void*);
}callback_t;

typedef struct deviceCallback{
    callback_t suspend;
    callback_t resume;
}deviceCallback_t;

static deviceCallback_t device_callbacks;

static void DEVICE_SuspendCallback(PCD_HandleTypeDef *hpcd);
static void DEVICE_ResumeCallback(PCD_HandleTypeDef *hpcd);
/**
* Vile hack to reenumerate, physically _drag_ d+ low.
* (need at least 2.5us to trigger usb disconnect)
* @retval None
* */
static void USB_DEVICE_Reenumerate(void)
{
    USB->CNTR = USB_CNTR_PDWN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRH = (GPIOA->CRH & ~(0x0F << 16)) | (2 << 16);
    GPIOA->BRR = (1 << 12); //GPIO_PIN_12;
    asm volatile(
        "mov r0,#0x30000 \n"
        "loop: \n"
        "nop \n"
        "subs r0,#1 \n"
        "bne loop \n");
    GPIOA->CRH = (GPIOA->CRH & ~(0x0F << 16)) | (4 << 16);
}

/* init function */
void USB_DEVICE_Init(void)
{
    USBD_Composite_Set_Descriptor(COMPOSITE_CDC_HID_DESCRIPTOR, COMPOSITE_CDC_HID_DESCRIPTOR_SIZE);
    USBD_Composite_Set_Classes(USBD_CDC_CLASS, USBD_HID_CLASS);
    in_endpoint_to_class[HID_EPIN_ADDR & 0x7F] = 1;

    USB_DEVICE_Reenumerate();
    USBD_Init(&hUsbDeviceFS, &FS_Desc_Composite, DEVICE_FS);
    USBD_RegisterClass(&hUsbDeviceFS, USBD_COMPOSIT_CLASS);
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
    USBD_Start(&hUsbDeviceFS);
}

uint8_t USB_DEVICE_SendReport(uint8_t *report, uint16_t len)
{
    return USBD_HID_SendReport(&hUsbDeviceFS, report, len);
}

void USB_DEVICE_RegisterCallback(HAL_PCD_CallbackIDTypeDef id, void(*cb)(void*), void *ptr){
    switch(id){
        case HAL_PCD_SUSPEND_CB_ID:
            device_callbacks.suspend.cb = cb;
            device_callbacks.suspend.param = ptr;
            HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_SUSPEND_CB_ID, DEVICE_SuspendCallback);
            break;

        case HAL_PCD_RESUME_CB_ID:
            device_callbacks.resume.cb = cb;
            device_callbacks.resume.param = ptr;
            HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_RESUME_CB_ID, DEVICE_ResumeCallback);
            break;
        default :
            break;
    }
}

void USB_DEVICE_RegisterSuspendCallback(void(*cb)(void*), void *ptr)
{
    USB_DEVICE_RegisterCallback(HAL_PCD_SUSPEND_CB_ID, cb, ptr);
}

void USB_DEVICE_RegisterResumeCallback(void(*cb)(void*), void *ptr)
{
    USB_DEVICE_RegisterCallback(HAL_PCD_RESUME_CB_ID, cb, ptr);
}

int USB_DEVICE_VcpWrite(const char *data, int len)
{
    return vcp_write(data, len);
}

int USB_DEVICE_VcpRead(char* data, int len)
{
    return vcp_read(data, len);
}

int USB_DEVICE_VcpAvailable(void)
{
    return vcp_available();
}

void USB_DEVICE_Handler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

static void DEVICE_SuspendCallback(PCD_HandleTypeDef *hpcd){
    USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
    if (hpcd->Init.low_power_enable){
        SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }
    device_callbacks.suspend.cb(device_callbacks.suspend.param);
}

static void DEVICE_ResumeCallback(PCD_HandleTypeDef *hpcd){
    USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
    device_callbacks.resume.cb(device_callbacks.resume.param);
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
