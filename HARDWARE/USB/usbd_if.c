/**
  ******************************************************************************
  * @file    usbd_if.c
  * @author  fpistm
  * @brief   USB Device common interface file
  ******************************************************************************
  */
#ifdef USBCON

#include "usbd_if.h"
#include "usbd_cdc_if.h"

/**
  * @brief  Force to re-enumerate USB
  * @param  None
  * @retval None
  */

 void USBD_reenumerate(void)
{
	uint16_t pinDP =0;
#ifndef USBD_REENUM_DISABLED
  /* Re-enumerate the USB */
#ifdef USB_DISC_PIN
  pinMode(USB_DISC_PIN, OUTPUT);
  digitalWrite(USB_DISC_PIN, LOW);
#else
#ifdef USE_USB_HS_IN_FS
 pinDP = USB_OTG_HS_DP;
#elif defined(USB_OTG_FS)
 pinDP = USB_OTG_FS_DP;
#else /* USB */
 pinDP =  USB_OTG_FS_DP;
#endif
	pinMode(pinDP,OUTPUT);//  pin_function(pinDP, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
  digitalWrite(pinDP, LOW);
  delay(USBD_ENUM_DELAY);
//  pin_function(pinDP, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
	pinMode(pinDP,OUTPUT);
	
  /*delay(USBD_ENUM_DELAY);*/
#endif /* USB_DISC_PIN */
#endif /* USBD_REENUM_DISABLED */
}

#ifdef USBD_USE_CDC
void USBD_CDC_init(void)
{
  CDC_init();
}
#endif /* USBD_USE_CDC */
#endif /* USBCON */
