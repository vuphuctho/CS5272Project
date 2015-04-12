/******************************************************************************/
/* IRQ.C: IRQ Handler                                                         */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <91x_lib.h>
#include <RTL.h>


short Potentiometer,SlideSensor;
int IRSensor;


unsigned char AD_in_progress;           /* AD conversion in progress flag     */

os_mbx_declare(mailbox1, 20);						/* Mailbox containing Potentiometer value	*/
os_mbx_declare(mailbox2, 20);						/* Mailbox containing SlideSensor value	*/
os_mbx_declare(mailbox3, 20);	

__irq void ADC_IRQ_Handler (void) {     /* AD converter interrupt routine     */
	void *msg1;
	void *msg2;
	void *msg3;
	Potentiometer = ADC->DR0 & 0x03FF;    /* AD value for global usage (10 bit) */	
	os_mbx_init(&mailbox1, sizeof(mailbox1));
	msg1 = &Potentiometer;	
	os_mbx_send(&mailbox1, msg1, 0xffff);
	SlideSensor = ADC->DR1 & 0x03FF;          /* AD value for global usage (10 bit) */	//dr4.1
	os_mbx_init(&mailbox2, sizeof(mailbox2));
	msg2 = &SlideSensor;
	os_mbx_send(&mailbox2, msg2, 0xffff);
	IRSensor = ADC->DR2 & 0x03FF;    /* AD value for global usage (10 bit) */	
	os_mbx_init(&mailbox3, sizeof(mailbox3));
	msg3 = &IRSensor;	
	os_mbx_send(&mailbox3, msg3, 0xffff);
	ADC->CR &= 0xFFFE;                    /* Clear STR bit (Start Conversion)   */
  ADC->CR &= 0x7FFF;                    /* Clear End of Conversion flag       */

  VIC0->VAR = 0;                        /* Acknowledge Interrupt              */  
  VIC1->VAR = 0;

  AD_in_progress = 0;                   /* Clear flag, as AD conv finished    */
}
