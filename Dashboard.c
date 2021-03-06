/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    DASHBOARD_TEMPLATE.C
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code has been modified from the example Blinky project of the
				RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <91x_lib.H>
#include "LCD.h"
#include <stdlib.h>

extern __irq void ADC_IRQ_Handler (void); /* ADC  interrupt routine           */
extern unsigned char AD_in_progress;      /* AD conversion in progress flag  */
extern short Potentiometer;
extern OS_MBX mailbox1;
extern OS_MBX mailbox2;
extern OS_MBX mailbox3;
extern OS_MBX mailbox4;

unsigned char A0,A1; //A0 - Button 3.5, A1 - Button 3.6
unsigned char B0 = 0,B1 = 0,B2 = 0,B3 = 0,B4 = 0,B5 = 0,B6 = 0,B7 = 0; //B0-B7 represent LED's 0 through 7

// global variable to access by all tasks
short d_cycle;														/* duty cycle to control brightness */
short potentiometer = 0; 											/* potentiometer value */
short slide_sensor = 0; 											/* slider value */
short force_sensor = 0;
short temperature = 0;

int counter;

// states to use in the program
enum B0_States {B0_OFF, B0_ON} B0_State;	/* states for PWM of B0 */
enum B1_States {B1_OFF, B1_ON} B1_State;	/* states for PWM of B1 */

// Internal Light Controller
int int_light_counter;
int int_light_dimmer;
int int_light_brightness;
int int_light_counter_max = 60;
int ambient_light_threshold = 512;
int int_light_door_reset = 0;
int int_light_dimming_counter;
// states used in the internal light controller
enum INT_LIGHT_States {INT_LIGHT_OFF, INT_LIGHT_ON, INT_LIGHT_DIMMING_OFF, INT_LIGHT_DIMMING_ON} INT_LIGHT_State;

// Alarm Controller
int alarm_counter;
enum ALARM_States {ALARM_ON, ALARM_OFF} ALARM_State;

// states used for engine
enum ENGINE_States {ENGINE_OFF, ENGINE_ON} ENGINE_State;
int engine_state = ENGINE_OFF;

// states used for door
enum DOOR_States {DOOR_OPEN, DOOR_CLOSE} DOOR_State;
int door_state = DOOR_CLOSE;

// states used for collision
enum AIRBAG_States {AIRBAG_ON, AIRBAG_OFF} AIRBAG_State;
int collision_display_counter;
int collision_threshold = 300;
int collision_reset = 0;

// states used for air conditioner control
enum AIRCON_States {ALL_OFF, HEATER_ON, COOLER_ON} AIRCON_state;
int temperature_range = 40;		// range of temperature: [0, 39]
int heater_lowerbound = 10;		
int heater_upperbound = 15;
int cooler_lowerbound = 20;
int cooler_upperbound = 25;

// Alarm Controller
enum SPEED_ALARM_States {SPEED_ALARM_ON, SPEED_ALARM_OFF} SPEED_ALARM_State;
int speed_alarm_counter;
int speed_alarm_threshold = 200;
int speed_alarm_counter;

// id of tasks involved in the program
OS_TID ADC_Conversion_ID;									/* ID for task 'ADC_Con' */
OS_TID DC_Computation_ID;									/* ID for task 'DC_Comp' */
OS_TID PWM_Generator_ID;									/* ID for task 'PWM_Gen' */
OS_TID INT_LIGHT_Controller_ID;						/* ID for task 'INT_LIGHT_Ctrl' */
OS_TID ALARM_Controller_ID;								/* ID for task 'ALARM_Ctrl' */
OS_TID COLLISION_Controller_ID;						/* ID for task 'COLLISION_Ctrl' */
OS_TID ENGINE_Controller_ID;							/* ID for task 'ENGINE_Ctrl' */
OS_TID DOOR_Controller_ID;								/* ID for task 'DOOR_Ctrl' */
OS_TID AIRCON_Controller_ID;							/* ID for task 'AIRCON_Ctrl' */
OS_TID INT_LIGHT_DIM_Controller_ID; 			/* ID for task 'INT_LIGHT_DIM_Ctrl' */
OS_TID SPEED_ALARM_Controller_ID; 				/* ID for task 'SPEED_ALARM_Ctrl' */

// Mutex
OS_MUT LCD_Mutex;

//Function to print an unsigned char value on the LCD Display
void print_uns_char (short value)
{	 
	int flag = 0,i;
	char c[4];
	do {
	   int rem = value%10;
	   value = value/10;
	   c[flag] = rem+48;
	   flag++;
	}while(value>0);
	for(i=flag-1;i>=0;i--) {
		LCD_putc(c[i]);
	}
}

//Function to read input
void read_buttons()
{
	//BUTTON_3_5:
	A0 = !(GPIO3->DR[0x080]>>5); // Returns 1 when pressed and 0 when released
	//BUTTON_3_6:
	A1 = !(GPIO3->DR[0x100]>>6); // Returns 1 when pressed and 0 when released
}

void read_mailboxes() 
{
	void *msg1, *msg2, *msg3, *msg4;
	// wait for mailboxes
	os_mbx_wait(&mailbox1, &msg1, 0xffff);
	potentiometer = * (short *)msg1;
	os_mbx_wait(&mailbox3, &msg3, 0xffff);
	force_sensor  = * (short *)msg3;
	if(engine_state == ENGINE_ON) {
		os_mbx_wait(&mailbox2, &msg2, 0xffff);
		slide_sensor  = * (short *)msg2;
		os_mbx_wait(&mailbox4, &msg4, 0xffff);
		temperature  = * (short *)msg4;
		// match reading of 2nd slide sensor to equivalent temperature
		temperature = temperature/12;
	} else {
		slide_sensor  = 0;
		temperature  = 0;
	}
	// free mailboxes
	free(msg1); free(msg2); free(msg3); free(msg4);
}

void start_ADC ( )
{
	if (!AD_in_progress)  
	{             						    /* If conversion not in progress      */
        AD_in_progress = 1;                 /* Flag that AD conversion is started */
        ADC->CR |= 0x0423;                  /* Set STR bit (Start Conversion)     */
    }
	//Now the interrupt will be called when conversion ends
}

//Function to write to led
void write_led()
{
  unsigned char mask = 0;

  mask  = (B0<<0);
  mask |= (B1<<1);
  mask |= (B2<<2);
  mask |= (B3<<3);
  mask |= (B4<<4);
  mask |= (B5<<5);
  mask |= (B6<<6);
  mask |= (B7<<7);

  GPIO7->DR[0x3FC] = mask;
}

void delay(int timeout) {
	U32 ticks;
	ticks = os_time_get ();
	while ((os_time_get () - ticks) < timeout){
	}
}

//Function to write to LCD
void write_LCD() {
	os_mut_wait(&LCD_Mutex, 0xffff);
	LCD_cls();
	LCD_puts("S:");
	print_uns_char(slide_sensor);
	LCD_gotoxy(1,2);
	LCD_puts("L:");
	print_uns_char(potentiometer);
	LCD_gotoxy(8,1);
	LCD_puts("E:");
	if(engine_state == ENGINE_ON) {
		LCD_puts("1");
	} else {
		LCD_puts("0");
	}
	LCD_gotoxy(14,1);
	LCD_puts("D:");
	if(door_state == DOOR_OPEN) {
		LCD_puts("1");
	} else {
		LCD_puts("0");
	}
	LCD_gotoxy(8, 2);
	LCD_puts("T:");
	print_uns_char(temperature);
	os_mut_release(&LCD_Mutex);
}

//Function to write collision to LCD
void write_speed_alarm_LCD() {
	os_mut_wait(&LCD_Mutex, 1000);
	LCD_cls();
	LCD_puts("Speed Limit");
	LCD_gotoxy(1,2);
	LCD_puts("Exceeded...");
	// delay for 1 sec and release
	delay(1000);
  os_mut_release(&LCD_Mutex);
}

//Function to write air conditioner status to LCD
void write_aircon_LCD(char * msg) {
	os_mut_wait(&LCD_Mutex, 1000);
	LCD_cls();
	LCD_puts("Temp: ");
	print_uns_char(temperature);
	LCD_gotoxy(1,2);
	LCD_puts(msg);
	// delay for 1 sec and release
	delay(1000);
	os_mut_release(&LCD_Mutex);
}

//Function to write collision to LCD
void write_collision_LCD() {
	os_mut_wait(&LCD_Mutex, 1000);
	LCD_cls();
	LCD_puts("Collision...");
	LCD_gotoxy(1,2);
	LCD_puts("Impact:");
	print_uns_char(force_sensor);
	write_led();
	// delay for 2 sec and release
	delay(2000);
	os_mut_release(&LCD_Mutex);
}

int PWM_StMch_B0(int state) 
{
	int high = 20 * d_cycle/100;
	int low = 20 - high;
	switch (state) {
		case (-1):
			B0 = 1;
			state = B0_ON;
			break;
		case B0_OFF:
			if (counter < low) {
				counter++;
			} else {
				counter = 0;
				if (high!=0) {
					B0 = 1;
					state = B0_ON;
				}
			}
			break;
		case B0_ON:
			if (counter <high) {
					counter++;
			} else {
				counter = 0;
				if (low!=0) {
					B0 = 0;
					state = B0_OFF;
				}
			}
			break;
		default:
			state = -1;
	}
	return state;
}

int PWM_StMch_B1(int state) 
{
	int high = 20 * int_light_brightness/30;
	int low = 20 - high;
	switch (state) {
		case (-1):
			B1 = 1;
			state = B1_ON;
			break;
		case B1_OFF:
			if (int_light_dimming_counter < low) {
				int_light_dimming_counter++;
			} else {
				B1 = 1;
				int_light_dimming_counter = 0;
				state = B1_ON;
			}
			break;
		case B1_ON:
			if (int_light_dimming_counter <high) {
					int_light_dimming_counter++;
			} else {
				B1 = 0;
				int_light_dimming_counter = 0;
				state = B1_OFF;
			}
			break;
		default:
			state = -1;
	}
	return state;
}

int SPEED_ALARM_StMch(int state) 
{
	switch (state) {
		case (-1):
			state = SPEED_ALARM_OFF;
			break;
		case SPEED_ALARM_OFF:
			if(engine_state == ENGINE_ON && slide_sensor >= speed_alarm_threshold) {
				state = SPEED_ALARM_ON;
				speed_alarm_counter = 0;
				B4 = 0;
				write_speed_alarm_LCD();
			} else {
				state = SPEED_ALARM_OFF;
				speed_alarm_counter = 0;
				B4 = 0;
			}
			break;
		case SPEED_ALARM_ON:
			if(engine_state == ENGINE_OFF || slide_sensor < speed_alarm_threshold - 10) {
				state = SPEED_ALARM_OFF;
				speed_alarm_counter = 0;
				B4 = 0;
			} else if(engine_state == ENGINE_ON){
					if(speed_alarm_counter == 0) {
						state = SPEED_ALARM_ON;
						B4 = 1;
						speed_alarm_counter = 1;
					} else if(speed_alarm_counter == 1) {
						state = SPEED_ALARM_ON;
						B4 = 0;
						speed_alarm_counter = 0;
					}
			}	
			break;
		default:
			state = -1;
	}
	return state;
}

/*-------------------------------------------------------------------------------
 *				Task 10 'SPEED_ALARM_Ctrl':	Controls speed alarm 	 
 --------------------------------------------------------------------------------*/
__task void SPEED_ALARM_Ctrl(void) {
	int state = -1;
	const unsigned int period = 100;
	os_itv_set(period);	
	while(1){
		state = SPEED_ALARM_StMch(state);
		write_led();
		os_itv_wait();
	}
}

/*-------------------------------------------------------------------------------
 *				Task 9 'INT_LIGHT_DIM_Ctrl':	Controls PWM generator for internal light 	 
 --------------------------------------------------------------------------------*/
__task void INT_LIGHT_DIM_Ctrl(void) {
	int state = -1;
	int int_light_dimming_counter = 5000;
	const unsigned int period = 1;
	os_itv_set(period);
	while (int_light_dimming_counter) {
		state = PWM_StMch_B1(state);
		int_light_dimming_counter--;
		os_itv_wait();
	}
}

int INT_LIGHT_StMch(int state) 
{
	switch (state) {
		case (-1):
			state = INT_LIGHT_OFF;
			break;
		case INT_LIGHT_OFF:
			if(engine_state == ENGINE_OFF && door_state == DOOR_OPEN && int_light_door_reset == 0) {
				if(potentiometer < ambient_light_threshold) {
					int_light_counter = 0;
					int_light_dimmer = 0;
					state = INT_LIGHT_ON;
					B1 = 1;
				} else {
					int_light_counter = 0;
					int_light_dimmer = 0;
					state = INT_LIGHT_OFF;
					B1 = 0;
				}
			} else {
				if(int_light_counter < int_light_counter_max) {
					int_light_counter++;
				} else {
					int_light_counter = 0;
				}
			}
			break;
		case INT_LIGHT_ON:
			// Engine Start
			if (engine_state == ENGINE_ON) {
					int_light_counter = 0;
					int_light_dimmer = 0;
					state = INT_LIGHT_OFF;
					B1 = 0;
			} else if(door_state == DOOR_CLOSE) {
				if(int_light_dimmer < 100) { //Wait for (100*100ms=)10 seconds
					int_light_dimmer++;
					int_light_counter = 0;
					state = INT_LIGHT_ON;
					B1 = 1;
				} else {
					int_light_counter = 0;
					int_light_dimmer = 0;
					int_light_brightness = 50;
					state = INT_LIGHT_DIMMING_OFF;
					B1 = 1;
				}
			}	else if(door_state == DOOR_OPEN) {
				if(int_light_counter < 200) { //Wait for (200*100ms=)20 seconds
					int_light_counter++;
					int_light_dimmer = 0;
					state = INT_LIGHT_ON;
					B1 = 1;
				} else {
					int_light_counter = 0;
					int_light_dimmer = 0;
					state = INT_LIGHT_OFF;
					B1 = 0;
					int_light_door_reset = 1;
				}
			} 
			break;
			case INT_LIGHT_DIMMING_OFF:
			if (engine_state == ENGINE_ON) {
					int_light_counter = 0;
					int_light_dimmer = 0;
					state = INT_LIGHT_OFF;
					B1 = 0;
			} else if(door_state == DOOR_CLOSE) {
					if(int_light_brightness > 1) {
						state = INT_LIGHT_DIMMING_OFF;
						B1 = 1;
						if(int_light_brightness == 50) {
							INT_LIGHT_DIM_Controller_ID = os_tsk_create(INT_LIGHT_DIM_Ctrl, 4);
						}
						int_light_brightness--;
					} else {
						os_tsk_delete (INT_LIGHT_DIM_Controller_ID);
						int_light_counter = 0;
						int_light_dimmer = 0;
						state = INT_LIGHT_OFF;
						B1 = 0;
					}
			} else if(door_state == DOOR_OPEN) {
					os_tsk_delete (INT_LIGHT_DIM_Controller_ID);
					int_light_counter = 0;
					int_light_dimmer = 0;
					state = INT_LIGHT_ON;
					B1 = 1;
			}
			break;
		default:
			state = -1;
	}
	return state;
}

int ALARM_StMch(int state) 
{
	switch (state) {
		case (-1):
			state = ALARM_OFF;
			break;
		case ALARM_OFF:
			if(engine_state == ENGINE_ON && door_state == DOOR_OPEN) {
				alarm_counter = 0;
				state = ALARM_ON;
			} else {
				state = ALARM_OFF;
				alarm_counter = 0;
				B2 = 0;
			}
			break;
		case ALARM_ON:
			if (engine_state == ENGINE_OFF || door_state == DOOR_CLOSE) {
					state = ALARM_OFF;
					alarm_counter = 0;
					B2 = 0;
			} else {
				if(alarm_counter == 0) {
					state = ALARM_ON;
					B2 = 1;
					alarm_counter = 1;
				} else if(alarm_counter == 1) {
					state = ALARM_ON;
					B2 = 0;
					alarm_counter = 0;
				}
			}	
			break;
		default:
			state = -1;
	}
	return state;
}

int AIRCON_StMch(int state) {
	switch (state) {
		case (-1):
			state = ALL_OFF;
			break;
		case ALL_OFF:
			if (engine_state==ENGINE_OFF) {
				state = ALL_OFF;
			} else {
				if (temperature<heater_lowerbound && engine_state==ENGINE_ON) {
					state = HEATER_ON;
					B5 = 1;
				  write_aircon_LCD("HEATER ON");
				} else if (temperature>cooler_upperbound && engine_state==ENGINE_ON) {
					state = COOLER_ON;
					B6 = 1;
					write_aircon_LCD("COOLER ON");
				} else {
					state = ALL_OFF;
				}
			}
			break;
		case HEATER_ON:
			if (engine_state==ENGINE_OFF) {
				state = ALL_OFF;
				B5 = 0;
			} else if (temperature>heater_upperbound) {
				state = ALL_OFF;
				B5 = 0;
				write_aircon_LCD("HEATER OFF");
			} else {
				state = HEATER_ON;
			}
			break;
		case COOLER_ON:
			if (engine_state==ENGINE_OFF) {
				state = ALL_OFF;
				B6 = 0;
			} else if (temperature<cooler_lowerbound) {
					state = ALL_OFF;
					B6 = 0;
					write_aircon_LCD("COOLER OFF");
			} else {
				state = COOLER_ON;
			}
			break;
		default:
			state = -1;
	}
	return state;
}

int COLLISION_StMch(int state) {
	switch (state) {
		case (-1):
			state = AIRBAG_OFF;
			break;
		case AIRBAG_OFF:
			if(collision_reset == 0 && force_sensor > collision_threshold) {
				state = AIRBAG_ON;
				collision_reset = 1;
				B3 = 1;
				write_collision_LCD();
			} else {
				state = AIRBAG_OFF;
				B3 = 0;
			}
			break;
		case AIRBAG_ON:
			if(collision_reset == 1) {
				state = AIRBAG_ON;
				collision_reset = 0;
				B3 = 1;
			} else {
				state = AIRBAG_OFF;
				collision_reset = 1;
				B3 = 0;
			}
			break;
		default:
			state = -1;
	}
	return state;
}

/*----------------------------------------------------------------------------
 *				Task 8 'COLLISION_Ctrl':	Controls collision of the Car 	 
 ----------------------------------------------------------------------------*/
__task void COLLISION_Ctrl(void) {
	int state = -1;
	const unsigned int period = 100;
	os_itv_set(period);	
	while(1){
		state = COLLISION_StMch(state);
		write_led();
		os_itv_wait();
	}
}

/*----------------------------------------------------------------------------
 * 				Task 'AIRCON_Ctrl: Controls air conditioner of the car
 ----------------------------------------------------------------------------*/
__task void AIRCON_Ctrl(void) {
	int state = -1;
	const unsigned int period = 100;
	os_itv_set(period);
	while (1) {
		state = AIRCON_StMch(state);
		write_led();
		os_itv_wait();
	}
}

/*----------------------------------------------------------------------------
 *				Task 7 'ALARM_Ctrl':	Controls alarm of the Car 	 
 ----------------------------------------------------------------------------*/
__task void ALARM_Ctrl(void) {
	int state = -1;
	const unsigned int period = 100;
	os_itv_set(period);	
	while(1){
		read_buttons();		
		state = ALARM_StMch(state);
		write_led();
		os_itv_wait();
	}
}


/*----------------------------------------------------------------------------
 *				Task 6 'INT_LIGHT_Ctrl':	Controls internal light of the Car 	 
 ----------------------------------------------------------------------------*/

__task void INT_LIGHT_Ctrl(void) {
	int state = -1;
	const unsigned int period = 100;
	os_itv_set(period);	
	while(1){
		read_buttons();		
		state = INT_LIGHT_StMch(state);
		write_led();
		os_itv_wait();
	}
}

/*----------------------------------------------------------------------------
 *				Task 5 'DOOR_Ctrl':	Controls Door	 
 ----------------------------------------------------------------------------*/

__task void DOOR_Ctrl(void) {
	const unsigned int period = 200;
	os_itv_set(period);	
	while(1){
		read_buttons();
		if(A0 && door_state == DOOR_OPEN) {
			door_state = DOOR_CLOSE;
			int_light_door_reset = 0;
		} else if(A0 && door_state == DOOR_CLOSE) {
			door_state = DOOR_OPEN;
		}
		write_led();
		os_itv_wait();
	}
}

/*----------------------------------------------------------------------------
 *				Task 4 'ENGINE_Ctrl':	Controls Engine	 
 ----------------------------------------------------------------------------*/

__task void ENGINE_Ctrl(void) {
	const unsigned int period = 200;
	os_itv_set(period);	
	while(1){
		read_buttons();
		if(A1 && engine_state == ENGINE_OFF) {
				engine_state = ENGINE_ON;
		} else if(A1 && engine_state == ENGINE_ON) {
				engine_state = ENGINE_OFF;
		}
		write_led();
		os_itv_wait();
	}
}

/*----------------------------------------------------------------------------
 *				Task 3 'PWM_Gen':	Generate PWM 	 
 ----------------------------------------------------------------------------*/

__task void PWM_Gen(void) {
	int state = -1;
	const unsigned int period = 1;
	os_itv_set(period);	
	while(1){ 
		state = PWM_StMch_B0(state);
		write_led();
		os_itv_wait();
	}
}

/*----------------------------------------------------------------------------
 *				Task 2 'DC_Comp' : Compute duty cycle to control brightness of LEDs
 ----------------------------------------------------------------------------*/
__task void DC_Comp(void){
	read_mailboxes();
	// compute the duty cycle and save value to D_Cycle
	d_cycle = potentiometer/200 * 20;
	write_LCD();	
	os_tsk_delete_self();
}

/*----------------------------------------------------------------------------
 *        Task 1 'ADC_Con': ADC Conversion
 *---------------------------------------------------------------------------*/
__task void ADC_Con(void){
	const unsigned int period = 100;
	os_itv_set(period);
	os_mut_init(&LCD_Mutex);
	for(;;){ 
		os_itv_wait();
		/* Do actions below */
		start_ADC();
		DC_Computation_ID = os_tsk_create(DC_Comp, 4);
	}
}	 // End ADC_Con(void)


/*----------------------------------------------------------------------------
 *        Task 0 'init': Initialize
 *---------------------------------------------------------------------------*/
__task void init (void) {

  unsigned int n = 0;
  		  
	/* Set up Potentiometer and Light sensor*/                                                              
  
  SCU->GPIOIN[4]  |= 0x07;                /* P4.0, P4.1 and P4.2 input  - mode 0             */
  SCU->GPIOOUT[4] &= 0xFFC0;              /* P4.0 output - mode 0             */

  /* To look up search for GPIO4_DIR in the peripherals manual 				  */
  GPIO4->DDR      &= 0xF8;                /* P4.0 and P4.1 direction - input  */
  SCU->GPIOANA    |= 0x0007;              /* P4.0 analog mode ON              */
	
	/* To understand the ADC configuration register consult the manual 				  */
	ADC->CR         |= 0x0042;              /* Set POR bit                      */
  for (n = 0; n < 100000; n ++);          /* Wait > 1 ms  (at 96 MHz)         */

  ADC->CR         &= 0xFFB7;              /* Clear STB bit                    */
  for (n = 0; n < 1500; n ++);            /* Wait > 15 us (at 96 MHz)         */

  ADC->CR         |= 0x0423;              /* Enable end of conversion interupt*/
  ADC->CCR         = 0x003F;              /* AD Conversion for Channels 0,1,2, No WDG on Ch 0    */

  /* Configure and enable IRQ for A/D Converter (ADC)                         */
  VIC0->VAiR[15]  = (unsigned int)ADC_IRQ_Handler; /* Setup ADC IRQ Hndl addr */
  VIC0->VCiR[15] |= 0x20;                 /* Enable the vector interrupt      */
  VIC0->VCiR[15] |= 15;                   /* Specify the interrupt number     */
  VIC0->INTER    |= (1<<15);              /* Enable ADC interrupt             */

  /* Configuring LED                     */
  SCU->GPIOOUT[7]  = 0x5555;
  GPIO7->DDR       = 0xFF;
  GPIO7->DR[0x3FC] = 0x00;

  /* LCD Setup                           */
  GPIO8->DDR       = 0xFF;
  GPIO9->DDR       = 0x07;

  /* Port 3 setup for button 3.5 and 3.6 */
  SCU->GPIOIN[3]  |= 0x60;
  SCU->GPIOOUT[3] &= 0xC3FF;
  GPIO3->DDR      &= 0x9F;

  LCD_init(); //Initialize LCD
  LCD_cur_off(); //Remove LCD cursor
  LCD_cls(); //Clearing LCD screen
 
  // create instance of tasks
  //os_mut_init(&LCD_Mutex);	
	ADC_Conversion_ID = os_tsk_create(ADC_Con, 1);
	COLLISION_Controller_ID = os_tsk_create(COLLISION_Ctrl, 1);
	ALARM_Controller_ID = os_tsk_create(ALARM_Ctrl, 2);
	SPEED_ALARM_Controller_ID = os_tsk_create(SPEED_ALARM_Ctrl, 2);
	ENGINE_Controller_ID = os_tsk_create(ENGINE_Ctrl, 2);
	DOOR_Controller_ID = os_tsk_create(DOOR_Ctrl, 2);	
	AIRCON_Controller_ID = os_tsk_create(AIRCON_Ctrl, 2);
	INT_LIGHT_Controller_ID = os_tsk_create(INT_LIGHT_Ctrl, 3);
	PWM_Generator_ID = os_tsk_create(PWM_Gen, 4);
  os_tsk_delete_self ();
}


/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {

  os_sys_init (init);                    /* Initialize RTX and start init    */
  return 0;
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
