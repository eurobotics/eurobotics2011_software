/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id$
 *
 *  Javier Bali�as Santos <javier@arc-robots.org>
 */

#include <stdio.h>
#include <string.h>

#include <configuration_bits_config.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <i2c_slave_lite.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>
#include <parse.h>
#include <rdline.h>

#include "../common/i2c_commands.h"
#include "ax12_user.h"
#include "cmdline.h"
#include "sensor.h"
#include "state.h"
#include "actuator.h"
#include "i2c_protocol.h"
#include "main.h"

struct genboard gen;
struct slavedspic slavedspic;

extern uint8_t i2c_watchdog_cnt;


void do_led_blink(__attribute__((unused)) void *dummy){
#if 1 /* simple blink */
	static uint8_t a=0;

	if(a){
		LED1_ON();
	}	
	else{
		LED1_OFF();
	}
	a = !a;
#endif
}

void do_i2c_watchdog(void *dummy){
	if(i2c_watchdog_cnt == 0){
		i2c_init(I2C_SLAVEDSPIC_ADDR);
		DEBUG(E_USER_I2C_PROTO,"I2C watchdog triggered, reinit i2c hw");
	}
	
	i2c_watchdog_cnt--;		
}

static void main_timer_interrupt(void){
	sei();
	scheduler_interrupt();
}

void timer_init(void)
{
	/* Scheduler uses timer1 */
	T1CON = 0;              
	IFS0bits.T1IF = 0;      
	IEC0bits.T1IE = 1;      
	TMR1 = 0x0000;  	
	PR1 = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	T1CONbits.TON = 1;  
}

/* Timer 1 interrupt handler */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
  IFS0bits.T1IF=0;
    
  /* scheduler interrupt */
  main_timer_interrupt();
}


void io_pins_init(void)
{
	/*************************************** 	*  IO portmap and config 	*/
	
	/* WARNING 1: after reset all pins are inputs */
	/* WARNING 2: after reset all ANALOG pins are analog
				  	  and has disabled the read operation
	*/
	
	/* leds */
	_TRISC9 = 0;	// SLAVE_LED1

	/* sensors */
	AD1PCFGL = 0xFF;	// all analog pins are digital
	_TRISB11 = 1;		// SENSOR1
	_TRISB10 = 1;		// SENSOR2
	_TRISB2 	= 1;		// SENSOR3
	_TRISA8 	= 1;		// SENSOR4
	_TRISC3 	= 1;		// SENSOR5
	_TRISB4 	= 1;		// SENSOR6
	_TRISC2 	= 1;		// SENSOR7
	
	/* dc motors */
	_TRISB12 = 0;	// SLAVE_MOT_2_INA
	_TRISB13 = 0;	// SLAVE_MOT_2_INB
	_LATB12  = 0;	// initialy breaked
	_LATB13  = 0;

	_TRISC6 = 0;	// SLAVE_MOT_1_INA
	_TRISC7 = 0;	// SLAVE_MOT_1_INB
	_LATC6  = 0;	// initialy braked
	_LATC7  = 0;

	/* other power outputs */
	_TRISC8 = 0;	// SLAVE_RELE_OUT
	_ODCC8  = 1;	// is open drain
	_LATC8  = 1;	// initialy Vload = 0

	_TRISA4 = 0;	// SLAVE_MOST_OUT
	_ODCA4  = 1; 	// is open drain
	_LATA4  = 1;	// initialy Vload = 0


	/* brushless motors */
	_TRISA10 = 0; 	// SLAVE_MOT_BRUSH_REV	_TRISA7  = 0; 	// SLAVE_MOT_BRUSH_BREAK
	_LATA7   = 0;	// initialy breaked
	
	/* servos  */
	_RP17R = 0b10010; // OC1 -> RP17(RC1) -> SLAVE_SERVO_PWM_1
	_RP16R = 0b10011; // OC2 -> RP16(RC0) -> SLAVE_SERVO_PWM_2
	_RP3R  = 0b10100; // OC3 -> RP3(RB3)  -> SLAVE_SERVO_PWM_3
		
	/* encoders */
	_QEA1R 	= 21;	// QEA1 <- RP21 <- SLAVE_ENC_CHA
	_TRISC5  = 1;	
	_QEB1R 	= 20;	// QEB1 <- RP20 <- SLAVE_ENC_CHB
	_TRISC4	= 1;
		
	/* i2c */
	_ODCB6 = 1;
	_ODCB5 = 1;
	
	/* uarts */
	_U1RXR 	= 8;	// U1RX <- RP8 <- SLAVE_UART_RX
	_TRISB8  = 1;	// U1RX is input
  	_RP7R 	= 3;	// U1TX -> RP7 -> SLAVE_UART_TX
	_TRISB7	= 0;	// U1TX is output
	
	_U2RXR 	= 9;	// U2RX <- RP9 <- SERVOS_AX12_UART  	_RP9R 	= 5;	// U2TX -> RP9 -> SERVOS_AX12_UART	_TRISB9	= 0;	// U2TX is output 	_ODCB9 	= 1;	// For half-duplex mode RP9 is open collector
}

int main(void)
{
	/* disable interrupts */
	cli();

	/* remapeable pins */
	io_pins_init();

	/* oscillator */
	oscillator_init();

	/* LEDS */
	LED1_OFF();

	/* clear structures */
	memset(&gen, 0, sizeof(gen));
	memset(&slavedspic, 0, sizeof(slavedspic));
	
	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* I2C */
	i2c_init(I2C_SLAVEDSPIC_ADDR);
	i2c_register_read_event(i2c_read_event);
	i2c_register_write_event(i2c_write_event);
	i2c_protocol_init();

	/* TIMER */
	timer_init();

	/* DO FLAGS */
	//slavedspic.flags |= DO_CS;

	/* SCHEDULER */
	scheduler_init();

	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						100000L / SCHEDULER_UNIT, 
						LED_PRIO);

	scheduler_add_periodical_event_priority(do_i2c_watchdog, NULL, 
						8000L / SCHEDULER_UNIT, 
						I2C_POLL_PRIO);

	scheduler_add_periodical_event_priority(do_sensors, NULL, 
						10000L / SCHEDULER_UNIT, 
						SENSOR_PRIO);

	/* TIME */
	time_init(TIME_PRIO);

	/* SERVOS AX12 */
	ax12_user_init();

	/* enable interrupt */
	sei();

	/* wait some ms */
	wait_ms(500);

	/* ACTUATORS */
	actuator_init();

	/* STATE MACHINE */
	//state_init();

	printf("\r\n");
	printf("Siempre falta tiempo para hacer pruebas. \r\n");

	/* LOGS */
 	gen.logs[0] = E_USER_ST_MACH;
	gen.log_level = 5;
	
	/* init cmdline */
	cmdline_init();

	/* main loop */
	while(1)
	{
		state_machines();
		cmdline_interact_nowait();
	}

	return 0;
}
