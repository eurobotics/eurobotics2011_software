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
 *  Javier Baliñas Santos <javier@arc-robots.org>
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <time.h>

#include "../common/i2c_commands.h"

#include "main.h"
#include "beacon.h"

#define BEACON_UART		1
#define LINE_BUFF_SIZE 	64
#define CMD_LINE_SIZE 	16

/* local header functions */
void beacon_pull_opponent(void);

/* global variables */
int8_t beacon_connected=0;
int16_t link_id = 0;
int16_t error_id = 0;

char line_buff[LINE_BUFF_SIZE];
int8_t cmd_buff[CMD_LINE_SIZE];
uint16_t cmd_size = 0;
int8_t i=0;



/*******************************************************************
 * BEACON WT11 COMMANDS 
 ******************************************************************/

/* reset wt11 of robot (mainboard) */
void beacon_cmd_wt11_local_reset(void)
{
	/* set uart mux */
	set_uart_mux(BEACON_CHANNEL);

	/* change to cmd mode */
	wait_ms(1000);
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	wait_ms(1000);
	
	uart_send(BEACON_UART,'\n');		
	uart_send(BEACON_UART,'\r');		

	/* reset wt11 */
	uart_send(BEACON_UART,'\r');	
	uart_send(BEACON_UART,'\n');	
	uart_send(BEACON_UART,'R');	
	uart_send(BEACON_UART,'E');	
	uart_send(BEACON_UART,'S');	
	uart_send(BEACON_UART,'E');	
	uart_send(BEACON_UART,'T');	
	uart_send(BEACON_UART,'\n');	
}

/* call to beacon wt11 to open connection */
void beacon_cmd_wt11_call(void)
{
	const char send_buf[] = "CALL 00:07:80:85:04:70 1 RFCOMM\n";		
	int16_t i=0;

	/* set uart mux */
	set_uart_mux(BEACON_CHANNEL);

	/* send call cmd */
	for(i=0; i<32; i++){
		uart_send(BEACON_UART, send_buf[i]);
	}	
}

/* close connection with beacon wt11 */
void beacon_cmd_wt11_close(void)
{
	/* set uart mux */
	set_uart_mux(BEACON_CHANNEL);

	/* change to cmd mode */
	wait_ms(1200);
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	wait_ms(1200);
	
	uart_send(BEACON_UART,'\n');		
	uart_send(BEACON_UART,'\r');		

	/* close conection */
	uart_send(BEACON_UART,'C');	
	uart_send(BEACON_UART,'L');	
	uart_send(BEACON_UART,'O');	
	uart_send(BEACON_UART,'S');	
	uart_send(BEACON_UART,'E');	
	uart_send(BEACON_UART,' ');	
	uart_send(BEACON_UART,'0');		
	uart_send(BEACON_UART,'\n');		
}	


/************************************************************
 * SEND AND RECEVE MESSAGES 
 ***********************************************************/

/* send command to beacon */
void beacon_send_cmd(int8_t *buff, uint16_t size){
	int16_t i;

	/* set uart mux */
	set_uart_mux(BEACON_CHANNEL);
	
	/* check length */
	if(size > CMD_LINE_SIZE){
		ERROR(E_USER_BEACON, "Command size is too large");	
		return;
	}
		
	/* fill buffer */
	for(i=0; i<size; i++){
		cmd_buff[i] = buff[i];	
	}

	/* command size != 0 indicate 
    * that there is a command to send */		
	cmd_size = size;
}

/* send a command on queque or pull info of beacon */
void beacon_send_daemon(void * dummy)
{
	int16_t i;
	static uint8_t a=0;
	
	/* set uart mux */
	set_uart_mux(BEACON_CHANNEL);
	
	/* command on queque, send it */
	if(cmd_size){

		for(i=0; i<cmd_size; i++){
			uart_send(BEACON_UART, cmd_buff[i]);	
		}	
		cmd_size = 0;

		uart_send(BEACON_UART, '\n');
		uart_send(BEACON_UART, '\r');		
	}
	/* pull info of beacon */
	else{

		/* get opponent position */
		if(mainboard.flags & DO_OPP){
			
			/* led debug */
			a++;
			if (a & 0x4)
				LED3_TOGGLE();

			beacon_pull_opponent();	
		}
	}	
}


void parse_line(char * buff) 
{
	int16_t ret;
	uint8_t flags;
	int16_t arg0, arg1, arg2, arg3;
	int32_t arg4;
	int32_t checksum;

	/* set uart mux */
	set_uart_mux(BEACON_CHANNEL);


	DEBUG(E_USER_BEACON,"from beacon: %s",buff);
	
	/* BEACON ANSWERS */

	/* beacon wt11 open link connection pass */
 	ret = sscanf(buff, "CONNECT %d RFCOMM 1", &link_id);
	if(ret == 1){
		beacon_connected = 1;
		NOTICE(E_USER_STRAT, "beacon wt11 link open PASS (%d)", link_id);						
	}

	/* beacon wt11 open link connection fails */
 	ret = sscanf(buff, "NO CARRIER %d ERROR %d RFC_CONNECTION_FAILED", &link_id, &error_id);
	if(ret == 2){
		beacon_connected = 0;
		ERROR(E_USER_STRAT, "beacon wt11 link open FAIL(%d,%d)", error_id, link_id);						
	}
	
	/* beacon wt11 closed conection */
	
	/* beacon wt11 lossed conection */
	
	/* opponent */
 	ret = sscanf(buff, "opponent is %d %d %d %d %lx",
 							 &arg0, &arg1, &arg2, &arg3, &arg4);
	if(ret == 5){

		/* check checksum */
		checksum  = arg0;
		checksum += arg1;
		checksum += arg2;
		checksum += arg3;

		if(checksum == arg4) {
			IRQ_LOCK(flags);
			beaconboard.opponent_x = (int16_t)arg0;
			beaconboard.opponent_y = (int16_t)arg1;		
			beaconboard.opponent_a = (int16_t)arg2;
			beaconboard.opponent_d = (int16_t)arg3;
			IRQ_UNLOCK(flags);		
		}		
		else
			NOTICE(E_USER_BEACON, "checksum error: %d %d %d %d %lx",
 					arg0, arg1, arg2, arg3, arg4);		
	}
	
}


void line_char_in(char c)
{
	
	if(c == '\r' || c == '\n'){
		if(i!=0){			
			line_buff[i] = '\0';
			parse_line(line_buff);
			i=0;
		}
	}
	else{
		line_buff[i++] = c;
		i &= 0x3F;
	}		
}

void beacon_recv_daemon(void)
{
	char c;

	/* set uart mux */
	set_uart_mux(BEACON_CHANNEL);

	c=uart_recv_nowait(BEACON_UART);
		
		if(c != -1)
			line_char_in(c);		
}

void beacon_daemon(void * dummy)
{
	int16_t i;
	static uint8_t a = 0;
	volatile char c = 0;

//	if((mainboard.flags & DO_OPP) == 0)
//		return;

	/* set uart mux */
	set_uart_mux(BEACON_CHANNEL);
	
	/* receive aswers */
	while(c != -1){	
		c=uart_recv_nowait(BEACON_UART);
		if(c != -1)
			line_char_in(c);	
	}

	
	/* pulling and send commands */
	if(cmd_size){

		for(i=0; i<cmd_size; i++){
			uart_send(BEACON_UART, cmd_buff[i]);	
		}	
		cmd_size = 0;

		uart_send(BEACON_UART, '\n');
		uart_send(BEACON_UART, '\r');		
	}
	else{

		if(mainboard.flags & DO_OPP){
			a++;
			if (a & 0x4)
				LED3_TOGGLE();

			beacon_pull_opponent();	
		}
	}	
}


void beacon_init(void)
{
}

/* BEACON COMMANDS */

/* set color */
void beacon_cmd_color(void)
{
	int8_t buff[20];
	uint16_t size;
	
	if(mainboard.our_color == I2C_COLOR_RED)
		size = sprintf((char *)buff,"\n\rcolor red");
	else
		size = sprintf((char *)buff,"\n\rcolor blue");
	
	beacon_send_cmd(buff, size);
}


/* get opponent */
void beacon_cmd_opponent(void)
{
	int8_t buff[32];
	uint16_t size;
	int16_t robot_x, robot_y, robot_a;
	uint8_t flags;
	
	IRQ_LOCK(flags);
	robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
	robot_a = position_get_a_deg_s16(&mainboard.pos);
	IRQ_UNLOCK(flags);

	size = sprintf((char *)buff,"opponent %d %d %d",
								robot_x, robot_y, robot_a);

	beacon_send_cmd(buff, size);
}

/* pull opponent position */
void beacon_pull_opponent(void)
{
	int8_t buff[32];
	uint16_t size;
	int16_t robot_x, robot_y, robot_a;
	uint8_t flags;

	/* set uart mux */
	set_uart_mux(BEACON_CHANNEL);
	
	IRQ_LOCK(flags);
	robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
	robot_a = position_get_a_deg_s16(&mainboard.pos);
	IRQ_UNLOCK(flags);

	size = sprintf((char *)buff,"opponent %d %d %d",
								robot_x, robot_y, robot_a);

	for(i=0; i<size; i++){
		uart_send(BEACON_UART, buff[i]);	
	}	
	
	uart_send(BEACON_UART, '\n');
	uart_send(BEACON_UART, '\r');		

}


/* beacon on */
void beacon_cmd_beacon_on(void)
{
	int8_t buff[] = "\n\rbeacon on";
	uint16_t size = 11;
	
	beacon_send_cmd(buff, size);
}

/* beacon off*/
void beacon_cmd_beacon_off(void)
{
	int8_t buff[] = "\n\rbeacon off";
	uint16_t size = 12;
	
	beacon_send_cmd(buff, size);
}
