/*  
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: state.c,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#include <math.h>
#include <string.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <encoders_dspic.h>
#include <pwm_servo.h>
#include <pwm_mc.h>
#include <dac_mc.h>

#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <vt100.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "ax12_user.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "arm_xy.h"
#include "arm_highlevel.h"
#include "state.h"

#if 0

#define STMCH_DEBUG(args...) DEBUG(E_USER_ST_MACH, args)
#define STMCH_NOTICE(args...) NOTICE(E_USER_ST_MACH, args)
#define STMCH_ERROR(args...) ERROR(E_USER_ST_MACH, args)

/* shorter aliases for this file */
#define HIDE_ARM								I2C_SLAVEDSPIC_MODE_HIDE_ARM
#define SHOW_ARM								I2C_SLAVEDSPIC_MODE_SHOW_ARM
#define PREPARE_HARVEST_BALL		I2C_SLAVEDSPIC_MODE_PREPARE_HARVEST_BALL
#define HARVEST_TOMATO					I2C_SLAVEDSPIC_MODE_HARVEST_TOMATO

#define PUTIN_FINGER_BALL 			I2C_SLAVEDSPIC_MODE_PUTIN_FINGER_BALL
#define ARM_PUMP_ON 						I2C_SLAVEDSPIC_MODE_ARM_PUMP_ON						
#define ARM_PUMP_OFF 						I2C_SLAVEDSPIC_MODE_ARM_PUMP_OFF					
#define CORN_ROLLS_IN 					I2C_SLAVEDSPIC_MODE_CORN_ROLLS_IN					
#define CORN_ROLLS_OUT 					I2C_SLAVEDSPIC_MODE_CORN_ROLLS_OUT		
#define CORN_ROLLS_STOP 				I2C_SLAVEDSPIC_MODE_CORN_ROLLS_STOP			

#define HARVEST_CORN						I2C_SLAVEDSPIC_MODE_HARVEST_CORN
#define OUT_CORNS								I2C_SLAVEDSPIC_MODE_OUT_CORNS

#define HARVEST_ORANGE					I2C_SLAVEDSPIC_MODE_HARVEST_ORANGE
#define ARM_GOTO_AH							I2C_SLAVEDSPIC_MODE_ARM_GOTO_AH

#define SET_COUNT								I2C_SLAVEDSPIC_MODE_SET_COUNT

#define WAIT               I2C_SLAVEDSPIC_MODE_WAIT
#define INIT               I2C_SLAVEDSPIC_MODE_INIT
#define EXIT               I2C_SLAVEDSPIC_MODE_EXIT

#define TOMATO				0	
#define ORANGE				1

static struct i2c_cmd_slavedspic_set_mode mainboard_command;
static struct vt100 local_vt100;
static volatile uint8_t prev_state;

static volatile uint8_t changed = 0;

uint8_t state_debug = 0;


void state_debug_wait_key_pressed(void)
{
	if (!state_debug)
		return;
	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd)
{
	changed = 1;
	prev_state = mainboard_command.mode;
	memcpy(&mainboard_command, cmd, sizeof(mainboard_command));
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, mainboard_command.mode);
	return 0;
}

/* check that state is the one in parameter and that state did not
 * changed */
uint8_t state_check(uint8_t mode)
{
	int16_t c;
	if (mode != mainboard_command.mode)
		return 0;

	if (changed)
		return 0;

	/* force quit when CTRL-C is typed */
	c = cmdline_getchar();
	if (c == -1)
		return 1;
	if (vt100_parser(&local_vt100, c) == KEY_CTRL_C) {
		mainboard_command.mode = EXIT;
		return 0;
	}
	return 1;
}


uint8_t state_get_mode(void)
{
	return mainboard_command.mode;
}


typedef struct {
#define TOKEN_SYSTEM_STATE_IDLE
#define TOKEN_SYSTEM_STATE_TAKE
#define TOKEN_SYSTEM_STATE_TAKEING
#define TOKEN_SYSTEM_STATE_EJECT
#define TOKEN_SYSTEM_STATE_EJECTING
#define TOKEN_SYSTEM_STATE_SHOW

	uint8_t state;

	uint8_t token_catched;
	uint8_t stop_sensor;
	uint8_t side;
	uint16_t speed;
}token_system_t;

void token_system_init(token_system_t *ts, uint8_t belts_side,
								uint8_t stop_sensor, uint16_t speed)
{
	ts->state = TOKEN_SYSTEM_STATE_IDLE;
	ts->token_catched = 0;
	ts->belts_side = side;
	ts->stop_sensor = stop_sensor;
	ts->speed = speed;

	/* apply to belts */
	belts_mode_set(ts->belts_side, BELTS_MODE_OUT, ts->stop_sensor, ts->speed);
}

void token_system_manage(token_sytem_t *ts)
{

}


static void state_do_token_system(void)
{
	if (!state_check(TOKEN_SYSTEM))
		return;
}



/* wait mode */
static void state_do_wait(void)
{
	if (!state_check(WAIT))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	while (state_check(WAIT));
}

/* init mode */
static void state_do_init(void)
{
	if (!state_check(INIT))
		return;
	state_init();
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	while (state_check(INIT));
}


/* main state machine */
void state_machine(void)
{
	while (state_get_mode() != EXIT) {
		changed = 0;
		state_do_init();
		state_do_token_systems();
	}
}

void state_init(void)
{
	vt100_init(&local_vt100);
	mainboard_command.mode = WAIT;

	token_system_init(&token_system_front, BELTS_SIDE_FRONT, 
							S_FRONT_TOKEN_STOP, TOKEN_SYSTEM_SPEED_DEFAULT);
	token_system_init(&token_system_rear, BELTS_SIDE_REAR, 
							S_REAR_TOKEN_STOP, TOKEN_SYSTEM_SPEED_DEFAULT);
}
#endif
