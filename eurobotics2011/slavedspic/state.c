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

/*   *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011) *  Javier Baliñas Santos <javier@arc-robots.org> * *  Code ported to family of microcontrollers dsPIC from *  state.c,v 1.4 2009/05/27 20:04:07 zer0 Exp. */


#include <math.h>
#include <string.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <rdline.h>
#include <vt100.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "ax12_user.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "state.h"

#if 0

#define STMCH_DEBUG(args...) DEBUG(E_USER_ST_MACH, args)
#define STMCH_NOTICE(args...) NOTICE(E_USER_ST_MACH, args)
#define STMCH_ERROR(args...) ERROR(E_USER_ST_MACH, args)

/* shorter aliases for this file */
#define IDLE               I2C_SLAVEDSPIC_MODE_IDLE
#define INIT               I2C_SLAVEDSPIC_MODE_INIT

typedef struct {
#define TOKEN_SYSTEM_STATE_IDLE
#define TOKEN_SYSTEM_STATE_TAKE
#define TOKEN_SYSTEM_STATE_TAKEING
#define TOKEN_SYSTEM_STATE_EJECT
#define TOKEN_SYSTEM_STATE_EJECTING
#define TOKEN_SYSTEM_STATE_SHOW

	uint8_t state;

	uint8_t belts_bloqued;
	uint8_t token_catched;
	uint8_t sensor_stop;
	uint8_t sensor_catched;
	uint8_t side;
	uint16_t speed;
}token_system_t;

static struct i2c_cmd_slavedspic_set_mode mainboard_command;
//static struct vt100 local_vt100;
static volatile uint8_t prev_state;
static volatile uint8_t changed = 0;
uint8_t state_debug = 0;

static token_system_t ts[I2C_SIDE_MAX];


/* debug state machines step to step */
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

	/* update systems state and parameters */
	if (state_check_update(TOKEN_TAKE)){
		ts[mainboard_command.mode.ts.side].state = TOKEN_SYSTEM_STATE_TAKE;
		ts[mainboard_command.mode.ts.side].speed = mainboard_command.mode.ts.speed;
	}
	else if (state_check_update(TOKEN_EJECT)){
		ts[mainboard_command.mode.ts.side].state = TOKEN_SYSTEM_STATE_EJECT;
		ts[mainboard_command.mode.ts.side].speed = mainboard_command.mode.ts.speed;
	}
	else if (state_check_update(TOKEN_STOP)){
		ts[mainboard_command.mode.ts.side].state = TOKEN_SYSTEM_STATE_STOP;
		ts[mainboard_command.mode.ts.side].speed = mainboard_command.mode.ts.speed;
	}
	else if (state_check_update(TOKEN_SHOW)){
		ts[mainboard_command.mode.ts.side].state = TOKEN_SYSTEM_STATE_SHOW;
		ts[mainboard_command.mode.ts.side].speed = mainboard_command.mode.ts.speed;
	}


	return 0;
}

/* get current mode */
uint8_t state_get_mode(void)
{
	return mainboard_command.mode;
}


/* check that state is the one in parameter and that state changed */
uint8_t state_check_update(uint8_t mode)
{
	if ((mode == mainboard_command.mode) && changed){
		changed = 0;
		return 1;
	}

	return 0;
}


/* init mode */
static void state_do_init(void)
{
	if (!state_check_update(INIT))
		return;
	state_init();
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}

/* token management init */
void token_system_init(token_system_t *ts, uint8_t belts_side,
								uint8_t sensor_stop, sensor_catched, uint16_t speed)
{
	ts->state = TOKEN_SYSTEM_STATE_IDLE;
	ts->token_catched = 0;
	ts->belts_bloqued = 0;
	ts->belts_side = side;
	ts->sensor_stop = sensor_stop;
	ts->sensor_catched = sensor_catched;
	ts->speed = speed;

	/* apply to belts */
	belts_mode_set(ts->belts_side, BELTS_MODE_OUT, ts->stop_sensor, ts->speed);
}


void token_system_manage(token_sytem_t *ts)
{
	/* update flags */
	ts->token_catched = sensor_get(ts->sensor_catched);

	switch(ts->state){
		case TOKEN_SYSTEM_STATE_IDLE:
			break;

		case TOKEN_SYSTEM_STATE_TAKE:
			if(!sensor_get(ts->sensor_stop)){
				belts_mode_set(ts->side, BELTS_MODE_IN, ts->speed);
				ts->state = TOKEN_SYSTEM_STATE_TAKING;
			}
			break;

		case TOKEN_SYSTEM_STATE_TAKING:
			if(sensor_get(ts->sensor_stop)){
				belts_mode_set(ts->side, BELTS_MODE_OUT, 0);
				ts->state = TOKEN_SYSTEM_IDLE;
			}

		case TOKEN_SYSTEM_STATE_EJECT:
			belts_mode_set(ts->side, BELTS_MODE_OUT, ts->speed);
			ts->state = TOKEN_SYSTEM_STATE_EJECTING;
			break;
	
		case TOKEN_SYSTEM_STATE_EJECTING:
			if(!sensor_get(ts->sensor_catched)){
				belts_mode_set(ts->side, BELTS_MODE_OUT, 0);
				ts->state = TOKEN_SYSTEM_IDLE;
			}
			break;

		case TOKEN_SYSTEM_STATE_STOP:
			belts_mode_set(ts->side, BELTS_MODE_OUT, 0);
			ts->state = TOKEN_SYSTEM_IDLE;
			break;
			
		case TOKEN_SYSTEM_STATE_SHOW:
			belts_mode_set(ts->side, BELTS_MODE_RIGHT, ts->speed);
			ts->state = TOKEN_SYSTEM_IDLE;
			break;

		case default:
			belts_mode_set(ts->side, BELTS_MODE_OUT, 0);
			ts->state = TOKEN_SYSTEM_IDLE;
			break;
	}
}


static void state_do_token_system(void)
{

	/* manage systems */
	token_system_manage(&ts[I2C_SIDE_FRONT]);
	token_system_manage(&ts[I2C_SIDE_REAR]);
}


/* main state machine */
void state_machine(void)
{
	state_do_init();
	state_do_token_systems();
}

void state_init(void)
{
	//vt100_init(&local_vt100);
	mainboard_command.mode = IDLE;
	changed = 1;

	token_system_init(&ts[I2C_SIDE_FRONT], BELTS_SIDE_FRONT, 
							S_FRONT_TOKEN_STOP, TOKEN_SYSTEM_SPEED_DEFAULT);
	token_system_init(&ts[I2C_SIDE_REAR], BELTS_SIDE_REAR, 
							S_REAR_TOKEN_STOP, TOKEN_SYSTEM_SPEED_DEFAULT);
}
#endif
