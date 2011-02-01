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
#define INIT               I2C_SLAVEDSPIC_MODE_INIT

typedef struct {
#define TS_IDLE				0
#define TS_TAKE				1
#define TS_WAITING_STOP		2
#define TS_EJECT				3
#define TS_WAITING_FREE		4
#define TS_STOP				5
#define TS_SHOW				6

	uint8_t state;
	uint16_t speed;
	uint8_t state_changed;
	uint8_t state_rqst;
	uint8_t speed_rqst;

	/* info */
	uint8_t belts_bloqued;
	uint8_t token_catched;

	/* conf */
	uint8_t sensor_stop;
	uint8_t sensor_catched;
	uint8_t side;

}token_system_t;

static struct i2c_cmd_slavedspic_set_mode mainboard_command;
static volatile uint8_t prev_state;
static volatile uint8_t mode_changed = 0;
uint8_t state_debug = 0;

static token_system_t ts[I2C_SIDE_MAX];


/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd)
{
	prev_state = mainboard_command.mode;
	memcpy(&mainboard_command, cmd, sizeof(mainboard_command));
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, mainboard_command.mode);

	/* states machines */
	if (mainboard_command.mode == TOKEN_TAKE){
		ts[mainboard_command.ts.side].state_rqst = TOKEN_SYSTEM_STATE_TAKE;
		ts[mainboard_command.ts.side].speed_rqst = mainboard_command.ts.speed;
		ts[mainboard_command.ts.side].state_changed = 1;
	}
	else if (mainboard_command.mode == TOKEN_EJECT){
		ts[mainboard_command.ts.side].state_rqst = TOKEN_SYSTEM_STATE_EJECT;
		ts[mainboard_command.ts.side].speed_rqst = mainboard_command.ts.speed;
		ts[mainboard_command.ts.side].state_changed = 1;	
	}
	else if (mainboard_command.mode == TOKEN_STOP){
		ts[mainboard_command.ts.side].state_rqst = TOKEN_SYSTEM_STATE_STOP;
		ts[mainboard_command.ts.side].speed_rqst = mainboard_command.ts.speed;
		ts[mainboard_command.ts.side].state_changed = 1;	
	}
	else if (mainboard_command.mode == TOKEN_SHOW){
		ts[mainboard_command.ts.side].state_rqst = TOKEN_SYSTEM_STATE_SHOW;
		ts[mainboard_command.ts.side].speed_rqst = mainboard_command.ts.speed;
		ts[mainboard_command.ts.side].state_changed = 1;
	}
	else
		mode_changed = 1;

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
	if ((mode == mainboard_command.mode) && mode_changed){
		mode_changed = 0;
		return 1;
	}
	return 0;
}

/* debug state machines step to step */
void state_debug_wait_key_pressed(void)
{
	if (!state_debug)
		return;
	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
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
								uint8_t sensor_stop, sensor_catched)
{
	ts->state = TS_IDLE;
	ts->speed = 0;
	ts->state_changed = 0;
	ts->state_rqst;
	ts->speed_rqst;
	ts->token_catched = 0;
	ts->belts_blocked = 0;
	ts->belts_side = side;
	ts->sensor_stop = sensor_stop;
	ts->sensor_catched = sensor_catched;

	/* apply to belts */
	belts_mode_set(ts->belts_side, BELTS_MODE_OUT, ts->stop_sensor, ts->speed);
}


void token_system_manage(token_sytem_t *ts)
{
	/* update state */
	if(ts->state_changed){
		ts->state = ts->state_rqst;
		ts->speed = ts->speed_rqst;
		STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	}

	/* ejecute state */
	switch(ts->state){
		case TS_IDLE:
			break;

		case TS_STATE_TAKE:
			if(!sensor_get(ts->sensor_stop)){
				belts_mode_set(ts->side, BELTS_MODE_IN, ts->speed);
				ts->state = TS_STATE_WAITING_STOP;
			}
			else
				ts->state = TS_IDLE;
			break;

		case TS_STATE_WAITING_STOP:
			/* update info */
			ts->token_catched = sensor_get(ts->sensor_catched);

			/* XXX stop belts when sensor or blocked*/
			if(sensor_get(ts->sensor_stop)){
				belts_mode_set(ts->side, BELTS_MODE_OUT, 0);
				ts->state = TS_IDLE;
			}
			break;

		case TS_STATE_EJECT:
			belts_mode_set(ts->side, BELTS_MODE_OUT, ts->speed);
			ts->state = TS_STATE_WAITING_FREE;
			break;
	
		case TS_STATE_WAITING_FREE:
			/* update info */
			ts->token_catched = sensor_get(ts->sensor_catched);

			/* stop belts when sensor */
			if(!sensor_get(ts->sensor_catched)){
				belts_mode_set(ts->side, BELTS_MODE_OUT, 0);
				ts->state = TS_IDLE;
			}
			break;

		case TS_STATE_STOP:
			belts_mode_set(ts->side, BELTS_MODE_OUT, 0);
			ts->state = TS_IDLE;
			break;
			
		case TS_STATE_SHOW:
			belts_mode_set(ts->side, BELTS_MODE_RIGHT, ts->speed);
			ts->state = TS_IDLE;
			break;

		case default:
			belts_mode_set(ts->side, BELTS_MODE_OUT, 0);
			ts->state = TS_IDLE;
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
	mainboard_command.mode = IDLE;
	changed = 1;

	token_system_init(&ts[I2C_SIDE_FRONT], BELTS_SIDE_FRONT, 
							S_FRONT_TOKEN_STOP, TOKEN_SYSTEM_SPEED_DEFAULT);
	token_system_init(&ts[I2C_SIDE_REAR], BELTS_SIDE_REAR, 
							S_REAR_TOKEN_STOP, TOKEN_SYSTEM_SPEED_DEFAULT);
}
#endif
