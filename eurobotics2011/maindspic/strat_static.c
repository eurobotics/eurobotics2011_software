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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>


#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)

/* pick and place line 1 */
#define LINE1_START_X			1150
#define LINE1_START_Y			350
#define LINE1_END_X				1150
#define LINE1_END_Y				1400
#define LINE1_LAST_TOKEN_X		1150
#define LINE1_LAST_TOKEN_Y		1730
#define LINE1_CATCHED_TIME 	100

uint8_t strat_harvest_line1(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side = SIDE_FRONT;
	uint8_t nb_tokens_catched = 0;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

	/* TODO: try to pick up last token of line 2 */

	/* goto line start */
	i2c_slavedspic_mode_token_take(side);
	trajectory_goto_forward_xy_abs(&mainboard.traj, COLOR_X(LINE1_START_X),  LINE1_START_Y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* check token catched  */
	WAIT_COND_OR_TIMEOUT(token_catched(side), LINE1_CATCHED_TIME);
	if(token_catched(side)) { 
		nb_tokens_catched ++;
		DEBUG(E_USER_STRAT, "token catched at first possition (%d)", nb_tokens_catched);
	}

	/* do straight line harvesting */
	do {

		/* go to line end harvesting tokens	*/
		if(nb_tokens_catched == 0) {
			side = SIDE_FRONT;
			trajectory_goto_forward_xy_abs(&mainboard.traj, COLOR_X(LINE1_END_X),  LINE1_END_Y);
		}
		else {
			side = SIDE_REAR;
			trajectory_goto_backward_xy_abs(&mainboard.traj, COLOR_X(LINE1_END_X),  LINE1_END_Y);
		}
		
		/* prepare for take a token */
		i2c_slavedspic_mode_token_take(side);

		/* wait traj end or token catched */
		err = WAIT_COND_OR_TRAJ_END(token_catched(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	
		/* inconditionaly stop */
		trajectory_stop(&mainboard.traj);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
		
		/* if token catched during the traj */
		WAIT_COND_OR_TIMEOUT(token_catched(side), LINE1_CATCHED_TIME);
		if(token_catched(side)) {
			nb_tokens_catched ++;
			DEBUG(E_USER_STRAT, "token catched in line (%d)", nb_tokens_catched);
		}
		else if (TRAJ_SUCCESS(err))
			break;
		else
			ERROUT(END_ERROR);

		/* TODO: opponent management */

	} while( (nb_tokens_catched < 2) );

	/* need one more token? */
	if( nb_tokens_catched < 2) {
		err = strat_pickup_token(COLOR_X(LINE1_LAST_TOKEN_X), LINE1_LAST_TOKEN_Y, side);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

	/* go to place origin */
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(1325),  
								 1575);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place tokens */
	err = strat_place_token(COLOR_X(1325),
						 1925, 
						 SIDE_REAR, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	err = strat_place_token(COLOR_X(1325),
						 1225, 
						 SIDE_FRONT, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go to safe position */
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(1325),  
								 1575);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

