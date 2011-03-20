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


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)

/* pick up a token */
#define PICKUP_D_TOKEN_OFFSET		((TOKEN_DIAMETER/2)-30)
#define PICKUP_D_SENSOR_RANGE		500
#define PICKUP_A_CENTER_TOKEN		10.0
#define PICKUP_BELTS_TRIES			5
#define PICKUP_CATCHED_TIME		100

uint8_t strat_pickup_token(int16_t x, int16_t y, uint8_t side)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	int16_t d_token, d_sign;
	uint8_t try = 0;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

	/* turn to token */
	if(side == SIDE_FRONT) {
		d_sign = 1;
		trajectory_turnto_xy(&mainboard.traj, x, y);
	}
	else {
		d_sign = -1;
		trajectory_turnto_xy_behind(&mainboard.traj, x, y);
	}
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	/* check sensor range */
	d_token = distance_from_robot(x, y);
	if(d_token > PICKUP_D_SENSOR_RANGE){
		
		/* go in range */
		trajectory_d_rel(&mainboard.traj, d_sign*(d_token-PICKUP_D_SENSOR_RANGE));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

	/* center token */
	if(side == SIDE_FRONT) {
		if(!sensor_get(S_TOKEN_FRONT_R) && sensor_get(S_TOKEN_FRONT_L)){
			trajectory_a_rel(&mainboard.traj, -PICKUP_A_CENTER_TOKEN);
			err = WAIT_COND_OR_TRAJ_END(sensor_get(S_TOKEN_FRONT_R), TRAJ_FLAGS_SMALL_DIST);
			trajectory_stop(&mainboard.traj);
			if (TRAJ_SUCCESS(err)) /* we should not reach end */
				ERROUT(END_ERROR);
		}
		else if(sensor_get(S_TOKEN_FRONT_R) && !sensor_get(S_TOKEN_FRONT_L)){
			trajectory_a_rel(&mainboard.traj, PICKUP_A_CENTER_TOKEN);
			err = WAIT_COND_OR_TRAJ_END(sensor_get(S_TOKEN_FRONT_L), TRAJ_FLAGS_SMALL_DIST);
			trajectory_stop(&mainboard.traj);
			if (TRAJ_SUCCESS(err)) /* we should not reach end */
				ERROUT(END_ERROR);
		}
	}
	else {
		if(!sensor_get(S_TOKEN_REAR_R) && sensor_get(S_TOKEN_REAR_L)){
			trajectory_a_rel(&mainboard.traj, PICKUP_A_CENTER_TOKEN);
			err = WAIT_COND_OR_TRAJ_END(sensor_get(S_TOKEN_REAR_R), TRAJ_FLAGS_SMALL_DIST);
			trajectory_stop(&mainboard.traj);
			if (TRAJ_SUCCESS(err)) /* we should not reach end */
				ERROUT(END_ERROR);
		}
		else if(sensor_get(S_TOKEN_REAR_R) && !sensor_get(S_TOKEN_REAR_L)){
			trajectory_a_rel(&mainboard.traj, -PICKUP_A_CENTER_TOKEN);
			err = WAIT_COND_OR_TRAJ_END(sensor_get(S_TOKEN_REAR_L), TRAJ_FLAGS_SMALL_DIST);
			trajectory_stop(&mainboard.traj);
			if (TRAJ_SUCCESS(err)) /* we should not reach end */
				ERROUT(END_ERROR);
		}
	}

	/* go to pick up token */
	i2c_slavedspic_mode_token_take(side);
	d_token = distance_from_robot(x, y);
	trajectory_d_rel(&mainboard.traj, d_sign*(d_token-PICKUP_D_TOKEN_OFFSET));

	/* wait for token catched, prevent belts blocking */
	while(try < PICKUP_BELTS_TRIES) {
		err = WAIT_COND_OR_TRAJ_END((token_catched(side)|| belts_blocked(side)),
											 TRAJ_FLAGS_SMALL_DIST);
		if(belts_blocked(side)) {
			DEBUG(E_USER_STRAT, "belts blocked, try %d", try);
			i2c_slavedspic_mode_token_take(side);
			try ++;
			i2cproto_wait_update();
		}
		else if(TRAJ_SUCCESS(err))
			break;
	}


	/* check if token is catched */
	WAIT_COND_OR_TIMEOUT(token_catched(side), PICKUP_CATCHED_TIME);
	if(token_catched(side))
		ERROUT(END_TRAJ);
	
	/* not catched yet, try go a little more (belts continue in movement) */
	DEBUG(E_USER_STRAT, "token not in point");
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
	trajectory_d_rel(&mainboard.traj, d_sign*PICKUP_D_TOKEN_OFFSET);
	err = WAIT_COND_OR_TRAJ_END(token_catched(side),
										 TRAJ_FLAGS_SMALL_DIST);
	if(token_catched(side))
		err = END_TRAJ;
	else {
		DEBUG(E_USER_STRAT, "token not found");
		err = END_ERROR;
	}

 end:
	/* restore speed */
	strat_set_speed(old_spdd, old_spda);
	return err;
}
