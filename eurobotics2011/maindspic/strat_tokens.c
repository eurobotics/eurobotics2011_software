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
/* XXX use it in short distance ranges */

#define PICKUP_D_TOKEN_OFFSET			((ROBOT_LENGTH/2)-35)
#define PICKUP_D_NEAR_TOKEN_OFFSET	((ROBOT_LENGTH/2)+70)
#define PICKUP_D_SENSOR_RANGE		500
#define PICKUP_D_NOTINPOINT		80
#define PICKUP_A_CENTER_TOKEN		10.0
#define PICKUP_BELTS_TRIES			5
#define PICKUP_CATCHED_TIME		100
#define PICKUP_SPEED_NOTINPOINT	300
#define PICKUP_SPEED_PICKUP		1000

uint8_t strat_pickup_token(int16_t x, int16_t y, uint8_t side)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	int16_t d_token, d_sign;
	uint8_t try = 0;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

	/* XXX fast angle speed -> problems with centering token */
	/* XXX fast distance speed -> crash token */ 
	/* TODO test speed fasters */
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);

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
		DEBUG(E_USER_STRAT, "go in sensor range");
		trajectory_d_rel(&mainboard.traj, d_sign*(d_token-PICKUP_D_SENSOR_RANGE));
		err = wait_traj_end(TRAJ_FLAGS_STD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

	/* center token */
	if(side == SIDE_FRONT) {

		/* center front side */

		if(!sensor_get(S_TOKEN_FRONT_R) && !sensor_get(S_TOKEN_FRONT_L)) {
			DEBUG(E_USER_STRAT, "token not found");
			ERROUT(END_ERROR);
		}
		else if(sensor_get(S_TOKEN_FRONT_R) != sensor_get(S_TOKEN_FRONT_L)) {
			DEBUG(E_USER_STRAT, "centering token R = %d L = %d",
 					sensor_get(S_TOKEN_FRONT_R), sensor_get(S_TOKEN_FRONT_L));

			if(!sensor_get(S_TOKEN_FRONT_R) && sensor_get(S_TOKEN_FRONT_L)) {
				trajectory_a_rel(&mainboard.traj, PICKUP_A_CENTER_TOKEN);
				err = WAIT_COND_OR_TRAJ_END(sensor_get(S_TOKEN_FRONT_R), TRAJ_FLAGS_SMALL_DIST);
			}
			else if(sensor_get(S_TOKEN_FRONT_R) && !sensor_get(S_TOKEN_FRONT_L)) {
				trajectory_a_rel(&mainboard.traj, -PICKUP_A_CENTER_TOKEN);
				err = WAIT_COND_OR_TRAJ_END(sensor_get(S_TOKEN_FRONT_L), TRAJ_FLAGS_SMALL_DIST);
			}

			/* end of centering */
			if (TRAJ_SUCCESS(err)) /* we should not reach end */
				ERROUT(END_ERROR);
			else {
				/* turnning stop */			
				trajectory_stop(&mainboard.traj);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					ERROUT(err);
			}
		}
	}
	else { /* SIDE_REAR */

		/* center rear side */
		if(!sensor_get(S_TOKEN_REAR_R) && !sensor_get(S_TOKEN_REAR_L)) {
			DEBUG(E_USER_STRAT, "token not found");
			ERROUT(END_ERROR);
		}
		else if(sensor_get(S_TOKEN_REAR_R) != sensor_get(S_TOKEN_REAR_L)) {

			DEBUG(E_USER_STRAT, "centering token R = %d L = %d",
 					sensor_get(S_TOKEN_REAR_R), sensor_get(S_TOKEN_REAR_L));
		
			if(!sensor_get(S_TOKEN_REAR_R) && sensor_get(S_TOKEN_REAR_L)) {
				trajectory_a_rel(&mainboard.traj, -PICKUP_A_CENTER_TOKEN);
				err = WAIT_COND_OR_TRAJ_END(sensor_get(S_TOKEN_REAR_R), TRAJ_FLAGS_SMALL_DIST);
			}
			else if(sensor_get(S_TOKEN_REAR_R) && !sensor_get(S_TOKEN_REAR_L)) {
				trajectory_a_rel(&mainboard.traj, PICKUP_A_CENTER_TOKEN);
				err = WAIT_COND_OR_TRAJ_END(sensor_get(S_TOKEN_REAR_L), TRAJ_FLAGS_SMALL_DIST);
			}
	
			/* end of centering */
			if (TRAJ_SUCCESS(err)) /* we should not reach end */
				ERROUT(END_ERROR);
			else {
				/* turnning stop */			
				trajectory_stop(&mainboard.traj);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					ERROUT(err);
			}
		}
	}
	

	/* go to near token */
	i2c_slavedspic_mode_token_take(side);
	d_token = distance_from_robot(x, y);
	trajectory_d_rel(&mainboard.traj, d_sign*(d_token-PICKUP_D_NEAR_TOKEN_OFFSET));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);


	/* go to pick up token */
	strat_set_speed(PICKUP_SPEED_PICKUP, SPEED_ANGLE_FAST);
	i2c_slavedspic_mode_token_take(side);
	d_token = distance_from_robot(x, y);
	trajectory_d_rel(&mainboard.traj, d_sign*(d_token-PICKUP_D_TOKEN_OFFSET));

	/* wait for token catched, prevent belts blocking */
	while(try < PICKUP_BELTS_TRIES) {
		err = WAIT_COND_OR_TRAJ_END((token_catched(side)|| belts_blocked(side)),
											 TRAJ_FLAGS_SMALL_DIST);
		
		if(belts_blocked(side)) { /* XXX not tested */
			DEBUG(E_USER_STRAT, "belts blocked, try %d", try);
			i2c_slavedspic_mode_token_take(side);
			try ++;
			i2cproto_wait_update();
		}
		else if(token_catched(side)) {
			DEBUG(E_USER_STRAT, "token catched!");
			trajectory_stop(&mainboard.traj);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			ERROUT(END_TRAJ);
		}
		else if(TRAJ_SUCCESS(err)) {
			break;
		}
	}

	/* check if token is catched */
	WAIT_COND_OR_TIMEOUT(token_catched(side), PICKUP_CATCHED_TIME);
	if(token_catched(side))
		ERROUT(END_TRAJ);
	
	/* not catched yet, try go a little more (belts continue in movement) */
	DEBUG(E_USER_STRAT, "token not in point");
	strat_set_speed(PICKUP_SPEED_NOTINPOINT, PICKUP_SPEED_NOTINPOINT);
	trajectory_d_rel(&mainboard.traj, d_sign*PICKUP_D_NOTINPOINT);
	err = WAIT_COND_OR_TRAJ_END(token_catched(side), TRAJ_FLAGS_SMALL_DIST);

	/* blocking or end traj, wait token catched at end of traj */
	WAIT_COND_OR_TIMEOUT(token_catched(side), PICKUP_CATCHED_TIME);

	if(token_catched(side)) {
		DEBUG(E_USER_STRAT, "token catched!");
		trajectory_stop(&mainboard.traj);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		err = END_TRAJ;
	}
	else {	/* token is too far */

		DEBUG(E_USER_STRAT, "token is too far");

		/* stop belts */
		i2c_slavedspic_mode_token_stop(side);
	
		/* go backwards, far from the wall */
		trajectory_d_rel(&mainboard.traj, (-d_sign)*PICKUP_D_NOTINPOINT);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		err = END_ERROR;
	}

 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}


/* pickup token chossing side automaticaly */
uint8_t strat_pickup_token_auto(int16_t x, int16_t y)
{
	double d_rel;
	double a_rel_rad;

	/* get angle to token xy */
	abs_xy_to_rel_da(x, y, &d_rel, &a_rel_rad);

	if(ABS(a_rel_rad) < (M_PI/2)) {
		if(!token_catched(SIDE_FRONT))
			return strat_pickup_token(x, y, SIDE_FRONT);
		else
			return strat_pickup_token(x, y, SIDE_REAR);
	}	
	else {
		if(!token_catched(SIDE_REAR))
			return strat_pickup_token(x, y, SIDE_REAR);
		else
			return strat_pickup_token(x, y, SIDE_FRONT);
	}
}



///* pickup a token depends tokens catched before */
//uint8_t strat_pickup_token_auto(int16_t x, int16_t y)
//{
//	if(token_catched(SIDE_FRONT))
//		return strat_pickup_token(x, y, SIDE_REAR);
//	else if(token_catched(SIDE_REAR))
//		return strat_pickup_token(x, y, SIDE_FRONT);
//	else
//		return END_ERROR;
//}

/* place a token */	
/* XXX use it in near range distance */
#define PLACE_D_TOKEN_OFFSET		((ROBOT_LENGTH/2)+10)
#define PLACE_D_SAFE					140
#define PLACE_EJECT_TIME			700
#define PLACE_SHOW_TIME				100
#define PLACE_EJECT_TRIES			5
#define PLACE_SPEED_BACK			300

uint8_t strat_place_token(int16_t x, int16_t y, uint8_t side, uint8_t go)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	int16_t d_token, d_sign;
	uint8_t try = 0;

	/* check if we have token to place */
	if(!token_catched(side))
		ERROUT(END_TRAJ);

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

	/* XXX fast distance speed -> shoot the token */
	/* TODO test speed fasters */
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);

	/* turn to token */
	if(side == SIDE_FRONT) {
		if(go == GO_FORWARD) {
			d_sign = 1;
			trajectory_turnto_xy(&mainboard.traj, x, y);
		
			/* save time */
			i2c_slavedspic_mode_token_eject(side);
		}
		else {
			d_sign = -1;
			trajectory_turnto_xy_behind(&mainboard.traj, x, y);
		}
	}
	else {
		if(go == GO_FORWARD) {
			d_sign = -1;
			trajectory_turnto_xy_behind(&mainboard.traj, x, y);

			/* save time */
			i2c_slavedspic_mode_token_eject(side);
		}
		else {
			d_sign = 1;
			trajectory_turnto_xy(&mainboard.traj, x, y);
		}
	}
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go to place */
	if(go == GO_FORWARD) {
		
		/* eject token */
		i2c_slavedspic_mode_token_eject(side);

		/* go token position */
		d_token = distance_from_robot(x, y);
		trajectory_d_rel(&mainboard.traj, d_sign*(d_token-PLACE_D_TOKEN_OFFSET));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* wait token free */
		WAIT_COND_OR_TIMEOUT(!token_catched(side), PLACE_EJECT_TIME);
	
	}
	else {
		
		/* go token position with token at our back */
		d_token = distance_from_robot(x, y);
		trajectory_d_rel(&mainboard.traj, d_sign*(d_token+PLACE_D_TOKEN_OFFSET));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
		
		/* eject token */
		i2c_slavedspic_mode_token_eject(side);

		/* wait token free */
		WAIT_COND_OR_TIMEOUT(!token_catched(side), PLACE_EJECT_TIME);

		/* invert d_sign for go safe distance after */	
		d_sign = -d_sign;
	}

	/* XXX check eject error and try fixed */
	while(token_catched(side) && (try < PLACE_EJECT_TRIES)) {
		i2c_slavedspic_mode_token_show(side);
		wait_ms(PLACE_SHOW_TIME);
		i2c_slavedspic_mode_token_eject(side);
		wait_ms(PLACE_EJECT_TIME);
		try ++;
	}
	if(token_catched(side)) {
		DEBUG(E_USER_STRAT, "token eject fail");
		ERROUT(END_ERROR);
	}

	/* go backward to safe distance from token */
	i2c_slavedspic_mode_token_eject(side);
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, (-d_sign)*PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);


 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* place token automaticaly */
uint8_t strat_place_token_auto(int16_t x, int16_t y, uint8_t go)
{
	double d_rel;
	double a_rel_rad;

	/* get angle to token xy */
	abs_xy_to_rel_da(x, y, &d_rel, &a_rel_rad);

	if(go == GO_FORWARD) {
		if(ABS(a_rel_rad) < (M_PI/2)) {
			if(token_catched(SIDE_FRONT))
				return strat_place_token(x, y, SIDE_FRONT, GO_FORWARD);
			else
				return strat_place_token(x, y, SIDE_REAR, GO_FORWARD);
		}	
		else {
			if(token_catched(SIDE_REAR))
				return strat_place_token(x, y, SIDE_REAR, GO_FORWARD);
			else
				return strat_place_token(x, y, SIDE_FRONT, GO_FORWARD);
		}
	}
	else {
		if(ABS(a_rel_rad) < (M_PI/2)) {
			if(token_catched(SIDE_FRONT))
				return strat_place_token(x, y, SIDE_REAR, GO_BACKWARD);
			else
				return strat_place_token(x, y, SIDE_FRONT, GO_BACKWARD);
		}	
		else {
			if(token_catched(SIDE_REAR))
				return strat_place_token(x, y, SIDE_FRONT, GO_BACKWARD);
			else
				return strat_place_token(x, y, SIDE_REAR, GO_BACKWARD);
		}
	}
}
