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
/* use it in short distance ranges */

#define PICKUP_D_TOKEN_OFFSET			((ROBOT_LENGTH/2)-35)
#define PICKUP_D_NEAR_TOKEN_OFFSET	((ROBOT_LENGTH/2)+100)
#define PICKUP_D_NEAR_TOKEN_NEAR		(70)
#define PICKUP_D_SENSOR_RANGE		500
#define PICKUP_D_NOTINPOINT		80
#define PICKUP_A_CENTER_TOKEN		15.0
#define PICKUP_BELTS_TRIES			5
#define PICKUP_CATCHED_TIME		100
#define PICKUP_SPEED_NOTINPOINT	300
#define PICKUP_SPEED_PICKUP		1200

uint8_t strat_pickup_token(int16_t x, int16_t y, uint8_t side)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	int16_t d_token, d_sign;
	uint8_t try = 0;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

	/* return if we has already a token catched */
	if(token_catched(side))
		ERROUT(END_TRAJ);

	/* XXX fast angle speed    -> problems with centering token */
	/* XXX fast distance speed -> crash token */ 
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

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
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

	/* center token */
	if(side == SIDE_FRONT) {

		/* center front side */

		if(!sensor_get(S_TOKEN_FRONT_R) && !sensor_get(S_TOKEN_FRONT_L)) {
			DEBUG(E_USER_STRAT, "token not found");
			ERROUT(END_TRAJ);
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
			if (TRAJ_SUCCESS(err))  /* we should not reach end */
				ERROUT(END_TRAJ);
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
			ERROUT(END_TRAJ);
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
				ERROUT(END_TRAJ);
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
	if(d_token > PICKUP_D_NEAR_TOKEN_OFFSET) {
		trajectory_d_rel(&mainboard.traj, d_sign*(d_token-PICKUP_D_NEAR_TOKEN_OFFSET));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

	/* go to pick up token */
	strat_set_speed(PICKUP_SPEED_PICKUP, SPEED_ANGLE_FAST);
	i2c_slavedspic_mode_token_take(side);
	d_token = distance_from_robot(x, y);
	trajectory_d_rel(&mainboard.traj, d_sign*(d_token-PICKUP_D_TOKEN_OFFSET));

//	/* set down speed when near */
//	err = WAIT_COND_OR_TRAJ_END( token_catched(side) || 
//										  //(((int32_t)distance_from_robot(x, y)- PICKUP_D_NEAR_TOKEN_OFFSET) <= 0),
//										  (distance_from_robot(x, y) < 300),
//										  TRAJ_FLAGS_SMALL_DIST);
//	DEBUG(E_USER_STRAT, "down speed at d = %d", (int16_t)distance_from_robot(x, y));
//	/* down speed */
//	strat_set_speed(PICKUP_SPEED_PICKUP, SPEED_ANGLE_FAST);
//	i2c_slavedspic_mode_token_take(side);

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
			strat_infos.num_tokens++;
			ERROUT(END_TRAJ);
		}
		else if(TRAJ_SUCCESS(err)) {
			break;
		}
	}

	/* check if token is catched */
	WAIT_COND_OR_TIMEOUT(token_catched(side), PICKUP_CATCHED_TIME);
	if(token_catched(side)) {
		DEBUG(E_USER_STRAT, "token catched!");
		strat_infos.num_tokens++;
		ERROUT(END_TRAJ);
	}
	
	/* not catched yet, try go a little more (belts continue in movement) */
	DEBUG(E_USER_STRAT, "token not in point");
	strat_set_speed(PICKUP_SPEED_NOTINPOINT, PICKUP_SPEED_NOTINPOINT);
	trajectory_d_rel(&mainboard.traj, d_sign*PICKUP_D_NOTINPOINT);
	err = WAIT_COND_OR_TRAJ_END(token_catched(side), TRAJ_FLAGS_SMALL_DIST);

	/* blocking or end traj, wait token catched at end of traj */
	WAIT_COND_OR_TIMEOUT(token_catched(side), PICKUP_CATCHED_TIME);

	if(token_catched(side)) {
		DEBUG(E_USER_STRAT, "token catched!");
		strat_infos.num_tokens++;
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
		err = END_TRAJ;
	}

 end:

	/* slot checked */
	strat_set_slot_flags(x, y, SLOT_CHECKED);

	strat_set_speed(old_spdd, old_spda);
	return err;
}


/* pickup token chossing side automaticaly */
/* we suppose that at least one side is empty */
uint8_t strat_pickup_token_auto(int16_t x, int16_t y, uint8_t *side)
{
	double d_rel;
	double a_rel_rad;

	/* get angle to token xy */
	abs_xy_to_rel_da(x, y, &d_rel, &a_rel_rad);

	if(ABS(a_rel_rad) < (M_PI/2)) {
		if(!token_catched(SIDE_FRONT)) {
			*side = SIDE_FRONT;
			return strat_pickup_token(x, y, SIDE_FRONT);
		}
		else if(!token_catched(SIDE_REAR)) {
			*side = SIDE_REAR;
			return strat_pickup_token(x, y, SIDE_REAR);
		}
		DEBUG(E_USER_STRAT, "no free side");	
	}	
	else {
		if(!token_catched(SIDE_REAR))  {
			*side = SIDE_REAR;
			return strat_pickup_token(x, y, SIDE_REAR);
		}
		else if(!token_catched(SIDE_FRONT)) {
			*side = SIDE_FRONT;
			return strat_pickup_token(x, y, SIDE_FRONT);
		}
		DEBUG(E_USER_STRAT, "no free side");
	}
	/* never shoult be reached */
	return END_TRAJ;
}


/* place a token */	
/* use it in near range distance */
#define PLACE_D_TOKEN_OFFSET		((ROBOT_LENGTH/2)+10)
#define PLACE_D_SAFE					140
#define PLACE_D_SAFE_ABORT			70
#define PLACE_EJECT_TIME			700
#define PLACE_SHOW_TIME				100
#define PLACE_EJECT_TRIES			5
#define PLACE_SPEED_BACK			300

uint8_t strat_place_token(int16_t x, int16_t y, uint8_t side, uint8_t go)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	int16_t d_token, d_sign;
#ifdef TRY_FORCE_EJECT_TOKEN
	uint8_t try = 0;
#endif

	/* check if we have token to place */
	if(!token_catched(side))
		ERROUT(END_TRAJ);

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

	/* XXX fast distance speed -> shoot the token */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST); /* we suppose that distance to place is short and 
		    																 token will not finish eject until reach place */

	/* turn to token */
	if(side == SIDE_FRONT) {
		if(go == GO_FORWARD) {
			d_sign = 1;
			trajectory_turnto_xy(&mainboard.traj, x, y);
		
			/* save time */
			//i2c_slavedspic_mode_token_eject(side);
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
			//i2c_slavedspic_mode_token_eject(side);
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

#ifdef TRY_FORCE_EJECT_TOKEN
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
		ERROUT(END_TRAJ);
	}
#else
	if(token_catched(side)) {
		DEBUG(E_USER_STRAT, "token eject fail, hold inside");
		i2c_slavedspic_mode_token_take(side);

		/* go backward to safe distance from token */
		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
		trajectory_d_rel(&mainboard.traj, (-d_sign)*PLACE_D_SAFE_ABORT);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		ERROUT(err);
	}
	else {
		strat_infos.num_tokens --;
		DEBUG(E_USER_STRAT, "num_tokens = %d", strat_infos.num_tokens);
	}
#endif

	/* go backward to safe distance from token */
	i2c_slavedspic_mode_token_out(side);
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, (-d_sign)*PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	i2c_slavedspic_mode_token_stop(side);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);


 end:
	/* apply flags */
	strat_set_slot_flags(x, y, (SLOT_BUSY|SLOT_AVOID) );
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* place token automaticaly */
/* we suppose that there is at least one token catched */
uint8_t strat_place_token_auto(int16_t x, int16_t y, uint8_t *side, uint8_t go)
{
	double d_rel;
	double a_rel_rad;

	/* get angle to token xy */
	abs_xy_to_rel_da(x, y, &d_rel, &a_rel_rad);

	if(go == GO_FORWARD) {
		if(ABS(a_rel_rad) < (M_PI/2)) {
			if(token_catched(SIDE_FRONT)) {
				*side = SIDE_FRONT;
				return strat_place_token(x, y, SIDE_FRONT, GO_FORWARD);
			}
			else if(token_catched(SIDE_REAR)) {
				*side = SIDE_REAR;
				return strat_place_token(x, y, SIDE_REAR, GO_FORWARD);
			}
			DEBUG(E_USER_STRAT, "no token to place");				
		}	
		else {
			if(token_catched(SIDE_REAR)) {
				*side = SIDE_REAR;
				return strat_place_token(x, y, SIDE_REAR, GO_FORWARD);
			}
			else if(token_catched(SIDE_FRONT)) {
				*side = SIDE_FRONT;
				return strat_place_token(x, y, SIDE_FRONT, GO_FORWARD);
			}
			DEBUG(E_USER_STRAT, "no token to place");	
		}
	}
	else {
		if(ABS(a_rel_rad) < (M_PI/2)) {
			if(token_catched(SIDE_REAR)) {
				*side = SIDE_REAR;
				return strat_place_token(x, y, SIDE_REAR, GO_BACKWARD);
			}
			else if(token_catched(SIDE_FRONT)) {
				*side = SIDE_FRONT;
				return strat_place_token(x, y, SIDE_FRONT, GO_BACKWARD);
			}
			DEBUG(E_USER_STRAT, "no token to place");	
		}	
		else {
			if(token_catched(SIDE_FRONT)) {
				*side = SIDE_FRONT;
				return strat_place_token(x, y, SIDE_FRONT, GO_BACKWARD);
			}
			else if(token_catched(SIDE_REAR)) {
				*side = SIDE_REAR;
				return strat_place_token(x, y, SIDE_REAR, GO_BACKWARD);
			}
			DEBUG(E_USER_STRAT, "no token to place");	
		}
	}
	/* never shoult be reached */
	return END_TRAJ;
}

/* push a token in order to get out an opponent token from slot */
uint8_t strat_push_slot_token(int8_t i, int8_t j)
{
#define PUSH_TOKEN_D	350

	int8_t err = 1;


	/* TODO check opponent */
	if(opponent_is_in_slot(i,j)) {
		DEBUG(E_USER_STRAT, "opponent in pushing slot!");
		ERROUT(END_OBSTACLE);
	}

	/* dangerous slots in front */
	/* near wall */
	if(j == 0 && i == strat_infos.slot_actual.i) {
		/* XXX nothing, is dangerous */
		/* we supose than we pickup before */
		Nop();
	}
	/* near safe zones */
	else if(j == 4 && i == strat_infos.slot_actual.i ) {
		/* XXX nothing, is dangerous */
		/* we supose than we pickup before */
		Nop();
	}
	/* any other, in front and diagonal */
	else {

		/* turn to coordinate */
		trajectory_turnto_xy(&mainboard.traj, strat_infos.slot[i][j].x, strat_infos.slot[i][j].y);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* push possible token */
		trajectory_d_rel(&mainboard.traj, PUSH_TOKEN_D);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

		/* invalidate slot */
		strat_infos.slot[i][j].flags |= SLOT_CHECKED;

		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		DEBUG(E_USER_STRAT, "pushing token");
	}

 end:	
	return err;
}

/* return 1 if slot is a valid for pickup token */
uint8_t strat_is_valid_pickup_slot(int8_t i, int8_t j)
{
	/* skip slots out of pickup range */
	if( (i < 0) || (i > (NB_SLOT_X-1)) )
		return 0;
	if( (j < 0) || (j > (NB_SLOT_Y-1)) )
		return 0;

	/* skip actual slot */
	if(i == strat_infos.slot_actual.i && j == strat_infos.slot_actual.j) {
		//DEBUG(E_USER_STRAT, "skip actual slot (%d, %d)", i, j);
		return 0;
	}

	/* skip slot before */
	if(i == strat_infos.slot_before.i && j == strat_infos.slot_before.j) {
		//DEBUG(E_USER_STRAT, "skip slot before (%d, %d)", i, j);
		return 0;
	}

	/* skip our color slots busy */
	if(strat_infos.slot[i][j].color == mainboard.our_color 
		&& strat_infos.slot[i][j].flags & SLOT_BUSY) {
		//DEBUG(E_USER_STRAT, "skip our color slot (%d, %d)", i, j);
		return 0;
	}

	/* skip our color slots when we are full of tokens */
	if(strat_infos.slot[i][j].color == mainboard.our_color 
		&& token_catched(SIDE_FRONT) && token_catched(SIDE_REAR)) {
		return 0;
	}

	/* skip checked slots */
	if(strat_infos.slot[i][j].flags & SLOT_CHECKED) {
		//DEBUG(E_USER_STRAT, "skip checked slot (%d, %d)", i, j);
		return 0;
	}

	/* skip safe slots */
	if(strat_infos.slot[i][j].flags & SLOT_SAFE)  {
		//DEBUG(E_USER_STRAT, "skip safe slot (%d, %d)", i, j);
		return 0;
	}

	/* skip dificult access bonus slots */
	if(strat_infos.slot_actual.i == 5 && strat_infos.slot_actual.j == 4 
		&&	i == 4 && j == 5) {
		//DEBUG(E_USER_STRAT, "skip danger slot (%d, %d)", i, j);
		return 0;
	}
	if(strat_infos.slot_actual.i == 2 && strat_infos.slot_actual.j == 4 
		&&	i == 3 && j == 5) {
		//DEBUG(E_USER_STRAT, "skip danger slot (%d, %d)", i, j);
		return 0;
	}
	
	/* skip home blue slot */
	if(strat_infos.slot_actual.i == 1 && strat_infos.slot_actual.j == 1 
		&&	i == 0 && j == 0)
		return 0;

	/* skip home red slot */
	if(strat_infos.slot_actual.i == 6 && strat_infos.slot_actual.j == 1 
		&&	i == 7 && j == 0)
		return 0;

	/* TODO: test opponent in near slot */
	if(opponent_is_in_slot(i,j)) {
		//DEBUG(E_USER_STRAT, "skip opponent is in slot (%d, %d)", i, j);
		return 0;
	}

	//DEBUG(E_USER_STRAT, "slot (%d, %d) is valid", i, j);
	return 1;
}

/* return 1 if a valid pickup slot position is found */
uint8_t strat_get_pickup_slot(slot_index_t *slot_pickup)
{
#define DEPENDS_ANGLE_AND_PRIORITY

	int8_t i, j;
	double d_rel, a_rel_rad;
	slot_index_t slot;

#ifndef DEPENDS_ANGLE_AND_PRIORITY
	double a_rel_rad_min = M_2PI;
#else
	double q = 0.0, q_max = 0.0;
#endif
		
	/* init*/
	slot.i = -1;
	slot.j = -1;


	/* return if there's no empty side */
//	if(token_catched(SIDE_FRONT) && token_catched(SIDE_REAR)) {
//		DEBUG(E_USER_STRAT, "no empty side");		
//		return 0;
//	}

	/* check 3x3 area */
	for(i=strat_infos.slot_actual.i-1; i<=strat_infos.slot_actual.i+1; i++) {
		for(j=strat_infos.slot_actual.j-1; j<=strat_infos.slot_actual.j+1; j++) {
	
			/* check if is a valid slot */
			if(!strat_is_valid_pickup_slot(i,j))
				continue;

#ifndef DEPENDS_ANGLE_AND_PRIORITY

			/* get relative angle from front side */
			abs_xy_to_rel_da(strat_infos.slot[i][j].x, strat_infos.slot[i][j].y, &d_rel, &a_rel_rad);
			a_rel_rad = ABS(a_rel_rad);

			/* search minimun angle with front side */
			if(!token_catched(SIDE_FRONT)) {	

				//DEBUG(E_USER_STRAT, "front a_rel_rad = %f", a_rel_rad);
	
				/* if is an minimun angle */
				if(a_rel_rad < a_rel_rad_min) {

					/* update minimun angle */
					a_rel_rad_min = a_rel_rad;
					//DEBUG(E_USER_STRAT, "min_a_rel_rad %f", a_rel_rad_min);	

					/* and slot index */
					slot.i = i;
					slot.j = j;
				}
			}

			/* search minimun angle with rear side */
			if(!token_catched(SIDE_REAR)) {

				/* get angle from rear side */ 
				a_rel_rad = M_PI - a_rel_rad;	

				//DEBUG(E_USER_STRAT, "rear a_rel_rad = %f", a_rel_rad);
	
				/* if is an minimun angle */
				if(a_rel_rad < a_rel_rad_min) {
		
					/* update minimun angle */
					a_rel_rad_min = a_rel_rad;
					//DEBUG(E_USER_STRAT, "min_a_rel_rad %f", a_rel_rad_min);
	
					/* and slot index */
					slot.i = i;
					slot.j = j;
				
				}
			}

#else	/* DEPENDS_ANGLE_AND_PRIORITY */

			/* get relative angle to front */
			abs_xy_to_rel_da(strat_infos.slot[i][j].x, strat_infos.slot[i][j].y, &d_rel, &a_rel_rad);	
			a_rel_rad = ABS(a_rel_rad);

			/* 
			 * cost function:
			 * q = priority + (PI - a_rel)/PI 
			 *
			 * priority++ -> q++
			 * a_rel--	  -> q++
			 *
			 */

			/* evaluate front side */
			if(!token_catched(SIDE_FRONT) || (token_catched(SIDE_FRONT) && token_catched(SIDE_REAR))) {

				/* calcule q */
				q = strat_infos.slot[i][j].prio + (M_PI - a_rel_rad)/M_PI;

				/* give more q to opponent slots */
				if(strat_infos.slot[i][j].color != mainboard.our_color)
					q += (SLOT_PRIOR_INC/2);

				/* get slot with the maximun q */
				if(q > q_max) {
				
					/* update maximun */
					q_max = q;

					/* update side */
					//*side = SIDE_FRONT;

					/* update slot index */
					slot.i = i;
					slot.j = j;
				}
			}

			/* evaluate front side */
			if(!token_catched(SIDE_REAR) || (token_catched(SIDE_FRONT) && token_catched(SIDE_REAR))) {

				/* calcule q, notice: a_rel_rad_rear = M_PI - a_rel_rad */
				q = strat_infos.slot[i][j].prio + a_rel_rad/M_PI;

				/* get slot with the maximun q */
				if(q > q_max) {
				
					/* update maximun */
					q_max = q;

					/* update side */
					//*side = SIDE_REAR;

					/* update slot index */
					slot.i = i;
					slot.j = j;
				}
			}

#endif

		}
	}

	/* return */
	if(slot.i == -1) {
		DEBUG(E_USER_STRAT, "no found pickup slot");
		return 0;
	}
	else {
		*slot_pickup = slot;
		DEBUG(E_USER_STRAT, "found pickup slot (%d,%d)", slot_pickup->i, slot_pickup->j);
		return 1;
	}
}


/* pickup tokens on near 3x3 area slots, return 0 if there aren't more slots */
uint8_t strat_pickup_near_slots(void)
{
	slot_index_t pickup_slot, origin_slot;
	uint8_t side;
	int8_t ret;

	/* save origin */
	origin_slot = strat_infos.slot_actual;

	/* get the first valid pickup slot */
	ret = strat_get_pickup_slot(&pickup_slot);

	/* while there is one pickup/push slot */
	while(ret) {
		
		/* check if we have a free side */
		if(strat_infos.num_tokens < 2)
		{
			/* try pickup token */
			strat_pickup_token_auto(strat_infos.slot[pickup_slot.i][pickup_slot.j].x,
										   strat_infos.slot[pickup_slot.i][pickup_slot.j].y, &side);
		}
		else {

			/* push possible opponent token */
			strat_push_slot_token(pickup_slot.i, pickup_slot.j);
		}


		/* TODO: check opponent is behind */
		if(opponent_is_opposite_side(side)) {
			DEBUG(E_USER_STRAT, "opponent is behind!");
			return 0;
		}

		/* back to origin slot center */
		strat_goto_xy_force(strat_infos.slot[origin_slot.i][origin_slot.j].x, 
								  strat_infos.slot[origin_slot.i][origin_slot.j].y);

		/* get the next valid pickup slot */
		ret = strat_get_pickup_slot(&pickup_slot);
	}

	return ret;
}

/* return 1 if a slot is valid to place */
uint8_t strat_is_valid_place_slot(int8_t i, int8_t j)
{
	/* skip slots out of place range */
	if( (i < 1) || (i > (NB_SLOT_X-2)) )
		return 0;
	if( (j < 0) || (j > (NB_SLOT_Y-1)) )
		return 0;

	/* skip actual slot */
	if(i == strat_infos.slot_actual.i && j == strat_infos.slot_actual.j)
		return 0;

	/* skip opponent color slots */
	if(strat_infos.slot[i][j].color != mainboard.our_color)
		return 0;

	/* skip busy slots */
	if(strat_infos.slot[i][j].flags & SLOT_BUSY)
		return 0;

	/* skip dificult access bonus slots */
	if(strat_infos.slot_actual.i == 5 && strat_infos.slot_actual.j == 4 
		&&	i == 4 && j == 5)
		return 0;

	if(strat_infos.slot_actual.i == 2 && strat_infos.slot_actual.j == 4 
		&&	i == 3 && j == 5)
		return 0;

	/* skip home blue slot */
	if(strat_infos.slot_actual.i == 1 && strat_infos.slot_actual.j == 1 
		&&	i == 0 && j == 0)
		return 0;

	/* skip home red slot */
	if(strat_infos.slot_actual.i == 6 && strat_infos.slot_actual.j == 1 
		&&	i == 7 && j == 0)
		return 0;

	/* skip safe slots from dificult diagonal slot */
	if(strat_infos.slot_actual.i == 3 && strat_infos.slot_actual.j == 4 
		&&	i == 2 && j == 5) {
		//DEBUG(E_USER_STRAT, "skip danger slot (%d, %d)", i, j);
		return 0;
	}
	if(strat_infos.slot_actual.i == 4 && strat_infos.slot_actual.j == 4 
		&&	i == 5 && j == 5) {
		//DEBUG(E_USER_STRAT, "skip danger slot (%d, %d)", i, j);
		return 0;
	}

	/* skip slot with priority lower than threshold */
	if(strat_infos.slot[i][j].prio < strat_infos.conf.th_place_prio)
		return 0;

	/* skip slot not visited for place on path */
	if( (strat_infos.slot[i][j].prio == SLOT_PRIO_PATH)
		&& ((strat_infos.slot[i][j].flags & SLOT_VISITED) == 0) )
		return 0;

	/* TODO: test opponent in near slot */
	if(opponent_is_in_slot(i,j)) {
		//DEBUG(E_USER_STRAT, "skip opponent is in slot (%d, %d)", i, j);
		return 0;
	}

	//DEBUG(E_USER_STRAT, "slot (%d, %d) is valid", i, j);
	return 1;
}

/* return 1 if a place slot position is found */
uint8_t strat_get_place_slot(slot_index_t *slot_place, uint8_t *side)
{
	int8_t i, j;
	double d_rel, a_rel_rad;
	double q = 0.0, q_max = 0.0;
	slot_index_t slot;

	/* init*/
	slot.i = -1;
	slot.j = -1;

	/* return if there's no token catched */
	if(!token_catched(SIDE_FRONT) && !token_catched(SIDE_REAR)) {
		DEBUG(E_USER_STRAT, "no tokens to place");		
		return 0;
	}

	DEBUG(E_USER_STRAT, "front token score = %d", token_side_score(SIDE_FRONT));
	DEBUG(E_USER_STRAT, "rear token score = %d", token_side_score(SIDE_REAR));

	/* check 3x3 area */
	for(i=strat_infos.slot_actual.i-1; i<=strat_infos.slot_actual.i+1; i++) {
		for(j=strat_infos.slot_actual.j-1; j<=strat_infos.slot_actual.j+1; j++) {
	
			/* check if is a valid slot */
 			if(!strat_is_valid_place_slot(i,j))
				continue;

			/* get relative angle to front */
			abs_xy_to_rel_da(strat_infos.slot[i][j].x, strat_infos.slot[i][j].y, &d_rel, &a_rel_rad);	
			a_rel_rad = ABS(a_rel_rad);

			/* 
			 * cost function:
			 * q = priority + (PI - a_rel)/PI 
			 *
			 * priority++ -> q++
			 * a_rel--	  -> q++
			 *
			 */

			/* TODO: play with slots priorities -> SLOT_LEVEL_THRESHOLD*/
			/* TODO: play with token scores -> TOKEN_SCORE_THRESHOLD*/


			/* evaluate front side */
			if(token_catched(SIDE_FRONT)
				&& ( (token_side_score(SIDE_FRONT) <= strat_infos.conf.th_token_score) || token_side_is_lower_score(SIDE_FRONT)) ) {

				/* calcule q */
				q = strat_infos.slot[i][j].prio + (M_PI - a_rel_rad)/M_PI;

				/* get slot with the maximun q */
				if(q > q_max) {
				
					/* update maximun */
					q_max = q;

					/* update side */
					*side = SIDE_FRONT;

					/* update slot index */
					slot.i = i;
					slot.j = j;
				}
			}

			/* evaluate front side */
			if(token_catched(SIDE_REAR)
				&& ( (token_side_score(SIDE_REAR) <= strat_infos.conf.th_token_score) || token_side_is_lower_score(SIDE_REAR)) ) {

				/* calcule q, notice: a_rel_rad_rear = M_PI - a_rel_rad */
				q = strat_infos.slot[i][j].prio + a_rel_rad/M_PI;

				/* get slot with the maximun q */
				if(q > q_max) {
				
					/* update maximun */
					q_max = q;

					/* update side */
					*side = SIDE_REAR;

					/* update slot index */
					slot.i = i;
					slot.j = j;
				}
			}
		}
	}
	
	if(slot.i == -1) {
		DEBUG(E_USER_STRAT, "no found place slot");
		return 0;
	}
	else {
		*slot_place = slot;
		DEBUG(E_USER_STRAT, "found place slot (%d,%d)", slot_place->i, slot_place->j);
		return 1;
	}

}


/* place tokens on near 3x3 area slots, return 0 if there aren't more slots */
uint8_t strat_place_near_slots(void)
{
	slot_index_t place_slot, origin_slot;
	uint8_t side;
	int8_t ret;

	/* save origin */
	origin_slot = strat_infos.slot_actual;

	/* get the first valid pickup slot */
	ret = strat_get_place_slot(&place_slot, &side);

	/* while there is one place slot and we has less than 2 token catched */
	while(ret) {
		
		/* TODO: check 45deg sensors */

		strat_place_token(strat_infos.slot[place_slot.i][place_slot.j].x,
							   strat_infos.slot[place_slot.i][place_slot.j].y, side, GO_FORWARD);
		

		/* TODO: check opponent is behind */
		if(opponent_is_opposite_side(side)) {
			DEBUG(E_USER_STRAT, "opponent is behind!");
			return 0;
		}

		/* back to origin slot center */
		strat_goto_xy_force(strat_infos.slot[origin_slot.i][origin_slot.j].x, 
								  strat_infos.slot[origin_slot.i][origin_slot.j].y);

		/* check if we are empty of tokens */
		if(strat_infos.num_tokens == 0)
			break;

		/* get the next valid pickup slot */
		ret = strat_get_place_slot(&place_slot, &side);
	}

	return ret;
}

/* pickup and place near tokens in 3x3 area where is the robot */
uint8_t strat_pickup_and_place_near_slots(void)
{

	/* TODO: check timeout */ 

	/* set default place priority */
	strat_infos.conf.th_place_prio = SLOT_PRIO_NEAR_GREEN;

	/* place tokens inside */
	strat_place_near_slots();

	/* work on 3x3 area */
	while(1) {
		/* pickup near tokens */
		if(!strat_pickup_near_slots())
			return END_TRAJ; 
	
		/* place picked tokens */
		if(!strat_place_near_slots())
			return END_TRAJ;
	} 

	if(strat_infos.conf.flags & STRAT_CONF_PLACE_ONLYEXT) {

	}
	else {

		/* if we are full of tokens drop one token to path */
		if(strat_infos.num_tokens == 2) {
			strat_infos.conf.th_place_prio = SLOT_PRIO_PATH;
			strat_place_near_slots();
		}
	
		/* if we are full of tokens drop one token to center */
		if(strat_infos.num_tokens == 2) {
			strat_infos.conf.th_place_prio = SLOT_PRIO_CENTER;
			strat_place_near_slots();
		}
	
		/* if we are full of tokens drop one token to ...? */
		if(strat_infos.num_tokens == 2) {
			//strat_infos.th_place_prio = SLOT_PRIO_CENTER;
			//strat_place_near_slots();
		}
	
		/* restore default place priority */
		strat_infos.conf.th_place_prio = SLOT_PRIO_NEAR_GREEN;
	}
}

/**********************************************************************
 * Functions for look for opponent towers 
 *********************************************************************/

typedef struct {
#define RISE_TRIGGER	0
#define FALL_TRIGGER	1
	uint8_t state;
		
	int16_t d_pt_old;

	int16_t x_pt_rise;
	int16_t y_pt_rise;

	int16_t x_pt_fall;
	int16_t y_pt_fall;
	
	int16_t x;
	int16_t y;
	int16_t w;

} match_tower_t;

/* matching of a tower with a laser measure */
uint8_t strat_match_tower( match_tower_t *mt, uint8_t laser_id)
{

#define TOWER_D_TRIGGER				200
#define TOWER_WIDTH_MAX 			150
#define DIST_PT_OPPOPONENT_MIN 	250

	uint8_t ret = 0;
	int16_t obj_width;

	/* laser point coordinates */
	int16_t x_pt, y_pt;
	int16_t d_pt;
	double a_pt_rad;

	/* opponent coordinates */
	int16_t x_opp;
	int16_t y_opp;
	int16_t dist_pt_opp;

	/* robot coordinates */
	int16_t x = position_get_x_s16(&mainboard.pos); 
	int16_t y = position_get_y_s16(&mainboard.pos);
	double a = position_get_a_rad_double(&mainboard.pos);

	/* no valid laser point */
	if(!sensor_get_laser_point_da(laser_id, &d_pt, &a_pt_rad)) {
		//DEBUG(E_USER_STRAT, "no valid laser point");
		d_pt = 5000;
		a_pt_rad = (laser_id == ADC_LASER_R? -(M_PI/2): (M_PI/2));
	}

	//DEBUG(E_USER_STRAT, "laser point (d,a) = (%d,%d)", d_pt, (int16_t)DEG(a_pt_rad));

	/* absolute laser point coordinates */
	if(a < 0)
		a += (2 * M_PI);

	x_pt = (int16_t)((double)d_pt * cos(a + a_pt_rad));
	x_pt += x;
	y_pt = (int16_t)((double)d_pt * sin(a + a_pt_rad));
	y_pt += y;

	//DEBUG(E_USER_STRAT, "pt_abs (%d,%d)", x_pt, y_pt);

	/* return if we are stoped */
	if(ABS(mainboard.speed_d) < 50 && ABS(mainboard.speed_a) < 50) {
//		lasers_set_off();
		mt->state = RISE_TRIGGER;
		mt->x_pt_rise = 0;
		mt->y_pt_rise = 0;
		mt->x_pt_fall = 0;
		mt->y_pt_fall = 0;
		goto end;
	}
//	else if(!lasers_get_state() || !laset_t_setup) {
//		lasers_set_on();
//	}

	/* return if laser point near opponent */
	if(get_opponent_xy(&x_opp, &y_opp) == 0) {

		dist_pt_opp = distance_between(x_opp, y_opp, x_pt, y_pt);
		dist_pt_opp = ABS(dist_pt_opp);
		
		if(dist_pt_opp < DIST_PT_OPPOPONENT_MIN) {

			//DEBUG(E_USER_STRAT, "dist pt-opp = %d", dist_pt_opp);
			//DEBUG(E_USER_STRAT, "pt discarted is near opponent");

			mt->state = RISE_TRIGGER;
			mt->x_pt_rise = 0;
			mt->y_pt_rise = 0;
			mt->x_pt_fall = 0;
			mt->y_pt_fall = 0;
			goto end;	
		}
	}

#ifdef ANY_PT_IS_TOWER

	/* return if laser point is out of accesible playground */
	if(!point_is_in_area(x_pt, y_pt, 450, 1750-90, 2550, 90)) {
		//DEBUG(E_USER_STRAT, "point is out of area");
		break;
	}

	/* found valid tower, we hope! :S */

	/* tower coordinates */
	mt->x = x_pt;
	mt->y = y_pt;
	
	ret = 1;

#else

	/* towers matching */
	switch(mt->state)
	{
		case RISE_TRIGGER:

			/* return if laser point is out of accesible playground */
			if(!point_is_in_area(x_pt, y_pt, 450, 1750-90, 2550, 90)) {
				//DEBUG(E_USER_STRAT, "point is out of area");
				break;
			}

			/* d_new < d_old */
			if(mt->d_pt_old - d_pt > TOWER_D_TRIGGER) {
				mt->x_pt_rise = x_pt;
				mt->y_pt_rise = y_pt;
				mt->x_pt_fall = x_pt;
				mt->y_pt_fall = y_pt;
				mt->state = FALL_TRIGGER;
				//DEBUG(E_USER_STRAT, "rise trigger");
			}
			//DEBUG(E_USER_STRAT, "rise d_pt diff = %d", mt->d_pt_old - d_pt);
			
			break;

		case FALL_TRIGGER:

			/* abort if width of object is too hi */
			obj_width = distance_between(mt->x_pt_fall, mt->y_pt_fall, mt->x_pt_rise, mt->y_pt_rise);
			if( obj_width > TOWER_WIDTH_MAX) {
				mt->x_pt_rise = 0;
				mt->y_pt_rise = 0;
				mt->x_pt_fall = 0;
				mt->y_pt_fall = 0;
				mt->state = RISE_TRIGGER;	
				//DEBUG(E_USER_STRAT, "object too wide");
				break;
			}	
			//DEBUG(E_USER_STRAT, "object width = %d", obj_width);
			//DEBUG(E_USER_STRAT, "fall d_pt diff = %d", d_pt - mt->d_pt_old);

			/* d_new > d_old */
			if(d_pt - mt->d_pt_old > TOWER_D_TRIGGER && obj_width != 0) {

				/* found valid tower, we hope! :S */

				/* tower coordinates */
				if(mt->x_pt_fall > mt->x_pt_rise)
					mt->x = mt->x_pt_rise + (mt->x_pt_fall - mt->x_pt_rise)/2;
				else
					mt->x = mt->x_pt_fall + (mt->x_pt_rise - mt->x_pt_fall)/2;

				if(mt->y_pt_fall > mt->y_pt_rise)
					mt->y = mt->y_pt_rise + (mt->y_pt_fall - mt->y_pt_rise)/2;
				else
					mt->y = mt->y_pt_fall + (mt->y_pt_rise - mt->y_pt_fall)/2;

				/* width of tower */
				mt->w = obj_width;

				ret = 1;
				mt->state = RISE_TRIGGER; 

				//DEBUG(E_USER_STRAT, "laser %s: tower (%d, %d), w = %d",
				//							 laser_id == ADC_LASER_R? "R": "L", mt->x, mt->y, obj_width);

			}
			else {
				mt->x_pt_fall = x_pt;
				mt->y_pt_fall = y_pt;
			}

			break;


		default:
			mt->state = RISE_TRIGGER; 
			break;
	}

#endif

 end:
	/* update d_pt_old */
	mt->d_pt_old = d_pt;
	return ret;
}

/* return 1 a new tower is added succesfully */ 
uint8_t strat_info_add_tower(int16_t x, int16_t y, int16_t w)
{
	int8_t i, j, k;

	/* if there is rooms */
	if(strat_infos.num_towers >= NB_TOWER_MAX) {
		ERROR(E_USER_STRAT, "NO MORE TOWER ROOMS");
		return 0;
	}

#ifdef ANY_PT_IS_TOWER

	/* reduce precision */
	x = x >> 7;
	x = x << 7;

#endif

	/* slot index */
	i = (int8_t)((x-strat_infos.grid_line_x[1])/SLOT_SIZE) + 1;
	j = (int8_t)(y/SLOT_SIZE);

	/* check if already exist in the list */
	for(k = 0; k < strat_infos.num_towers; k++) {
		if(strat_infos.towers[k].i == i && strat_infos.towers[k].j == j) {
			strat_infos.towers[k].c ++;
			return 0; 
		}
	}

	/* add new tower */
	strat_infos.towers[strat_infos.num_towers].i = i;
	strat_infos.towers[strat_infos.num_towers].j = j;
	strat_infos.towers[strat_infos.num_towers].x = x;
	strat_infos.towers[strat_infos.num_towers].y = y;
	strat_infos.towers[strat_infos.num_towers].w = w;
	strat_infos.towers[strat_infos.num_towers].c = 1;


	/* increment number of towers */
	strat_infos.num_towers ++;

	return 1;
}

/* return 1 a exist tower is deleted succesfully */
uint8_t strat_info_del_tower(int8_t i, int8_t j)
{
	int8_t k;

	/* find index */
	for(k = 0; k < strat_infos.num_towers; k++) {
		if(strat_infos.towers[k].i == i && strat_infos.towers[k].j == j)	
			break;
	}

	/* no entry found */
	if(k == strat_infos.num_towers)
		return 0;

	/* del and adjust list elements */
	strat_infos.num_towers --;
	for(k = k; k < strat_infos.num_towers; k++) {
		strat_infos.towers[k] = strat_infos.towers[k+1]; 
	}
	strat_infos.towers[strat_infos.num_towers].i = 0;
	strat_infos.towers[strat_infos.num_towers].j = 0;
	strat_infos.towers[strat_infos.num_towers].x = 0;
	strat_infos.towers[strat_infos.num_towers].y = 0;
	strat_infos.towers[strat_infos.num_towers].w = 0;
	strat_infos.towers[strat_infos.num_towers].c = 0;

	return 1;
}

static uint8_t look_for_towers_enable = 0;

/* enable look for towers */
void strat_look_for_towers_enable(void)
{
	look_for_towers_enable = 1;
}

/* disable look for towers */
void strat_look_for_towers_disable(void)
{
	look_for_towers_enable = 0;
}

/* look for tower of 2 or 3 levels */
void strat_look_for_towers(void)
{
	static match_tower_t mt_left, mt_right;

	/* return if not enable */
	if(!look_for_towers_enable) {
		mt_left.d_pt_old = 0;
		mt_right.d_pt_old = 0;
		return;
	}

	/* matching with right laser */
	if(strat_match_tower( &mt_right, ADC_LASER_R)) {
		strat_info_add_tower(mt_right.x, mt_right.y, mt_right.w);
	}

	/* matching with left laser */
	if(strat_match_tower( &mt_left, ADC_LASER_L)) {
		strat_info_add_tower(mt_left.x, mt_left.y, mt_left.w);
	}
}

/**************************************************************************
 * Functions for look for figures of green area, 
 * think to use during line 1. 
 *************************************************************************/

static uint8_t look_for_figures_enable = 0;

/* enable look for figures */
void strat_look_for_figures_enable(void)
{
	look_for_figures_enable = 1;
}

/* disable look for figures */
void strat_look_for_figures_disable(void)
{
	look_for_figures_enable = 0;
}

/* try to find figures from line 1 */
void strat_look_for_figures(void)
{
#define ANGLE_ABS_MAX	91
#define ANGLE_ABS_MIN	89

#define FIGURE_D_MAX		1100
#define FIGURE_D_MIN		750

#define FIGURE_Y_MARGIN	120
#define FIGURE_1_Y_MIN	(690 - FIGURE_Y_MARGIN)
#define FIGURE_1_Y_MAX	(690 + FIGURE_Y_MARGIN)
#define FIGURE_2_Y_MIN	(970 - FIGURE_Y_MARGIN)
#define FIGURE_2_Y_MAX	(970 + FIGURE_Y_MARGIN)
#define FIGURE_3_Y_MIN	(1250 - FIGURE_Y_MARGIN)
#define FIGURE_3_Y_MAX	(1250 + FIGURE_Y_MARGIN)
#define FIGURE_4_Y_MIN	(1530 - FIGURE_Y_MARGIN)
#define FIGURE_4_Y_MAX	(1530 + FIGURE_Y_MARGIN)

	/* robot position */
	int16_t robot_y = position_get_y_s16(&mainboard.pos);
	int16_t robot_a_deg = position_get_a_deg_s16(&mainboard.pos);

	int16_t laser_d;

	/* return if not enable */
	if(!look_for_figures_enable)
		return;
	
	/* laser distance depends on angle */
	if(robot_a_deg > 0)
		laser_d = sensor_get_laser_distance(ADC_LASER_R);
	else
		laser_d = sensor_get_laser_distance(ADC_LASER_L);

	DEBUG(E_USER_STRAT, "robot (y,a) = (%d, %d)", robot_y, robot_a_deg);
	DEBUG(E_USER_STRAT, "laser d = %d", laser_d);


	/* return if angle is diferent of +/- 90 deg */
	if( ABS(robot_a_deg) > ANGLE_ABS_MAX || ABS(robot_a_deg) < ANGLE_ABS_MIN)
		return;

	/* check if is a figure */
	if( laser_d < FIGURE_D_MIN || laser_d > FIGURE_D_MAX)
		return;

	/* what position */
	if( robot_y > FIGURE_1_Y_MIN && robot_y < FIGURE_1_Y_MAX) {
		strat_infos.slot[0][1].flags &= SLOT_FIGURE; 
		strat_infos.slot[7][1].flags &= SLOT_FIGURE; 
	}
	else if( robot_y > FIGURE_2_Y_MIN && robot_y < FIGURE_2_Y_MAX) {
		strat_infos.slot[0][2].flags &= SLOT_FIGURE; 
		strat_infos.slot[7][2].flags &= SLOT_FIGURE; 
	}
	else if( robot_y > FIGURE_3_Y_MIN && robot_y < FIGURE_3_Y_MAX) {
		strat_infos.slot[0][3].flags &= SLOT_FIGURE; 
		strat_infos.slot[7][3].flags &= SLOT_FIGURE; 
	}
	else if( robot_y > FIGURE_4_Y_MIN && robot_y < FIGURE_4_Y_MAX) {
		strat_infos.slot[0][4].flags &= SLOT_FIGURE; 
		strat_infos.slot[7][4].flags &= SLOT_FIGURE; 
	}
	else
		ERROR(E_USER_STRAT, "no figure pos matched", laser_d);

}
