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



void strat_goto_prepared_xy_abs(struct trajectory *traj, double x_abs_mm, double y_abs_mm)
{
	if(token_catched(SIDE_FRONT)) {
		trajectory_goto_backward_xy_abs(traj, x_abs_mm,  y_abs_mm);
	}
	else if(token_catched(SIDE_REAR)) {
		trajectory_goto_forward_xy_abs(traj, x_abs_mm,  y_abs_mm);
	}
	else
		trajectory_goto_xy_abs(traj, x_abs_mm,  y_abs_mm);
}

#define NO_MORE_ROOMS	(SIDE_MAX+1)

uint8_t strat_goto_harvesting_xy_abs(struct trajectory *traj, double x_abs_mm, double y_abs_mm)
{
	if(!token_catched(SIDE_FRONT) && !token_catched(SIDE_REAR)) {
		trajectory_goto_forward_xy_abs(traj, x_abs_mm,  y_abs_mm);
		i2c_slavedspic_mode_token_take(SIDE_FRONT);
		return SIDE_FRONT;
	}
	else if(token_catched(SIDE_FRONT)) {
		trajectory_goto_backward_xy_abs(traj, x_abs_mm,  y_abs_mm);
		i2c_slavedspic_mode_token_take(SIDE_REAR);
		return SIDE_REAR;
	}
	else if(token_catched(SIDE_REAR)) {
		trajectory_goto_forward_xy_abs(traj, x_abs_mm,  y_abs_mm);
		i2c_slavedspic_mode_token_take(SIDE_FRONT);
		return SIDE_FRONT;
	}
	else /* no more rooms */
		return NO_MORE_ROOMS;

}

uint8_t sensor_tokens(uint8_t side)
{
	if(side == SIDE_FRONT)
		return (sensor_get(S_TOKEN_FRONT_R) && sensor_get(S_TOKEN_FRONT_R));
	else 
		return (sensor_get(S_TOKEN_REAR_R) && sensor_get(S_TOKEN_REAR_R));
}

/* pick and place line 1 */
#define LINE1_START_X			1150
#define LINE1_START_Y			350
#define LINE1_END_X				1150
#define LINE1_END_Y				1400
#define LINE1_LAST_TOKEN_X		1150
#define LINE1_LAST_TOKEN_Y		1750-50

#define LINE2_LAST_TOKEN_X		800
#define LINE2_LAST_TOKEN_Y		350

#define LINE_CATCHED_TIME 		10

uint8_t line2_only_one_token = 0;

uint8_t strat_harvest_line1(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side;
	uint8_t nb_tokens_catched = 0;


	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* try to pick up last token of line 2 */
	err = strat_pickup_token(COLOR_X(LINE2_LAST_TOKEN_X), LINE2_LAST_TOKEN_Y, SIDE_FRONT);
	if (TRAJ_SUCCESS(err)) {
		line2_only_one_token = 1;
	}

	/* goto line start */
	side = strat_goto_harvesting_xy_abs(&mainboard.traj, COLOR_X(LINE1_START_X),  LINE1_START_Y);

	/* wait traj end or token detected */
	err = WAIT_COND_OR_TRAJ_END(sensor_tokens(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);

	/* down speed */
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);

	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* restore speed */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* check token catched  */
	WAIT_COND_OR_TIMEOUT(token_catched(side), LINE_CATCHED_TIME);
	if(token_catched(side)) { 
		nb_tokens_catched ++;
		DEBUG(E_USER_STRAT, "token catched at first possition (%d)", nb_tokens_catched);
	}

	/* check if we had found the last token of line 2 at begins */
	if(line2_only_one_token) {
		/* place token */
		err = strat_place_token(COLOR_X(strat_infos.slot_grid[1][0].x),
							 strat_infos.slot_grid[1][0].y, 
							 SIDE_FRONT, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}


	/* do straight line harvesting */
	do {

		/* restore speed */
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

		/* go to line end harvesting tokens	*/
		side = strat_goto_harvesting_xy_abs(&mainboard.traj, COLOR_X(LINE1_END_X),  LINE1_END_Y);

		/* wait traj end or token catched */
		err = WAIT_COND_OR_TRAJ_END(sensor_tokens(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	
		/* down speed */
		strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);

		/* wait token catche */
		err = WAIT_COND_OR_TRAJ_END(token_catched(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);

		/* inconditionaly stop */
		trajectory_stop(&mainboard.traj);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
		/* if token catched during the traj */
		WAIT_COND_OR_TIMEOUT(token_catched(side), LINE_CATCHED_TIME);
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
		err = strat_pickup_token_auto(COLOR_X(LINE1_LAST_TOKEN_X), LINE1_LAST_TOKEN_Y);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

	/* restore speed */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* go to place origin */
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(strat_infos.slot_grid[2][4].x),  
								 strat_infos.slot_grid[2][4].y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place tokens */
	err = strat_place_token(COLOR_X(strat_infos.slot_grid[2][5].x),
						 strat_infos.slot_grid[2][5].y, 
						 side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* invert side */
	side ^= 0x01;

	err = strat_place_token(COLOR_X(strat_infos.slot_grid[2][3].x),
						 strat_infos.slot_grid[2][3].y, 
						 side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

 end:
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);
	strat_set_speed(old_spdd, old_spda);
	return err;
}



#define LINE2_START_X			800
#define LINE2_START_Y			1400
#define LINE2_END_X				800
#define LINE2_END_Y				700
	
#define LINE2_FIRST_TOKEN_X	800
#define LINE2_FIRST_TOKEN_Y	1750

uint8_t strat_harvest_line2(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side = SIDE_FRONT;
	uint8_t nb_tokens_catched = 0;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* check token catched carry */
	if(line2_only_one_token)
		nb_tokens_catched = 1;

	/* try to pick up token */
	err = strat_pickup_token(COLOR_X(LINE2_FIRST_TOKEN_X), LINE2_FIRST_TOKEN_Y, SIDE_FRONT);
	if (TRAJ_SUCCESS(err)) {
		nb_tokens_catched ++;
	}

	/* check flag first possition */
	if(nb_tokens_catched < 2) {

		/* goto line start */
		side = strat_goto_harvesting_xy_abs(&mainboard.traj, COLOR_X(LINE2_START_X),  LINE2_START_Y);

		/* wait traj end or token detected */
		err = WAIT_COND_OR_TRAJ_END(sensor_tokens(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	
		/* down speed */
		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);

		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* restore speed */
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);	

		/* check token catched  */
		WAIT_COND_OR_TIMEOUT(token_catched(side), LINE_CATCHED_TIME);
		if(token_catched(side)) { 
			nb_tokens_catched ++;
			DEBUG(E_USER_STRAT, "token catched at second possition (%d)", nb_tokens_catched);
		}
	
		/* do straight line harvesting */
		while( (nb_tokens_catched < 2) ){

			/* restore speed */
			strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	
			/* go to line end harvesting tokens	*/
			side = strat_goto_harvesting_xy_abs(&mainboard.traj, COLOR_X(LINE2_END_X),  LINE2_END_Y);
	
			/* wait traj end or token catched */
			err = WAIT_COND_OR_TRAJ_END(sensor_tokens(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		
			/* speed */
			strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
	
			/* wait token catche */
			err = WAIT_COND_OR_TRAJ_END(token_catched(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);

			/* inconditionaly stop */
			trajectory_stop(&mainboard.traj);
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
			if (!TRAJ_SUCCESS(err))
				ERROUT(err);
		
			/* if token catched during the traj */
			WAIT_COND_OR_TIMEOUT(token_catched(side), LINE_CATCHED_TIME);
			if(token_catched(side)) {
				nb_tokens_catched ++;
				DEBUG(E_USER_STRAT, "token catched in line (%d)", nb_tokens_catched);
			}
			else if (TRAJ_SUCCESS(err))
				break;
			else
				ERROUT(END_ERROR);
	
			/* TODO: opponent management */
		}
	}

	/* restore speed */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* go to place origin */
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(strat_infos.slot_grid[1][1].x),  
								 strat_infos.slot_grid[1][1].y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place tokens */
	if(!line2_only_one_token) {
		err = strat_place_token(COLOR_X(strat_infos.slot_grid[1][0].x),
							 strat_infos.slot_grid[1][0].y, 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* invert side */
		side ^= 0x01;
	}

	err = strat_place_token(COLOR_X(strat_infos.slot_grid[2][1].x),
						 strat_infos.slot_grid[2][1].y, 
						 side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

 end:
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);
	strat_set_speed(old_spdd, old_spda);
	return err;
}

