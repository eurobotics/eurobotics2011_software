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


#if 1
#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)
#else
#define ERROUT(e) do {err = 0} while(0) /* XXX no interrups trajs are possible */
#endif

/* pick and place tokens on line 1 */

#define LINE1_START_X			1150
#define LINE1_START_Y			350
#define LINE1_END_X				1150
#define LINE1_END_Y				(1050+100)
#define LINE1_LAST_TOKEN_X		1150
#define LINE1_LAST_TOKEN_Y		(1750-50)

#define LINE2_LAST_TOKEN_X		800
#define LINE2_LAST_TOKEN_Y		350

#define LINE_CATCHED_TIME 		10

uint8_t line2_only_one_token = 0;

uint8_t strat_harvest_line1(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side = SIDE_FRONT;
	uint8_t nb_tokens_catched = 0;

	line2_only_one_token = 0;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* try to pick up last token of line 2 */
	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(LINE2_LAST_TOKEN_X), LINE2_LAST_TOKEN_Y, SIDE_FRONT);
	if (token_catched(SIDE_FRONT)) {
		line2_only_one_token = 1;
	}

	/* goto line start */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE1_START_X), LINE1_START_Y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(LINE1_START_X), LINE1_START_Y);		
	i2c_slavedspic_mode_token_take(side);


	/* wait traj end or token detected */
	err = WAIT_COND_OR_TRAJ_END(sensor_token_side(side), TRAJ_FLAGS_SMALL_DIST);

	/* token detected, down speed */
	if(sensor_token_side(side)) {
		/* down speed and wait end traj */
		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
		/* restore speed */
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	}
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);			

	/* check token catched at first position */
	WAIT_COND_OR_TIMEOUT(token_catched(side), LINE_CATCHED_TIME);
	if(token_catched(side)) { 
		nb_tokens_catched ++;
		DEBUG(E_USER_STRAT, "token catched at first possition (%d)", nb_tokens_catched);
	}

	/* check if we had found the last token of line 2 at begins */
	if(line2_only_one_token) {
		/* place token */
		err = strat_place_token(COLOR_X(strat_infos.slot[2][0].x),
							 strat_infos.slot[2][0].y-70, 
							 SIDE_FRONT, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* turn to second position on line */
		wait_until_opponent_is_far();
		side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(1150), 700);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* in there is a token, pickup it */
		if(sensor_token_side(side)) {
			err = strat_pickup_token(COLOR_X(1150), 700, side);
			if (!TRAJ_SUCCESS(err))
				ERROUT(err);		
		}

		/* go in line */
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
		i2c_slavedspic_mode_token_take(side);	
		trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(1150), 700);		
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
			
	}

	/* do straight line harvesting */
	do {


		/* go to line end harvesting tokens	*/
		/* TODO: divide in two steps, turn and go */
		wait_until_opponent_is_far();
		side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE1_END_X), LINE1_END_Y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* set speed */
		if(!sensor_token_side(side))
			strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

		trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(LINE1_END_X), LINE1_END_Y);		
		i2c_slavedspic_mode_token_take(side);

		/* wait traj end or token detected */
		err = WAIT_COND_OR_TRAJ_END(sensor_token_side(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	
		/* token detected */
		if(sensor_token_side(side)) {

			/* down speed */
			strat_set_speed(1800, SPEED_ANGLE_FAST);
			DEBUG(E_USER_STRAT, "token detected, down speed");

			/* wait token catched or end traj */
			err = WAIT_COND_OR_TRAJ_END(token_catched(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	
			/* if token catched */
			if(token_catched(side)) {
		
				trajectory_stop(&mainboard.traj);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					ERROUT(err);
		
				nb_tokens_catched ++;
				DEBUG(E_USER_STRAT, "token catched in line (%d)", nb_tokens_catched);
			}
			else if (TRAJ_SUCCESS(err))
				break;
		}
		else if (err&END_OBSTACLE) {
			wait_ms(5000);	/* wait to sensor re-enable, we not avoid the opponent */
			continue;		/* TODO: second way / scape sequence */
		}
		else if (!TRAJ_SUCCESS(err))
			ERROUT(err);	

	} while(nb_tokens_catched < 2);

	/* no tokens catched, there are two token in two last positions */
	if( nb_tokens_catched < 1) {
		err = strat_pickup_token_auto(COLOR_X(1150), 1400, &side);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);	

		if(token_catched(side)) {	
			nb_tokens_catched ++;
			DEBUG(E_USER_STRAT, "token catched at end of line (%d)", nb_tokens_catched);
		}
		else
			ERROUT(END_ERROR);
	}

	/* need one more token? */
	if( nb_tokens_catched < 2) {
		wait_until_opponent_is_far();
		err = strat_pickup_token_auto(COLOR_X(LINE1_LAST_TOKEN_X), LINE1_LAST_TOKEN_Y, &side);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		if(token_catched(side)) {	
			nb_tokens_catched ++;
			DEBUG(E_USER_STRAT, "token catched at last position (%d)", nb_tokens_catched);
		}
		else
			ERROUT(END_ERROR);
			
	}

	/* restore speed */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

 place_origin:
	/* go to place origin */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(strat_infos.slot[3][4].x),  
								 strat_infos.slot[3][4].y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (err & END_OBSTACLE) {
		wait_ms(5000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
		goto place_origin;
	}
	else if (!TRAJ_SUCCESS(err))
		ERROUT(err);	

#ifdef LINE1_V1

	/* place tokens */
	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[3][5].x),
						 strat_infos.slot[3][5].y, 
						 side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* invert side */
	side ^= 0x01;

	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[3][3].x),
						 strat_infos.slot[3][3].y, 
						 side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);	

#elif defined(LINE1_V2)

	/* try place token in opponent safe slot */
	if(!opponent_is_in_area(COLOR_X(1500), 2100, COLOR_X(2375), 1250))
	{
		DEBUG(E_USER_STRAT, "no opponent");

		/* goto near safe zone */
		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(strat_infos.slot[5][4].x),  
								 1495);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		/* place token on safe slot */
		wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[5][5].x),
							 strat_infos.slot[5][5].y, 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* go backwards */
		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(strat_infos.slot[5][4].x),  
								 1495);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

	
		/* place the other token in bonus slot */

		/* invert side */
		side ^= 0x01;
	
		wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[3][5].x-80),
							 (strat_infos.slot[3][5].y+50), 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);	
	}
	else {	/* opponent near safe slot */

 place_near:

		/* goto near place slot */
		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(1500),  
								 1575);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		if (err & END_OBSTACLE) {
			wait_ms(5000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
			goto place_near;
		}
		else if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		/* place tokens in bonus slot */
		wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[3][5].x),
							 strat_infos.slot[3][5].y, 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* invert side */
		side ^= 0x01;
	
		wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[3][5].x+80),
							 (strat_infos.slot[3][5].y-80), 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);				
	}
	
back_origin:

	/* back to place origin */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(strat_infos.slot[3][4].x),  
								 strat_infos.slot[3][4].y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (err & END_OBSTACLE) {
		//wait_ms(5000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
		goto back_origin;
	}
	else if (!TRAJ_SUCCESS(err))
		ERROUT(err);
		
#else /* LINE1_V3 */

place_near:

	/* goto near place slot */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj,
							 COLOR_X(1500),  
							 1750);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (err & END_OBSTACLE) {
		wait_ms(5000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
		goto place_near;
	}
	else if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	/* place 2 tokens in bonus slot */
	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[3][5].x-50),
						 strat_infos.slot[3][5].y+50, 
						 side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go backwards */
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -20);
	else
		trajectory_d_rel(&mainboard.traj, 20);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place token, force blocking */
	wait_until_opponent_is_far();
	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[3][5].x+70),
						 (strat_infos.slot[3][5].y-70), GO_FORWARD); //60
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	i2c_slavedspic_mode_token_out(side);
	
	strat_set_speed(1000, 1000);
	trajectory_goto_xy_abs(&mainboard.traj,
								COLOR_X(strat_infos.slot[3][5].x+60),
						 		(strat_infos.slot[3][5].y-60));
	err = wait_traj_end(END_BLOCKING);
	
	/* go backwards slowly */
	i2c_slavedspic_mode_token_out(side);
	strat_set_speed(200, 200);
	trajectory_goto_xy_abs(&mainboard.traj,
							 COLOR_X(strat_infos.slot[3][5].x+350), //68
						 	 (strat_infos.slot[3][5].y-350));
	time_wait_ms(800);
	strat_set_speed(1000, 1000);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);

	i2c_slavedspic_mode_token_stop(side);
			
back_origin:

		/* back to place origin */
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj,
									 COLOR_X(1150+60),  
									 strat_infos.slot[3][4].y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		if (err & END_OBSTACLE) {
			//wait_ms(5000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
			goto back_origin;
		}
		else if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
#endif


 end:
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);
	strat_set_speed(old_spdd, old_spda);
	return err;
}


/* pick and place tokens on line 2 */

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
	uint8_t side;
	uint8_t nb_tokens_catched = 0;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* check token catched carry */
	if(line2_only_one_token)
		nb_tokens_catched = 1;

	/* try to pick up token */
	wait_until_opponent_is_far();
	err = strat_pickup_token_auto(COLOR_X(LINE2_FIRST_TOKEN_X), LINE2_FIRST_TOKEN_Y, &side);
	if (token_catched(side)) {
		nb_tokens_catched ++;
	}

	/* check flag first possition */
	if(nb_tokens_catched < 2) {

		/* goto line start */
		/* TODO: divide in two steps, turn and go */
		wait_until_opponent_is_far();
		side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE2_START_X), LINE2_START_Y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
		trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(LINE2_START_X), LINE2_START_Y);		
		i2c_slavedspic_mode_token_take(side);

		//side = strat_goto_harvesting_xy_abs(&mainboard.traj, COLOR_X(LINE2_START_X),  LINE2_START_Y);

		/* wait traj end or token detected */
		err = WAIT_COND_OR_TRAJ_END(sensor_token_side(side), TRAJ_FLAGS_SMALL_DIST);

		/* token detected */
		if(sensor_token_side(side)) {
			/* down speed and wait end traj */
			strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		}
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
			/* TODO: divide in two steps, turn and go */
			wait_until_opponent_is_far();
			side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE2_END_X), LINE2_END_Y);
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
			if (!TRAJ_SUCCESS(err))
				ERROUT(err);
			trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(LINE2_END_X), LINE2_END_Y);		
			i2c_slavedspic_mode_token_take(side);

			//side = strat_goto_harvesting_xy_abs(&mainboard.traj, COLOR_X(LINE2_END_X),  LINE2_END_Y);
	
			/* wait traj end or token catched */
			err = WAIT_COND_OR_TRAJ_END(sensor_token_side(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		
			/* token detected */
			if(sensor_token_side(side)) {
	
				/* down speed */
				strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_FAST);
	
				/* wait token catched or end traj */
				err = WAIT_COND_OR_TRAJ_END(token_catched(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		
				/* if token catched */
				if(token_catched(side)) {
			
					trajectory_stop(&mainboard.traj);
					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
			
					nb_tokens_catched ++;
					DEBUG(E_USER_STRAT, "token catched in line (%d)", nb_tokens_catched);
				}
				else if (TRAJ_SUCCESS(err))
					break;
			}
			if (err&END_OBSTACLE) {
				wait_ms(5000);		/* XXX wait to sensor re-enable, we not avoid the opponent */
				continue;			/* TODO: sencond way / scape sequence */
			}
			else if (!TRAJ_SUCCESS(err))
				ERROUT(err);		
		}
	}

	/* restore speed */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

 place_origin:
	/* go to place origin */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(strat_infos.slot[2][1].x),  
								 strat_infos.slot[2][1].y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (err&END_OBSTACLE) {
		wait_ms(5000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
		goto place_origin;		
	}
	else if (!TRAJ_SUCCESS(err))
		ERROUT(err);	

	/* place tokens */
	if(!line2_only_one_token) {
		wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[2][0].x),
							 strat_infos.slot[2][0].y-70, 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* invert side */
		side ^= 0x01;
	}

#ifdef LINE2_V1
	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[3][1].x),
						 strat_infos.slot[3][1].y, 
						 side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);	

#else	/* LINE2 V2 */
	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[4][0].x),
						 strat_infos.slot[4][0].y-30, 
						 side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
#endif

 end:
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* pick and place tokens on green area */

#define TOKENS_GREEN_START_X 			625
#define TOKENS_GREEN_START_Y 			700
#define TOKENS_GREEN_D_AVOID_WALL	150
#define TOKEN_INSIDE_TIME				800

uint8_t strat_harvest_green_area(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* goto init possition */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(TOKENS_GREEN_START_X), TOKENS_GREEN_START_Y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);	

	/* pick token 1 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][1].x),
									 strat_infos.slot[0][1].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][1].x),
									 strat_infos.slot[0][1].y, side );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -TOKENS_GREEN_D_AVOID_WALL);
	else
		trajectory_d_rel(&mainboard.traj, TOKENS_GREEN_D_AVOID_WALL);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place token 1 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[1][1].x),
									 strat_infos.slot[1][1].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[1][1].x),
									 strat_infos.slot[1][1].y, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
#define GREEN_AREA_V1
#ifdef GREEN_AREA_V1
	/* pick token 2 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][2].x),
									 strat_infos.slot[0][2].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][2].x),
									 strat_infos.slot[0][2].y, side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */ 
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -(TOKENS_GREEN_D_AVOID_WALL-50));
	else
		trajectory_d_rel(&mainboard.traj, (TOKENS_GREEN_D_AVOID_WALL-50));

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* goto intermediate possition */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, 
									COLOR_X(strat_infos.slot[1][2].x),
									strat_infos.slot[1][2].y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place token 2 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[2][2].x),
									 strat_infos.slot[2][2].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[2][2].x),
									 strat_infos.slot[2][2].y, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick token 3 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][3].x),
									 strat_infos.slot[0][3].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][3].x),
									 strat_infos.slot[0][3].y, side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -TOKENS_GREEN_D_AVOID_WALL);
	else
		trajectory_d_rel(&mainboard.traj, TOKENS_GREEN_D_AVOID_WALL);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick token 4 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][4].x),
									 strat_infos.slot[0][4].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][4].x),
									 strat_infos.slot[0][4].y, side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -TOKENS_GREEN_D_AVOID_WALL);
	else
		trajectory_d_rel(&mainboard.traj, TOKENS_GREEN_D_AVOID_WALL);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);


	/* place token 3 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[1][3].x),
									 strat_infos.slot[1][3].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[1][3].x),
									 strat_infos.slot[1][3].y, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go to intermediate possition */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(450), 1575);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	

	/* pick token 5 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][5].x),
									 strat_infos.slot[0][5].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][5].x),
									 strat_infos.slot[0][5].y, side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -TOKENS_GREEN_D_AVOID_WALL);
	else
		trajectory_d_rel(&mainboard.traj, TOKENS_GREEN_D_AVOID_WALL);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place token 4 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[2][4].x),
									 strat_infos.slot[2][4].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[2][4].x),
									 strat_infos.slot[2][4].y, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place token 5 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[1][5].x),
									 strat_infos.slot[1][5].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[1][5].x),
									 strat_infos.slot[1][5].y+12, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

#else /* GREEN_AREA_V2 */

//	/* pickup token 2 and 3 */
//
//	/* pick token 2 */
//	wait_until_opponent_is_far();
//	err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[0][2].x),
//									 strat_infos.slot[0][2].y, &side );
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//
//	/* go backwards to avoid wall */
//	wait_until_opponent_is_far();
//	if(side == SIDE_FRONT)
//		trajectory_d_rel(&mainboard.traj, -(TOKENS_GREEN_D_AVOID_WALL-50));
//	else
//		trajectory_d_rel(&mainboard.traj, (TOKENS_GREEN_D_AVOID_WALL-50));
//
//	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//	/* place pion (try) on safe zone */
//
//	/* place the other token on slot near green area */
//
//
//	/* pickup token 4 and 5 */



#endif


 end:
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);
	strat_set_speed(old_spdd, old_spda);
	return err;
}
