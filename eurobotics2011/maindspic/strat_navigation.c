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

/* update slot position */
#define GRID_MARGIN 10

void strat_update_slot_position(void)
{
	int16_t x,y;
	int8_t x_index = -1, y_index = -1;
	int8_t i;
	int8_t flags;


	/* TODO: update opponet positon */

	/* get robot possition */
	x = position_get_x_s16(&mainboard.pos);
	y = position_get_y_s16(&mainboard.pos);

	/* get x grid index */
	for(i=0; i<(NB_GRID_LINES_X-1); i++) {

		if( (x > (strat_infos.grid_line_x[i] + GRID_MARGIN)) &&
		    (x < (strat_infos.grid_line_x[i+1] - GRID_MARGIN)) ) {
			
			x_index = i;
			break; 
		}
	}		

	/* get x grid index */
	for(i=0; i<(NB_GRID_LINES_Y-1); i++) {

		if( (y > (strat_infos.grid_line_y[i] + GRID_MARGIN)) &&
		    (y < (strat_infos.grid_line_y[i+1] - GRID_MARGIN)) ) {
			
			y_index = i;
			break; 
		}
	}			

	/* return if not found both index */
	if( (x_index == -1) || (y_index == -1) )
		return;

	/* if it's changed */
	if( (strat_infos.slot_actual.i != x_index) ||
		 (strat_infos.slot_actual.j != y_index) ) {

		IRQ_LOCK(flags);

		/* save last position */
		strat_infos.slot_before = strat_infos.slot_actual;
	
		/* update actual position*/
		strat_infos.slot_actual.i = x_index; 
		strat_infos.slot_actual.j = y_index; 

		/* update flags */
		strat_infos.slot[strat_infos.slot_before.i][strat_infos.slot_before.j].flags &= ~(SLOT_ROBOT);
		strat_infos.slot[strat_infos.slot_actual.i][strat_infos.slot_actual.j].flags |= (SLOT_ROBOT|SLOT_CHECKED|SLOT_VISITED);
		
		IRQ_UNLOCK(flags);

		DEBUG(E_USER_STRAT, "new slot position (%d,%d), before (%d, %d)",
					strat_infos.slot_actual.i, strat_infos.slot_actual.j,
					strat_infos.slot_before.i, strat_infos.slot_before.j);
	}

}


/* place two tokens in bonus points,
	return END_TRAJ if sucess o END_TIMER (only one token in bonus */

#define DATA_UPDATE_TIME 50
uint8_t strat_bonus_point(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side;
	int8_t opp_there; 
	int16_t opp_x, opp_y;
	uint8_t path;
	uint8_t nb_tokens_catched = 0;

#define CHECK_OPPONENT						0
#define GO_LEFT_PATH							1
#define GO_RIGHT_PATH						2
#define WAIT_OPP								3
#define PLACE_TWO_ON_BONUS_POINT			4
#define PLACE_ONE_ON_BONUS_POINT_UP		5
#define PLACE_ONE_ON_BONUS_POINT_DOWN	6
#define PICK_AND_PLACE_NEAR_TOKENS		7
#define CHECK_OPP_BONUS_TOKEN				8
#define PICKUP_OPP_BONUS_TOKEN			9

	uint8_t state = CHECK_OPPONENT;

#define WAIT_OPP_TIMEOUT (90-15)

	while(1) {

		switch(state) {
			case CHECK_OPPONENT:		
				DEBUG(E_USER_STRAT, "CHECK_OPPONENT");
				/* chech opponent */
				time_wait_ms(300);
				opp_there = get_opponent_xy(&opp_x, &opp_y);
	
				/* decide path */
				if(opp_there != -1) {

					if(opp_y < 1050) {
						state = GO_LEFT_PATH;
						path = GO_LEFT_PATH;
					}
					else {
						state = GO_RIGHT_PATH;
						path = GO_RIGHT_PATH;
					}
				}
				else {
					state = GO_RIGHT_PATH;
					path = GO_RIGHT_PATH;
				}
				break;
	
			case GO_LEFT_PATH:
				DEBUG(E_USER_STRAT, "GO_LEFT_PATH");
				/* position 1 */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[3][4].x),
											 strat_infos.slot[3][4].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err)) {
//					/* escape 1 */
//					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[2][3].x),
//											 strat_infos.slot[2][3].y);
//					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
//
//					path = GO_RIGHT_PATH;
//					state = GO_RIGHT_PATH;
					break;
				}
	
				/* position 2 */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][3].x),
											 strat_infos.slot[4][3].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err)) {
					/* escape 2 */
//					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[3][4].x),
//											 strat_infos.slot[3][4].y);
//					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
//
//					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[2][3].x),
//											 strat_infos.slot[2][3].y);
//					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
//
//					path = GO_RIGHT_PATH;
//					state = GO_RIGHT_PATH;	
					break;			
				}
	
				state = WAIT_OPP;		
				break;
	
			case GO_RIGHT_PATH:
				DEBUG(E_USER_STRAT, "GO_RIGHT_PATH");
				/* position 1 */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][1].x-175),
											 strat_infos.slot[4][1].y+175);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err)) {
					/* escape 1 */
//					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[2][3].x),
//											 strat_infos.slot[2][3].y);
//					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
//					path = GO_LEFT_PATH;
//					state = GO_LEFT_PATH;
					break;
				}
	
				/* position 2 */
				strat_limit_speed_disable();
				strat_get_speed(&old_spdd, &old_spda);
				strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);

				strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y, GO_FORWARD);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

				strat_set_speed(old_spdd, old_spda);
				strat_limit_speed_enable();

				if (!TRAJ_SUCCESS(err)) {
					break;
				}


				state = WAIT_OPP;		
				break;
	
			case WAIT_OPP:
				DEBUG(E_USER_STRAT, "WAIT_OPP");
				/* test opponent xy */
				time_wait_ms(300);
				opp_there = get_opponent_xy(&opp_x, &opp_y);
				if(opp_there != -1) {
					if((mainboard.our_color == I2C_COLOR_BLUE)
						 && ((opp_x < 1500) || (opp_y > 1400))) {
						state = PLACE_TWO_ON_BONUS_POINT;
					}
					else if((mainboard.our_color == I2C_COLOR_RED)
						 && ((opp_x > 1500) || (opp_y > 1400))) {
						state = PLACE_TWO_ON_BONUS_POINT;
					}
				}
					
				/* test time */
				if(time_get_s() >= WAIT_OPP_TIMEOUT) {
					if(path == GO_LEFT_PATH) {
						state = PLACE_ONE_ON_BONUS_POINT_UP;
					}
				 	else {	
						state = PLACE_ONE_ON_BONUS_POINT_DOWN;
					}
				}
				break;
	
			case PLACE_TWO_ON_BONUS_POINT:
				DEBUG(E_USER_STRAT, "PLACE_TWO_ON_BONUS_POINT");			
				/* go in the middle */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err))
					break;
	
				/* place one token */
			 	err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][3].x),
												 strat_infos.slot[5][3].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;
	
				/* place the other */
			 	err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][1].x),
												 strat_infos.slot[5][1].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;
	
				state = PICK_AND_PLACE_NEAR_TOKENS;
				break;
	
			case PLACE_ONE_ON_BONUS_POINT_UP:
				DEBUG(E_USER_STRAT, "PLACE_ONE_ON_BONUS_POINT_UP");
				/* goto place slot */
				strat_limit_speed_disable();
				strat_get_speed(&old_spdd, &old_spda);
				strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][3].x),
											 strat_infos.slot[6][3].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				strat_set_speed(old_spdd, old_spda);
				strat_limit_speed_enable();
	
				/* place one token */
			 	err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][3].x),
												 strat_infos.slot[5][3].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;
	
				/* goto place slot */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][3].x),
											 strat_infos.slot[6][3].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				/* place the other */
			 	err = strat_place_token_auto( COLOR_X(strat_infos.slot[6][4].x),
												 strat_infos.slot[6][4].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;

				/* goto place slot */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][3].x),
											 strat_infos.slot[6][3].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				return END_TRAJ;
	
			case PLACE_ONE_ON_BONUS_POINT_DOWN:
				DEBUG(E_USER_STRAT, "PLACE_ONE_ON_BONUS_POINT_DOWN");
				/* goto place slot */
				strat_limit_speed_disable();
				strat_get_speed(&old_spdd, &old_spda);
				strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);

				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][1].x),
											 strat_infos.slot[6][1].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				strat_set_speed(old_spdd, old_spda);
				strat_limit_speed_enable();

				/* place one token */
			 	err = strat_place_token_auto( COLOR_X(strat_infos.slot[5][1].x),
												 strat_infos.slot[5][1].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;

				/* goto place slot */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][1].x),
											 strat_infos.slot[6][1].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				/* place the other */
			 	err = strat_place_token_auto( COLOR_X(strat_infos.slot[6][0].x),
												 strat_infos.slot[6][0].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;
	
				return END_TRAJ;
	

			case PICK_AND_PLACE_NEAR_TOKENS:
				DEBUG(E_USER_STRAT, "PICK_AND_PLACE_NEAR_TOKENS");
				/* slot 1 */
				err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[6][1].x),
							 strat_infos.slot[6][1].y, &side);

				WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
				if(token_catched(side))
					nb_tokens_catched++;

				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;


				/* slot 2 */
				err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[4][3].x),
							 strat_infos.slot[4][3].y, &side);

				WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
				if(token_catched(side))
					nb_tokens_catched++;		
				
				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				/* slot 3 */
				if(nb_tokens_catched == 2) {
					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][1].x),
							 						strat_infos.slot[4][1].y);
					err = wait_traj_end(TRAJ_FLAGS_STD);
				}
				else {
					err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[4][1].x),
								 strat_infos.slot[4][1].y, &side);

					WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
					if(token_catched(side))
						nb_tokens_catched++;
				}

				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;


				/* slot 4 */
				if(nb_tokens_catched == 2) {
					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][3].x),
							 						strat_infos.slot[6][3].y);
					err = wait_traj_end(TRAJ_FLAGS_STD);
				}
				else {
					err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[6][3].x),
								 strat_infos.slot[6][3].y, &side);
	
					WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
					if(token_catched(side))
						nb_tokens_catched++;
				}

				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;


				/* place tokens */
				if(nb_tokens_catched == 2) {
					err = strat_place_token_auto(COLOR_X(strat_infos.slot[4][2].x),
								 strat_infos.slot[4][2].y, &side, GO_FORWARD);

					/* back */
					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
												 strat_infos.slot[5][2].y);
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
					if (!TRAJ_SUCCESS(err))
						break;

					err = strat_place_token_auto(COLOR_X(strat_infos.slot[6][2].x),
								 strat_infos.slot[6][2].y, &side, GO_FORWARD);
				}	
				else if(nb_tokens_catched == 1) {
					err = strat_place_token_auto(COLOR_X(strat_infos.slot[6][2].x),
								 strat_infos.slot[6][2].y, &side, GO_FORWARD);
				}		

				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				state = CHECK_OPP_BONUS_TOKEN;
				break;

			case CHECK_OPP_BONUS_TOKEN:
				DEBUG(E_USER_STRAT, "CHECK_OPP_BONUS_TOKEN");
				/* chech opponent */
				time_wait_ms(300);
				opp_there = get_opponent_xy(&opp_x, &opp_y);
				if(opp_there != -1) {
					if((mainboard.our_color == I2C_COLOR_BLUE)
						 && (opp_x < 1500)) {
						state = PICKUP_OPP_BONUS_TOKEN;
					}
					else if((mainboard.our_color == I2C_COLOR_RED)
						 && (opp_x > 1500)) {
						state = PICKUP_OPP_BONUS_TOKEN;
					}
				}
				break;
			
			case PICKUP_OPP_BONUS_TOKEN:
				DEBUG(E_USER_STRAT, "PICKUP_OPP_BONUS_TOKEN");

				/* go near */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][3].x),
						 						strat_infos.slot[4][3].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][4].x),
						 						strat_infos.slot[4][4].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				/* pickup */
				err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[4][5].x),
							 strat_infos.slot[4][5].y, &side);

				/* go near */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][3].x),
						 						strat_infos.slot[4][3].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				/* wait time */
				while(time_get_s() < 85);

				/* place */
				WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
				err = strat_place_token(COLOR_X(strat_infos.slot[4][4].x),
									 strat_infos.slot[4][4].y, side, GO_FORWARD);
	
				return END_TRAJ;

			default:
				return 0;
		}
	}
	return 0;
}


uint8_t strat_traj_slots(slot_index_t *go, slot_index_t *back)
{
	uint8_t err;

	/* go path */
	while(go->i != NULL) {
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[go->i][go->j].x),
									 strat_infos.slot[go->i][go->j].y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err); 

		go++;
	}
	
	return END_TRAJ;

end:
	/* back path */
	while(back->i != NULL) {
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[back->i][back->j].x),
									 strat_infos.slot[go->i][go->j].y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err); 
		back++;
	}
	
	return err;
}
