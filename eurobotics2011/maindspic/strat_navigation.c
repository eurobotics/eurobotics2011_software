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
	if( (strat_infos.slot_actual.x_index != x_index) ||
		 (strat_infos.slot_actual.y_index != y_index) ) {

		/* save last position */
		strat_infos.slot_before = strat_infos.slot_actual;
	
		/* update actual position*/
		strat_infos.slot_actual.x_index = x_index; 
		strat_infos.slot_actual.y_index = y_index; 

		DEBUG(E_USER_STRAT, "new slot position (%d,%d), before (%d, %d)",
					strat_infos.slot_actual.x_index, strat_infos.slot_actual.y_index,
					strat_infos.slot_before.x_index, strat_infos.slot_before.y_index);
	}

}


/* return next slot depends on direction */ 
struct slot_position strat_calculate_next_slot(uint8_t direction)
{
	struct slot_position slot_ret;

	/* actual slot */
	slot_ret = strat_infos.slot_actual;

	switch(direction) {
		case GO_XPOS:
			slot_ret.x += 1; 
			break;

		case GO_XNEG:
			slot_ret.x -= 1; 
			break;

		case GO_XPOS_YPOS:
			slot_ret.x += 1; 
			slot_ret.y += 1; 
			break;

		case GO_XPOS_YNEG:
			slot_ret.x += 1; 
			slot_ret.y -= 1; 
			break;

		case GO_XNEG_YNEG:
			slot_ret.x -= 1; 
			slot_ret.y -= 1; 
			break;

		case GO_XNEG_YPOS:	
			slot_ret.x -= 1; 
			slot_ret.y += 1; 
			break;

		default:
			break;
	}

	return slot_ret;
}

//strat_calculate_slot_right
//strat_calculate_slot_left
//strat_calculate_slot_forward
//strat_calculate_slot_backward
//strat_calculate_slot_forward_right
//strat_calculate_slot_forward_left
//strat_calculate_slot_backward_right
//strat_calculate_slot_backward_left



void strat_get_next_slot_target(void)
{
	struct slot_position slot;
	
	/* calculate next slot */
	slot = strat_calculate_next_slot(strat_infos.slot_actual);

	/* turn if next slot has flag wall */
	if(strat_infos.slot[slot.x_index][slot.y_index].flag & SLOT_WALL) {

		DEBUG(E_USER_STRAT, "wall slot found");

		/* try right slot, if is a wall also calculate left slot */
		slot = strat_calculate_slot_right(strat_infos.slot);

		if(strat_infos.slot[slot.x_index][slot.y_index].flag & SLOT_WALL) {
			slot = strat_calculate_slot_left(strat_infos.go_direction);
		}
	 
	}

	/* check opponent */

	/* update target slot and direction */
	strat_infos.slot_target = slot_position;
}