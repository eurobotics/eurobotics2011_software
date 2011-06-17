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

#ifndef HOST_VERSION
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

#endif

struct strat_infos strat_infos = { 
	/* conf */
	.conf = {
		.flags = 0,
	},

	/* grid slots 
	.slot[X][Y] = { .x,	.y ,  	.color,       		.prio,   					.flags, 			.flags_poly},  */
	.slot[0][0] = { 200,	200,		SLOT_BLUE,			SLOT_PRIO_GREEN,			0, 				0, },
	.slot[0][1] = { 200,	690,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			0, 				0, },
	.slot[0][2] = { 200,	970,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			0, 				0, },
	.slot[0][3] = { 200,	1250,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			0, 				0, },
	.slot[0][4] = { 200,	1530,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			0, 				0, },
	.slot[0][5] = { 200,	1810,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			0, 				0, },

	.slot[1][0] = { 625,	175,		SLOT_RED, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[1][1] = { 625,	525,		SLOT_BLUE,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[1][2] = { 625,	875,		SLOT_RED,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[1][3] = { 625,	1225,		SLOT_BLUE,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[1][4] = { 625,	1575,		SLOT_RED,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[1][5] = { 625,	1865+10,	SLOT_BLUE,			SLOT_PRIO_SAFE,			SLOT_SAFE,		0, },

	.slot[2][0] = { 975,	175,		SLOT_BLUE, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[2][1] = { 975,	525,		SLOT_RED,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[2][2] = { 975,	875,		SLOT_BLUE,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[2][3] = { 975,	1225,		SLOT_RED,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[2][4] = { 975,	1575,		SLOT_BLUE,			SLOT_PRIO_NEAR_SAFE,		0, 				0,	},
	.slot[2][5] = { 975, 1865+10,	SLOT_RED,			SLOT_PRIO_SAFE,			SLOT_SAFE, 		0, },

	.slot[3][0] = { 1325, 175,		SLOT_RED, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[3][1] = { 1325, 525,		SLOT_BLUE,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[3][2] = { 1325, 875,		SLOT_RED,			SLOT_PRIO_CENTER,			0,					0, },
	.slot[3][3] = { 1325, 1225,	SLOT_BLUE,			SLOT_PRIO_CENTER,			0, 				0, },
	.slot[3][4] = { 1325, 1575,	SLOT_RED,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[3][5] = { 1325, 1925,	SLOT_BLUE,			SLOT_PRIO_BONUS_WALL,	0, 				0, },

	.slot[4][0] = { 1675, 175,		SLOT_BLUE, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[4][1] = { 1675, 525,		SLOT_RED,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[4][2] = { 1675, 875,		SLOT_BLUE,			SLOT_PRIO_CENTER,			0, 				0, },
	.slot[4][3] = { 1675, 1225,	SLOT_RED,			SLOT_PRIO_CENTER,			0, 				0, },
	.slot[4][4] = { 1675, 1575,	SLOT_BLUE,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[4][5] = { 1675, 1925,	SLOT_RED,			SLOT_PRIO_BONUS_WALL,	0, 				0, },

	.slot[5][0] = { 2025, 175,		SLOT_RED, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[5][1] = { 2025, 525,		SLOT_BLUE,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[5][2] = { 2025, 875,		SLOT_RED,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[5][3] = { 2025, 1225,	SLOT_BLUE,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[5][4] = { 2025, 1575,	SLOT_RED,			SLOT_PRIO_NEAR_SAFE,		0, 				0, },
	.slot[5][5] = { 2025, 1865+10,SLOT_BLUE,			SLOT_PRIO_SAFE,			SLOT_SAFE, 		0, },

	.slot[6][0] = { 2375, 175,		SLOT_BLUE, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[6][1] = { 2375, 525,		SLOT_RED,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[6][2] = { 2375, 875,		SLOT_BLUE,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[6][3] = { 2375, 1225,	SLOT_RED,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[6][4] = { 2375, 1575,	SLOT_BLUE,			SLOT_PRIO_NEAR_SAFE,		0, 				0, },
	.slot[6][5] = { 2375, 1865+10,SLOT_RED,			SLOT_PRIO_SAFE,			SLOT_SAFE, 		0, },

	.slot[7][0] = { 2800, 200,		SLOT_RED,			SLOT_PRIO_GREEN,			0, 				0, },
	.slot[7][1] = { 2800, 690,		SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			0, 				0, },
	.slot[7][2] = { 2800, 970,		SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			0, 				0, },
	.slot[7][3] = { 2800, 1250,	SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			0, 				0, },
	.slot[7][4] = { 2800, 1530,	SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			0, 				0, },
	.slot[7][5] = { 2800, 1810,	SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			0, 				0, },

	/* grid lines */
	.grid_line_x = { 0, 450, 800, 1150, 1500, 1850, 2200, 2550, 3000 },
	.grid_line_y = { 0, 350, 700, 1050, 1400, 1750, 2100 },

};



/*************************************************************/

/*                  INIT                                     */

/*************************************************************/

void strat_set_bounding_box(void)
{

#ifdef AREA_BBOX_6X5
	/* area 6x5 */
	strat_infos.area_bbox.x1 = 625;
	strat_infos.area_bbox.y1 = 220;
	strat_infos.area_bbox.x2 = 2375;
	strat_infos.area_bbox.y2 = 1575;
#else
	/* area 4x4	*/
	strat_infos.area_bbox.x1 = 945;
	strat_infos.area_bbox.y1 = 495;
	strat_infos.area_bbox.x2 = 2055;
	strat_infos.area_bbox.y2 = 1575;
#endif

	polygon_set_boundingbox(strat_infos.area_bbox.x1,
				strat_infos.area_bbox.y1,
				strat_infos.area_bbox.x2,
				strat_infos.area_bbox.y2);
}

#ifndef HOST_VERSION

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
	time_reset();
	interrupt_traj_reset();
	mainboard.flags =  DO_ENCODERS | DO_CS | DO_RS |
							 DO_POS | DO_BD | DO_POWER | DO_OPP;

	strat_dump_conf();
	strat_dump_infos(__FUNCTION__);
}

/* display curret strat configuration */
void strat_dump_conf(void)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("-- conf --\r\n"));

	/* thresholds */
	printf(PSTR("th_place_prio = %d \r\n"), strat_infos.conf.th_place_prio);
	printf(PSTR("th_token_score = %d \r\n"),strat_infos.conf.th_token_score);

	
}

int8_t strat_print_flag(uint8_t i, uint8_t j)
{

	/* robot slot position */
	if(strat_infos.slot[i][j].flags & SLOT_ROBOT)
		return 'R';	

	/* opponent slot position */
	if(strat_infos.slot[i][j].flags & SLOT_OPPONENT)
		return 'P';	

	/* slot busy */
	if(strat_infos.slot[i][j].flags & SLOT_BUSY)
		return 'O';	

	/* slot checked */
	if(strat_infos.slot[i][j].flags & SLOT_CHECKED)
		return 'X';	

	/* green area */
	if( (i == 0 || i == 7) && j > 0)
		return ' '; 

	/* any slot */
	return '_'; 

}

/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	int8_t j;

	if (!strat_infos.dump_enabled)
		return;

	printf(PSTR("%s() dump strat infos:\r\n"), caller);

	/* tokens catched */
	printf(PSTR("num_tokens = %d \r\n"), strat_infos.num_tokens);

	/* slot position */
	printf(PSTR("slot_actual = (%d,%d) \r\n"),
			strat_infos.slot_actual.i, strat_infos.slot_actual.j);
	printf(PSTR("slot_before = (%d,%d) \r\n"),
			strat_infos.slot_before.i, strat_infos.slot_before.j );


	/* slots flags */
	for(j=5; j>=0; j--) { 
		if(j==5) {
			printf(PSTR("y  _______________  \r\n"));
			printf(PSTR("%d |%c|%c_%c|%c|%c|%c_%c|%c|"),
					j,
					strat_print_flag(0, j),
					strat_print_flag(1, j),
					strat_print_flag(2, j),
					strat_print_flag(3, j),
					strat_print_flag(4, j),
					strat_print_flag(5, j),
					strat_print_flag(6, j),
					strat_print_flag(7, j) );
		}
		else {
			printf(PSTR("%d |%c|%c|%c|%c|%c|%c|%c|%c|"),
					j,
					strat_print_flag(0, j),
					strat_print_flag(1, j),
					strat_print_flag(2, j),
					strat_print_flag(3, j),
					strat_print_flag(4, j),
					strat_print_flag(5, j),
					strat_print_flag(6, j),
					strat_print_flag(7, j) );
		}
		printf(PSTR(" \r\n"));
	}

	printf(PSTR("   0 1 2 3 4 5 6 7 x \r\n"));
	

//	printf(PSTR("y  _______________  \r\n"));
//	printf(PSTR("5 | |___|_|_|___| | \r\n"));
//	printf(PSTR("4 | |_|_|_|_|_|_| | \r\n"));
//	printf(PSTR("3 | |P|_|_|X|O|X| | \r\n"));
//	printf(PSTR("2 | |_|_|_|O|R|O| | \r\n"));
//	printf(PSTR("1 |_|_|_|_|X|O|X|_| \r\n"));
//	printf(PSTR("0 |_|_|_|_|_|_|_|_| \r\n"));
//	printf(PSTR("   0 1 2 3 4 5 6 7 x\r\n"));


	/* towers found */
	printf(PSTR(" num_towers = %d \r\n"), strat_infos.num_towers);

	if(strat_infos.num_towers) {
		printf(PSTR(" tower n: i, j, x, y, w c \r\n"));		
		for(j = 0; j < strat_infos.num_towers; j++) {
			printf(PSTR(" tower %d: %d, %d, %.4d, %.4d, %.3d %.2d"), j+1,
					 strat_infos.towers[j].i, strat_infos.towers[j].j,
					 strat_infos.towers[j].x, strat_infos.towers[j].y,
					 strat_infos.towers[j].w, strat_infos.towers[j].c);
			printf(PSTR(" \r\n"));
		}		
	}
}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
	uint8_t i, j;

	/* bounding box */
	strat_set_bounding_box();
	
	/* reset flags of slots */
	for(i=0; i<NB_SLOT_X; i++) {
		for(j=0; j<NB_SLOT_Y; j++) {
			strat_infos.slot[i][j].flags = 0;
		}
	}

	/* specific flags */
	strat_infos.slot[1][5].flags = SLOT_SAFE;
	strat_infos.slot[2][5].flags = SLOT_SAFE;
	strat_infos.slot[5][5].flags = SLOT_SAFE;
	strat_infos.slot[6][5].flags = SLOT_SAFE;

	/* tokens catched */
	strat_infos.num_tokens = 0;

	/* slot position */
	strat_infos.slot_before = strat_infos.slot_actual;

	/* thresholds */
	strat_infos.conf.th_place_prio = SLOT_PRIO_NEAR_GREEN;
	strat_infos.conf.th_token_score = PION_SCORE;

	/* towers found */
	strat_infos.num_towers = 0;
	memset(&strat_infos.towers, 0, sizeof(strat_infos.towers));	
}

/* call it just before launching the strat */
void strat_init(void)
{
	strat_reset_infos();

	/* we consider that the color is correctly set */

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	time_reset();
	interrupt_traj_reset();

	/* used in strat_base for END_TIMER */
	mainboard.flags = DO_ENCODERS | DO_CS | DO_RS | 
		DO_POS | DO_BD | DO_TIMER | DO_POWER | DO_OPP;
}


/* call it after each strat */
void strat_exit(void)
{
	uint8_t flags;

	/* stop robot, disable timer */
	mainboard.flags &= ~(DO_TIMER);
	strat_hardstop();
	time_reset();

	/* disable CS, and motors */
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_CS);
	dac_mc_set(LEFT_MOTOR, 0);
	dac_mc_set(RIGHT_MOTOR, 0);
	IRQ_UNLOCK(flags);

	/* stop beacon */
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_OPP);
	IRQ_UNLOCK(flags);
	beacon_cmd_beacon_off();

	/* power off lasers */
	lasers_set_off();

	/* stop slavespic */
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);

	/* stop beacon */
	beacon_cmd_beacon_off();
	beacon_cmd_beacon_off();
	beacon_cmd_beacon_off();
	beacon_cmd_beacon_off();

}

/* called periodically */
void strat_event(void *dummy)
{
	/* XXX in parallel with main strat, 
	 *	disable/enable events depends on case.
	 */

	/* limit speed when opponent or tokens(TODO) are close */
	//strat_limit_speed();

	/* update actual slot position */
	strat_update_slot_position();	

	/* TODO: catch tokens in straight travels */

	/* TODO: update num of token catched */

	/* TODO: check figures of green zone */

	/* manage mirrors position */
	mirrors_state_machine();
	
}

/* dump state (every 5 s max) XXX */
#define DUMP_RATE_LIMIT(dump, last_print)		\
	do {						\
		if (time_get_s() - last_print > 5) {	\
			dump();				\
			last_print = time_get_s();	\
		}					\
	} while (0)


#define ERROUT(e) do {\
		err = e;			 \
	} while(0)	


/* begining trajs related with static elements */
uint8_t strat_beginning(void)
{
	uint8_t err = 0;
	uint16_t old_spdd, old_spda;

	/* set new speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* go out of start position */
	wait_until_opponent_is_far();
	trajectory_d_rel(&mainboard.traj, 200);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick & place tokens on line 1 */
	err = strat_harvest_line1();
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

#ifndef HOMOLOGATION
	/* pick & place tokens on line 2 */
	err = strat_harvest_line2();
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick & place tokens on green area */
	err = strat_harvest_green_area();
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
#endif

// end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* strat main loop */
uint8_t strat_main(void)
{
	uint8_t err; //i, why=0;

	/* pick & place our static tokens */
	err = strat_beginning();

#ifndef HOMOLOGATION
	/* place two token on the other side */
	err = strat_bonus_point();
#endif

#ifdef TEST_EXIT
	i2c_slavedspic_mode_token_take(SIDE_FRONT);
	i2c_slavedspic_mode_token_take(SIDE_REAR);
	lasers_set_on();
#endif

	/* autoplay */
	while (1) {

		err = wait_traj_end(TRAJ_FLAGS_STD);

		/* check end of match */
		if (err == END_TIMER) {
			DEBUG(E_USER_STRAT, "End of time");
			strat_exit();
			break;
		}

	}
	return END_TRAJ;
}

#endif /* HOST_VERSION */


