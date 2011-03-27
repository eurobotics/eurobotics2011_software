/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2010)
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

#ifndef _STRAT_H_
#define _STRAT_H_

/* area */
#define AREA_X 3000
#define AREA_Y 2100

/* convert coords according to our color */
#define COLOR_Y(y)     (y)
#define COLOR_X(x)     ((mainboard.our_color==I2C_COLOR_BLUE)? (x) : (AREA_X-(x)))

#define COLOR_A_REL(a) ((mainboard.our_color==I2C_COLOR_BLUE)? (a) : (-a))
#define COLOR_A_ABS(a) ((mainboard.our_color==I2C_COLOR_BLUE)? (a) : (180-a))

#define COLOR_SIGN(x)  ((mainboard.our_color==I2C_COLOR_BLUE)? (x) : (-x))
#define COLOR_INVERT(x)((mainboard.our_color==I2C_COLOR_BLUE)? (x) : (!x))

#define START_X 200
#define START_Y COLOR_Y(200)
#define START_A COLOR_A(45)

#define CENTER_X 1500
#define CENTER_Y 1050

#define SIDE_REAR		I2C_SIDE_REAR
#define SIDE_FRONT 	I2C_SIDE_FRONT 
#define SIDE_MAX		I2C_SIDE_MAX

#define GO_FORWARD	0
#define GO_BACKWARD	1

#define TOKEN_DIAMETER	200

/* useful traj flags */
#define TRAJ_SUCCESS(f) 				(f & (END_TRAJ|END_NEAR))

#define TRAJ_FLAGS_STD 					(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER 			(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR 			(END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_INTR)

/* default speeds */
#ifdef HOMOLOGATION
#define SPEED_DIST_FAST 		1000
#define SPEED_ANGLE_FAST 		1000
#else
#define SPEED_DIST_FAST 		4000
#define SPEED_ANGLE_FAST 		4000
#endif
#define SPEED_DIST_SLOW 		2000
#define SPEED_ANGLE_SLOW 		2000
#define SPEED_DIST_VERY_SLOW 	1000
#define SPEED_ANGLE_VERY_SLOW 1000

/************************************************************* 
 * Strat data structures 
 ************************************************************/

/* boulding box */
struct bbox {
	int32_t x1;
	int32_t y1;
	int32_t x2;
	int32_t y2;
};

/* configuration */
struct conf {

/* depends on flags the robot
 * do one things or anothers */

	uint8_t flags;
/* example : */
#define STRAT_CONF_A		 	0x01
#define STRAT_CONF_B			0x02
#define STRAT_CONF_C 		0x04
};

/* slot of token */
struct slot_info {
	int16_t x;
	int16_t y;

	uint8_t color;
#define SLOT_BLUE			I2C_COLOR_BLUE
#define SLOT_RED			I2C_COLOR_RED
#define SLOT_GREEN_AREA	I2C_COLOR_MAX
	
	uint8_t prio;
#define SLOT_PRIO_0			0
#define SLOT_PRIO_1			1
#define SLOT_PRIO_2			2
#define SLOT_PRIO_3			3
#define SLOT_PRIO_WALL		4
#define SLOT_PRIO_BONUS		5
#define SLOT_PRIO_SAFE		6

	uint16_t flags;
#define SLOT_BONUS				0x01
#define SLOT_SAFE					0x02
#define SLOT_WALL					0x04
#define SLOT_CHECK				0x08
#define SLOT_CHECK_ONESIDE		0x16
#define SLOT_BUSY					0x32
#define SLOT_NEAR_GREEN_RED	0x64
#define SLOT_NEAR_GREEN_BLUE	0x128

};



/* infos about strat */
#define NB_SLOT_X				6
#define NB_SLOT_Y				6
#define NB_SLOT_GREEN		5
#define NB_GRID_LINES_X 	9
#define NB_GRID_LINES_Y 	7


struct strat_infos {
	uint8_t dump_enabled;
	struct conf conf;
	struct bbox area_bbox;

	/* playing areas */
	struct slot_info slot_grid[NB_SLOT_X][NB_SLOT_Y];
	struct slot_info slot_green[I2C_COLOR_MAX][NB_SLOT_GREEN];

	/* grid lines */
	uint16_t grid_line_x[NB_GRID_LINES_X];
	uint16_t grid_line_y[NB_GRID_LINES_Y];

	/* slot position */
	uint8_t slot_actual;
	uint8_t slot_next;
	uint8_t slot_before;
};

extern struct strat_infos strat_infos;


/************************************************************* 
 * Functions headers of strat files
 ************************************************************/

/* in strat.c */
void strat_dump_infos(const char *caller);
void strat_dump_conf(void);
void strat_reset_infos(void);

void strat_preinit(void);
void strat_init(void);
void strat_exit(void);

uint8_t strat_main(void);
uint8_t strat_beginning(void);
void strat_event(void *dummy);

/* in strat_tokens.c */
uint8_t strat_pickup_token(int16_t x, int16_t y, uint8_t side);
uint8_t strat_pickup_token_auto(int16_t x, int16_t y);
uint8_t strat_place_token(int16_t x, int16_t y, uint8_t side, uint8_t go);
uint8_t strat_place_token_auto(int16_t x, int16_t y, uint8_t go);

/* in strat_static.c */
uint8_t strat_harvest_line1(void);
uint8_t strat_harvest_line2(void);
uint8_t strat_harvest_green_area(void);

/* add here more strat functions in files */

#endif
