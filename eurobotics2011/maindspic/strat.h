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
 *  Javier Baliñas Santos <javier@arc-robots.org>
 */

#ifndef _STRAT_H_
#define _STRAT_H_

/* area */
#define AREA_X 3000
#define AREA_Y 2100

/* convert coords according to our color */
#define COLOR_Y(y)     ((mainboard.our_color==I2C_COLOR_BLUE)? (y) : (AREA_Y-(y)))
#define COLOR_X(x)     (x)

#define COLOR_A_REL(a) ((mainboard.our_color==I2C_COLOR_BLUE)? (a) : (-a))
#define COLOR_A_ABS(a) ((mainboard.our_color==I2C_COLOR_BLUE)? (a) : (180-a))

#define COLOR_SIGN(x)  ((mainboard.our_color==I2C_COLOR_BLUE)? (x) : (-x))
#define COLOR_INVERT(x)((mainboard.our_color==I2C_COLOR_BLUE)? (x) : (!x))

#define START_X 200
#define START_Y COLOR_Y(200)
#define START_A COLOR_A(45)

#define CENTER_X 1500
#define CENTER_Y 1050

#define CORNER_X 3000
#define CORNER_Y COLOR_Y(2100)

/* grid */
#define GRID_X(num)	(150 + 225*num)
#define GRID_Y(num)	(2100-(128 + 125*(11-num)))

/* useful traj flags */
#define TRAJ_SUCCESS(f) 		(f & (END_TRAJ|END_NEAR))

#define TRAJ_FLAGS_STD 					(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER 			(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR 			(END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_INTR)

/* default speeds */
#define SPEED_DIST_FAST 		2000
#define SPEED_ANGLE_FAST 		2000
#define SPEED_DIST_SLOW 		1500
#define SPEED_ANGLE_SLOW 		1500
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
#define STRAT_CONF_A		 	0x01
#define STRAT_CONF_B			0x02
#define STRAT_CONF_C 		0x04
};

/* infos */
struct strat_infos {
	uint8_t dump_enabled;
	struct conf conf;
	struct bbox area_bbox;
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
void strat_event(void *dummy);

/* add here more strat files */

#endif
