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

#define OPPOSITE_SIDE(side) ((side==I2C_SIDE_FRONT)? (I2C_SIDE_REAR) : (I2C_SIDE_FRONT))	

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

/* homologation compilation */
//#define HOMOLOGATION

/* default speeds */
#ifdef HOMOLOGATION
#define SPEED_DIST_FAST 		2000
#define SPEED_ANGLE_FAST 		2000
#else
#define SPEED_DIST_FAST 		4000
#define SPEED_ANGLE_FAST 		4000
#endif
#define SPEED_DIST_SLOW 		1000
#define SPEED_ANGLE_SLOW 		1000
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
#define LINE1_CONF_2TOKENS_ON_BONUS		1
#define LINE1_CONF_2TOKENS_NEAR_WALL	2
#define LINE1_CONF_OPP_TOKEN_FIRST		4
#define LINE1_CONF_OPP_TOKEN_LAST		8
#define STRAT_CONF_PLACE_ONLYEXT		  16

	/* thresholds */
	uint8_t th_place_prio;
	uint8_t th_token_score;

};

/* token scores */
#define NULL_SCORE		0
#define PION_SCORE		10
#define FIGURE_SCORE		20
#define TOWER1H_SCORE	40
#define TOWER2H_SCORE	60


/* slot dimensions */
#define SLOT_SIZE 		350
#define SLOT_SIZE_HALF	175
#define SLOT_DIAGONAL	495
#define SLOT_GREEN_SIZE 280

/* slot of token */
struct slot_info {
	int16_t x;
	int16_t y;

	uint8_t color;
#define SLOT_BLUE			I2C_COLOR_BLUE
#define SLOT_RED			I2C_COLOR_RED
#define SLOT_GREEN_BLUE	I2C_COLOR_MAX
#define SLOT_GREEN_RED	(I2C_COLOR_MAX+1)

	
	uint8_t prio;
#define SLOT_PRIO_0		0
#define SLOT_PRIO_1		10
#define SLOT_PRIO_2		20
#define SLOT_PRIO_3		30
#define SLOT_PRIO_4		40
#define SLOT_PRIO_5		50
#define SLOT_PRIO_6		60
#define SLOT_PRIO_7		70
#define SLOT_PRIO_MAX	80
#define SLOT_PRIOR_INC	10

/* strat areas priorities */
#define SLOT_PRIO_GREEN			SLOT_PRIO_0	
#define SLOT_PRIO_CENTER		SLOT_PRIO_1
#define SLOT_PRIO_PATH			SLOT_PRIO_2
#define SLOT_PRIO_NEAR_GREEN	SLOT_PRIO_3
#define SLOT_PRIO_NEAR_SAFE	SLOT_PRIO_4
#define SLOT_PRIO_WALL			SLOT_PRIO_5
#define SLOT_PRIO_SAFE			SLOT_PRIO_6
#define SLOT_PRIO_BONUS_WALL	SLOT_PRIO_7
#define SLOT_PRIO_BONUS			SLOT_PRIO_MAX


	uint16_t flags;
#define SLOT_BONUS			1
#define SLOT_SAFE				2
#define SLOT_AVOID			4
#define SLOT_CHECKED			8
#define SLOT_BUSY				16
#define SLOT_VISITED			32
#define SLOT_OPPONENT		64
#define SLOT_ROBOT			128
#define SLOT_FIGURE			256

	uint8_t flags_poly;
#define SLOT_POLY_SQUARE	1

};

typedef struct {
	int8_t i;
	int8_t j;
}slot_index_t;

/* infos about strat */
#define NB_SLOT_X				8
#define NB_SLOT_Y				6
#define NB_SLOT_GREEN		5
#define NB_GRID_LINES_X 	9
#define NB_GRID_LINES_Y 	7


#define NB_TOWER_MAX	6
typedef struct {
	int8_t i;
	int8_t j;
	int16_t x;
	int16_t y;
	int16_t w;
	int8_t c;		
} tower_t;

struct strat_infos {
	uint8_t dump_enabled;
	struct conf conf;
	struct bbox area_bbox;

	/* playing areas */
	struct slot_info slot[NB_SLOT_X][NB_SLOT_Y];

	/* grid lines */
	uint16_t grid_line_x[NB_GRID_LINES_X];
	uint16_t grid_line_y[NB_GRID_LINES_Y];

	/* our slot position */
	slot_index_t slot_actual;
	slot_index_t slot_before;

	/* tokens catched */
	uint8_t num_tokens;

	/* towers found */
	uint8_t num_towers;
	tower_t towers[NB_TOWER_MAX];

	/* TODO opponent stadistics */

	/* TODO working zones */

};

extern struct strat_infos strat_infos;


#ifndef HOST_VERSION

/************************************************************* 
 * Functions headers of strat files
 ************************************************************/

/********************************************
 * in strat.c 
 *******************************************/
void strat_dump_infos(const char *caller);
void strat_dump_conf(void);
void strat_reset_infos(void);

void strat_preinit(void);
void strat_init(void);
void strat_exit(void);

uint8_t strat_main(void);
uint8_t strat_beginning(void);
void strat_event(void *dummy);


/*********************************************
 * in strat_tokens.c 
 ********************************************/

/* pick up a token */
/* use it in short distance ranges */
uint8_t strat_pickup_token(int16_t x, int16_t y, uint8_t side);

/* pick up token chossing side automaticaly */
/* we suppose that at least one side is empty */
uint8_t strat_pickup_token_auto(int16_t x, int16_t y, uint8_t *side);

/* place a token */	
/* use it in near range distance */
uint8_t strat_place_token(int16_t x, int16_t y, uint8_t side, uint8_t go);

/* place token automaticaly */
/* we suppose that there is at least one token catched */
uint8_t strat_place_token_auto(int16_t x, int16_t y, uint8_t *side, uint8_t go);

/* push a token in order to get out an opponent token from slot */
uint8_t strat_push_slot_token(int8_t i, int8_t j);

/* pickup near slots on an area 3x3 with center the robot */
uint8_t strat_pickup_near_slots(void);

/* place tokens on near 3x3 area slots, return 0 if there aren't more slots */
uint8_t strat_place_near_slots(void);

/* pickup and place near tokens in 3x3 area where is the robot */
uint8_t strat_pickup_and_place_near_slots(void);

/* enable/disable look for towers */
void strat_look_for_towers_enable(void);
void strat_look_for_towers_disable(void);

/* look for tower of 2 or 3 levels, if find any tower, it's added to infos */
void strat_look_for_towers(void);

/* return 1 if a new tower is added succesfully */ 
uint8_t strat_info_add_tower(int16_t x, int16_t y, int16_t w);

/* return 1 if an exist tower is deleted succesfully */
uint8_t strat_info_del_tower(int8_t i, int8_t j);

/* enable/disable look for figures */
void strat_look_for_figures_enable(void);
void strat_look_for_figures_disable(void);

/* try to find figures from line 1 */
void strat_look_for_figures(void);


/**************************************************
 * in strat_static.c 
 *************************************************/
uint8_t strat_harvest_line1(void);
uint8_t strat_harvest_line2(void);
uint8_t strat_harvest_green_area(void);

/* harvest green area ending with two figures inside */
uint8_t strat_harvest_green_area_smart(void);

/**************************************************
 * in strat_navigation.c 
 *************************************************/
void strat_update_slot_position(void);
uint8_t strat_bonus_point(void);



/* add here more strat functions in files */

#else

void strat_set_bounding_box(void);

#endif /* HOST_VERSION */

#endif
