/*  
 *  Copyright Droids Corporation, Microb Technology (2009)
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
 *  Revision : $Id: strat_utils.h,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Bali�as Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  strat_utils.h,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */

#ifndef _STRAT_UTILS_H_
#define _STRAT_UTILS_H_


#define DEG(x) (((double)(x)) * (180.0 / M_PI))
#define RAD(x) (((double)(x)) * (M_PI / 180.0))
#define M_2PI (2*M_PI)

struct xy_point {
	int16_t x;
	int16_t y;
};

/* wait traj end flag or cond. return 0 if cond become true, else
 * return the traj flag */
#define WAIT_COND_OR_TRAJ_END(cond, mask)				\
	({								\
		uint8_t __err = 0;					\
		while ( (! (cond)) && (__err == 0)) {			\
			__err = test_traj_end(mask);	\
		}							\
		__err;							\
	})								\

int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
int16_t distance_from_robot(int16_t x, int16_t y);

int16_t simple_modulo_360(int16_t a);
double simple_modulo_2pi(double a);

int16_t angle_abs_to_rel(int16_t a_abs);

void rel_da_to_abs_xy(double d_rel, double a_rel_rad, double *x_abs, double *y_abs);

double norm(double x, double y);

void rel_xy_to_abs_xy(double x_rel, double y_rel, double *x_abs, double *y_abs);
void abs_xy_to_rel_da(double x_abs, double y_abs, double *d_rel, double *a_rel_rad);

void rotate(double *x, double *y, double rot);

uint8_t is_in_area(int16_t x, int16_t y, int16_t margin);
uint8_t point_is_in_area(int16_t px, int16_t py,
								 int16_t x_up, int16_t y_up,
								 int16_t x_down, int16_t y_down);
uint8_t robot_is_in_area(int16_t margin);

uint8_t y_is_more_than(int16_t y);
uint8_t x_is_more_than(int16_t x);

uint8_t opp_x_is_more_than(int16_t x);
uint8_t opp_y_is_more_than(int16_t y);

int16_t fast_sin(int16_t deg);
int16_t fast_cos(int16_t deg);

uint8_t get_color(void);
uint8_t get_opponent_color(void);

int8_t get_opponent_xy(int16_t *x, int16_t *y);
int8_t get_opponent_da(int16_t *d, int16_t *a);
int8_t get_opponent_xyda(int16_t *x, int16_t *y, int16_t *d, int16_t *a);

uint8_t opponent_is_behind(void);
uint8_t opponent_is_infront(void);
uint8_t opponent_is_behind_side(uint8_t side);
uint8_t opponent_is_infront_side(uint8_t side);
uint8_t opponent_is_in_area(int16_t x_up, int16_t y_up,
									 int16_t x_down, int16_t y_down);
uint8_t opponent_is_in_slot(int8_t i, int8_t j);
uint8_t opponent_is_in_near_slots(void);
uint8_t opponent_is_near_to_slot(int8_t i, int8_t j);
uint8_t opponent_is_near_to_target_slot(int8_t i, int8_t j);


uint8_t token_catched(uint8_t side);
uint8_t token_inside(uint8_t side);
uint8_t belts_blocked(uint8_t side);

/* return the score of a token on side */
uint8_t token_side_score(uint8_t side);

/* return 1 if there is a token on side and has the lower priority */
uint8_t token_side_is_lower_score(uint8_t side);


///* goto with the empty side, prepared to catch token, return goto side */
//uint8_t strat_goto_empty_side_xy_abs(struct trajectory *traj, double x_abs_mm, double y_abs_mm);

///* goto with the empty side and with belts in mode take, return goto side */
//uint8_t strat_goto_harvesting_xy_abs(struct trajectory *traj, double x_abs_mm, double y_abs_mm);

/* turn to pickup token, return side in front of token */
/* suppose that there is at least one side empty */
uint8_t strat_turnto_pickup_token(struct trajectory *traj, double x_abs_mm, double y_abs_mm);

/* turn to place token automaticaly, return side used to place token */
/* suppose that there is at least one token catched */
uint8_t strat_turnto_place_token(struct trajectory *traj, double x_abs_mm, double y_abs_mm, uint8_t go);

/* go straight forward with no side dependence (d is in mm) */
void strat_d_rel_side(struct trajectory *traj, double d_mm, uint8_t side);

/* return 1 if the opponent is near */
/* only compile with HOMOLOGATION define */
void wait_until_opponent_is_far(void);


/* apply flags to slot */
void strat_set_slot_flags(int16_t x, int16_t y, uint16_t flags);
void strat_clear_slot_flags(int16_t x, int16_t y, uint16_t flags);


/* get index (i,j) of slot from (x,y) coordinates */
void get_slot_index(int16_t x, int16_t y, int8_t *i, int8_t *j);

#endif
