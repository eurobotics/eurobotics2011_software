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
#include <string.h>
#include <math.h>

#include <aversive.h>
#include <aversive\pgmspace.h>
#include <aversive\wait.h>
#include <aversive\error.h>

#include <uart.h>
#include <i2c.h>
#include <parse.h>
#include <rdline.h>
#include <pwm_mc.h>
#include <encoders_dspic.h>
#include <timer.h>
#include <scheduler.h>
#include <pid.h>
#include <time.h>
#include <quadramp.h>
#include <control_system_manager.h>

#include <blocking_detection_manager.h>

#include "sensor.h"

#include "../common/i2c_commands.h"
#include "main.h"
#include "beacon.h"

/* some conversions and constants */
#define DEG(x) (((double)(x)) * (180.0 / M_PI))
#define RAD(x) (((double)(x)) * (M_PI / 180.0))
#define M_2PI (2*M_PI)

/* field area */
#define AREA_X 3000
#define AREA_Y 2100

/* convert coords according to our color */
#define COLOR_X(x)     ((beaconboard.our_color==I2C_COLOR_RED)? (x) : (AREA_X-(x)))
#define COLOR_Y(y)     ((beaconboard.our_color==I2C_COLOR_RED)? (y) : (AREA_Y-(y)))

/* fixed beacon coordenates */
#define BEACON_X_OFFSET	(-62)				// beacon calibration includes the offset of 6.2cm
#define BEACON_Y_OFFSET	(AREA_Y/2)
#define BEACON_A_OFFSET	(0)

/* IR sensors pin read value */
#define IR_SENSOR_0_DEG_PIN() 	(!(_RC4))
#define IR_SENSOR_180_DEG_PIN() 	(!(_RC5))

/* IR sensor management */
#define IR_SENSOR_0_DEG		0
#define IR_SENSOR_180_DEG	1
#define IR_SENSOR_MAX		2

#define EDGE_RISING		0
#define EDGE_FALLING 	1
#define EDGE_MAX 			2

/* modulo of timer base */
#define MODULO_TIMER (65535L)

/* to work in units of mili degrees */
#define MULT_ANGLE (1000L)

/* debug macros */
#define BEACON_DEBUG(args...) DEBUG(E_USER_BEACON, args)
#define BEACON_NOTICE(args...) NOTICE(E_USER_BEACON, args)
#define BEACON_ERROR(args...) ERROR(E_USER_BEACON, args)

/* beacon calculations funcions */
static int32_t get_dist(int32_t size, int32_t period);
static int32_t get_angle(int32_t middle, int32_t period, int32_t offset);

/* data structure to store beacon results */
struct beacon beacon;

/* turn period measure */
static volatile int32_t count_period = 0;		/* counts of timer asociated */
static volatile int32_t count_period_ov = 0;	/* overflow flag of timer    */
static volatile int8_t valid_period = 0;		/* new valid measure is available */

/* IR sensors pulse measure */
static volatile int32_t 
count_edge[IR_SENSOR_MAX][EDGE_MAX] = {{0, 0}, {0 ,0}}; 		/* counts of timer */		
static volatile int32_t 
count_edge_ov[IR_SENSOR_MAX][EDGE_MAX] = {{0, 0}, {0 ,0}};	/* overflow flag   */
static volatile int8_t 
valid_pulse[IR_SENSOR_MAX] = {0, 0};	/* new valid measures available */								
static int32_t invalid_count = 0;		/* timeout of pulse measure */


/* initialize beacon */
void beacon_init(void)
{
	/* clear data structures */
	memset(&beacon, 0, sizeof(struct beacon));
	beacon.opponent_x = I2C_OPPONENT_NOT_THERE;
	
	/* default values */		

	beaconboard.our_color = I2C_COLOR_RED;


	/* HARDWARE INIT */

	/* XXX: all measures are syncronized with then Timer 2 */

	/* initialize input capture (IC) 1 and 2 for IR sensors events */
	IC1CONbits.ICM =0b000;		// disable Input Capture module
	IC1CONbits.ICTMR = 1; 		// select Timer2 as the IC time base
	IC1CONbits.ICI = 0b00; 		// interrupt on every capture event
	IC1CONbits.ICM = 0b001; 	// generate capture event on every edge
	
	IC2CONbits.ICM =0b00; 		// disable Input Capture module
	IC2CONbits.ICTMR = 1; 		// select Timer2 as the IC time base
	IC2CONbits.ICI = 0b00; 		// interrupt on every capture event
	IC2CONbits.ICM = 0b001; 	// generate capture event on every edge

	/* initialize input capture 7 and 8 for turn sensors */
	IC7CONbits.ICM =0b000;		// disable Input Capture module
	IC7CONbits.ICTMR = 1; 		// select Timer2 as the IC1 time base
	IC7CONbits.ICI = 0b00; 		// interrupt on every capture event
	IC7CONbits.ICM = 0b011; 	// generate capture event on every rising edge

	IC8CONbits.ICM =0b000;		// disable Input Capture module
	IC8CONbits.ICTMR = 1; 		// select Timer2 as the IC1 time base
	IC8CONbits.ICI = 0b00; 		// interrupt on every capture event
	IC8CONbits.ICM = 0b011; 	// generate capture event on every rising edge

	/* enable all inputs capture interrupts and Timer 2 */
	IPC0bits.IC1IP = 5; 	// setup IC1 interrupt priority level XXX, higher than scheduler!
	IFS0bits.IC1IF = 0; 	// clear IC1 Interrupt Status Flag
	IEC0bits.IC1IE = 1; 	// enable IC1 interrupt

	IPC1bits.IC2IP = 5; 	// setup IC2 interrupt priority level XXX, higher than scheduler!
	IFS0bits.IC2IF = 0; 	// clear IC2 Interrupt Status Flag
	IEC0bits.IC2IE = 1; 	// enable IC2 interrupt

	IPC5bits.IC7IP = 5; 	// setup IC1 interrupt priority level XXX, higher than scheduler!
	IFS1bits.IC7IF = 0; 	// clear IC1 Interrupt Status Flag
	IEC1bits.IC7IE = 1; 	// enable IC1 interrupt

	/* TODO: for the moment beacon only uses one turn sensor */
/*	IPC5bits.IC8IP = 5; 	// setup IC1 interrupt priority level XXX, higher than scheduler!
	IFS1bits.IC8IF = 0; 	// clear IC1 Interrupt Status Flag
	IEC1bits.IC8IE = 1; 	// enable IC1 interrupt
*/

	/* config and enable Timer 2 */
	PR2 = 65535;
	IFS0bits.T2IF 	 = 0; 	// clear T2 Interrupt Status Flag

	T2CONbits.TCKPS = 0b10;	// Timer 2 prescaler = 256, T_timer2 = 1.6us (0b11 for 6.4 us)
	T2CONbits.TON	 = 1;		// enable Timer 2


	/* CS EVENT */
	scheduler_add_periodical_event_priority(beacon_calc, NULL, 
						EVENT_PERIOD_BEACON / SCHEDULER_UNIT, EVENT_PRIO_BEACON);

}

/* input compare 1 interrupt connected to IR_SENSOR_0_DEG */
void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{
	uint8_t flags;
	
	/* reset flag */
	_IC1IF=0;
	
	/* Capture of Timer 2 counts on falling and risign edge of IR sensor.
    * After falling edge set valid_pulse.
	 */

	/* NOTE: Timer 2 count is hardware buffered by Input capture so,
	 *       we don't lose counts.
	 */

	/* rising edge */
	if ( IR_SENSOR_0_DEG_PIN()) {
		IRQ_LOCK(flags);
		count_edge[IR_SENSOR_0_DEG][EDGE_RISING] = (int32_t)IC1BUF;
		count_edge_ov[IR_SENSOR_0_DEG][EDGE_RISING] = _T2IF;
		valid_pulse[IR_SENSOR_0_DEG] = 0;
		IRQ_UNLOCK(flags);

	}
	/* falling edge */
	else {
		IRQ_LOCK(flags);
		count_edge[IR_SENSOR_0_DEG][EDGE_FALLING] = (int32_t)IC1BUF;
		count_edge_ov[IR_SENSOR_0_DEG][EDGE_FALLING] = _T2IF;
		valid_pulse[IR_SENSOR_0_DEG] = 1;
		IRQ_UNLOCK(flags);
	}
	
}

/* input compare 2 interrupt connected to IR_SENSOR_180_DEG */
void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{
	uint8_t flags;
	
	/* reset flag */
	_IC2IF=0;


	/* Capture of Timer 2 counts on falling and risign edge of IR sensor.
    * After falling edge set valid_pulse.
	 */

	/* NOTE: Timer 2 count is hardware buffered by Input capture so,
	 *       we don't lose counts.
	 */

	/* rising edge */
	if ( IR_SENSOR_180_DEG_PIN()) {
		IRQ_LOCK(flags);
		count_edge[IR_SENSOR_180_DEG][EDGE_RISING] = (int32_t)IC2BUF;
		count_edge_ov[IR_SENSOR_180_DEG][EDGE_RISING] = _T2IF;
		valid_pulse[IR_SENSOR_180_DEG] = 0;
		IRQ_UNLOCK(flags);

	}
	/* falling edge */
	else {
		IRQ_LOCK(flags);
		count_edge[IR_SENSOR_180_DEG][EDGE_FALLING] = (int32_t)IC2BUF;
		count_edge_ov[IR_SENSOR_180_DEG][EDGE_FALLING] = _T2IF;
		valid_pulse[IR_SENSOR_180_DEG] = 1;
		IRQ_UNLOCK(flags);
	}
}

/* input compare 7 interrupt connected to turn sensor aligned with IR_SENSOR_0_DEG */
void __attribute__((__interrupt__, no_auto_psv)) _IC7Interrupt(void)
{
	uint8_t flags;
	
	/* reset flag */
	_IC7IF=0;
	
	
	/* Capture of Timer 2 counts on risign edge of turn sensor.
	 */

	/* block interrupt */
	IRQ_LOCK(flags);
	
	/* reset timer */
	TMR2 = 0;

	/* XXX: there is a delay between the instant of capture
    * and timer reset, this involve an offset error on angle measure.
    */

	/* save bufferd counts and overflow flag */
	count_period = (int32_t)IC7BUF;
	count_period_ov = _T2IF;

	/* reset overflow flag */
	_T2IF = 0;	
	
	/* set valid_period */
	valid_period = 1;

	/* unblock interrupt */
	IRQ_UNLOCK(flags);
}

/* TODO: input compare 7 interrupt connected to turn sensor aligned with IR_SENSOR_180_DEG */
/*
void __attribute__((__interrupt__, no_auto_psv)) _IC8Interrupt(void)
{
	uint8_t flags;
	
	IFS1bits.IC8IF=0;
	
	IRQ_LOCK(flags);
	TMR2 = 0;
	count_period[1] = (int32_t)IC8BUF;
	valid_period[1] = 1;
	IRQ_UNLOCK(flags);

}
*/

/* start turn and measures */
void beacon_start(void)
{
	/* enable beacon_calc event flag */
	beaconboard.flags |= DO_BEACON;

	/* enable cs */
	beacon_reset_pos();
	beaconboard.speed.on = 1;
	cs_set_consign(&beaconboard.speed.cs, 80/4);
}

/* stop turn and measures */
void beacon_stop(void)
{
	/* disable beacon_calc event flag */
	beaconboard.flags &= ~(DO_BEACON);

	/* disable cs */
	beaconboard.speed.on = 0;
	cs_set_consign(&beaconboard.speed.cs, 0);
	pwm_mc_set(BEACON_PWM, 0);
}

/**********************************************************************
 * HELPERS FOR BEACON CALCULUS
 *********************************************************************/

/* return the distance between two points */
int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int32_t x,y;
	x = (x2-x1);
	x = x*x;
	y = (y2-y1);
	y = y*y;
	return sqrt(x+y);
}

/* return the normal of a vector with origin (0,0) */
double norm(double x, double y)
{
	return sqrt(x*x + y*y);
}

/* calculate the distance and angle (between -180 and 180) of a beacon
 * opponent coordenates relative to robot coordinates  						*/
void abs_xy_to_rel_da(double x_robot, double y_robot, double a_robot, 
							 double x_abs, double y_abs,
		      			 int32_t *d_rel, int32_t *a_rel_deg)
{
	double a_rel_rad;

	a_rel_rad = atan2(y_abs - y_robot, x_abs - x_robot) - RAD(a_robot);

	if (a_rel_rad < -M_PI) {
		a_rel_rad += M_2PI;
	}
	else if (a_rel_rad > M_PI) {
		a_rel_rad -= M_2PI;
	}
	
	*a_rel_deg = (int32_t)(DEG(a_rel_rad));
	*d_rel = (int32_t)(norm(x_abs-x_robot, y_abs-y_robot));
}


/* return true if the point (x,y) is in area defined by margin */
uint8_t is_in_margin(int16_t dx, int16_t dy, int16_t margin)
{
	if ((ABS(dx) < margin) && (ABS(dy) < margin))
		return 1;
	return 0;
}


/* calculate distance from size of a pulse width and turn period */
static int32_t get_dist(int32_t size, int32_t period)
{
	int32_t dist=0;
	double size_rel;
	
	/* calcule relative angle */
	size_rel = size*1.0/period;

	/* dist = offset + (a0 + a1*x + a2*x� + a3x�) */	
	dist =  (int32_t)((0.0062 + (-0.1546*size_rel) + 
							(1.1832*size_rel*size_rel) + 
							(-2.4025*size_rel*size_rel*size_rel))*100000);
	
	/* practical offset */
	dist += 16;      
 
	return dist;
}

/* calculate angle from middle of pulse width, turn period and angle offset */
static int32_t get_angle(int32_t middle, int32_t period, int32_t offset)
{
	int32_t ret_angle;
	
	ret_angle = (int32_t)(middle * 360.0 * MULT_ANGLE / period);
	ret_angle = (ret_angle + offset*MULT_ANGLE)%(360*MULT_ANGLE);
	
	return ret_angle;
}

/* calculate absolute (x,y) coordinates from angle and distance measures */
void beacon_angle_dist_to_x_y(int32_t angle, int32_t dist, int32_t *x, int32_t *y)
{
//	uint8_t flags;

	int32_t local_x = 0;
	int32_t local_y = 0;
	int32_t x_opponent;
	int32_t y_opponent;
	int32_t local_robot_angle = 0;

//	IRQ_LOCK(flags);
//	local_x           = beacon.robot_x;
//	local_y           = beacon.robot_y;
//	local_robot_angle = beacon.robot_angle;
//	IRQ_UNLOCK(flags);

	if (local_robot_angle < 0)
		local_robot_angle += 360;

	x_opponent = cos((local_robot_angle + angle)* 2 * M_PI / 360)* dist;
	y_opponent = sin((local_robot_angle + angle)* 2 * M_PI / 360)* dist;

//	BEACON_DEBUG("x_op= %ld\t",x_opponent);
//	BEACON_DEBUG("y_op= %ld\r\n",y_opponent);
//	BEACON_NOTICE("robot_x= %ld\t",local_x);
//	BEACON_NOTICE("robot_y= %ld\t",local_y);
//	BEACON_NOTICE("robot_angle= %ld\r\n",local_robot_angle);

	*x = local_x + x_opponent;
	*y = local_y + y_opponent;

}


/**********************************************************************
 * BEACON CALCULUS
 *********************************************************************/

/* calculate distance (d) and angle (a) and (x,y) possition of the 
 * opponent beacon relative to the robot coordinates.
 */
void sensor_calc(uint8_t sensor)
{
	/* TODO: eliminate magicnumbers */

	static int32_t local_count_period, local_count_period_filtered=0;
	static int32_t local_count_period_ov;
	static int8_t  local_valid_period;

	static int32_t local_count_edge[EDGE_MAX];
	static int32_t local_count_edge_ov[EDGE_MAX];
	static int8_t  local_valid_pulse;
	
	static int32_t count_size, count_size_filtered=0;
	static int32_t count_middle, count_middle_filtered=0;
	
	static int32_t local_angle;
	static int32_t local_dist, local_dist_total=0;
	
	int32_t local_robot_x=0;
	int32_t local_robot_y=0;
	int32_t local_robot_a=0;
	
	int32_t result_x=0;
	int32_t result_y=0;
	
	uint8_t flags;

	/* copy data to local variables */		
	IRQ_LOCK(flags);

	/* turn period measures */
	local_count_period 	 = count_period;
	local_count_period_ov = count_period_ov;
	local_valid_period 	 = valid_period;

	/* rising edge measure */
	local_count_edge[EDGE_RISING]  	= count_edge[sensor][EDGE_RISING];
	local_count_edge_ov[EDGE_RISING] = count_edge_ov[sensor][EDGE_RISING];

	/* falling edge measure */
	local_count_edge[EDGE_FALLING] = count_edge[sensor][EDGE_FALLING];
	local_count_edge_ov[EDGE_FALLING] = count_edge_ov[sensor][EDGE_FALLING];

	/* valid pulse flag */
	local_valid_pulse = valid_pulse[sensor];
	
	/* robot coodinates */
	local_robot_x = beacon.robot_x;
	local_robot_y = beacon.robot_y;
	local_robot_a = beacon.robot_a;

	IRQ_UNLOCK(flags);

#if BEACON_EXTERNAL_DEBUG_OTHER_LIKE_OUR
	/* for test other beacon like our */
	local_robot_x = 1500;
	local_robot_y = 1100;
	local_robot_a = 0;
#endif

	/* calculate/update turn period if valid*/
	if(local_valid_period){

		/* reset flag */
		IRQ_LOCK(flags);
		valid_period = 0;
		IRQ_UNLOCK(flags);

		/* calculate period in counts */
		local_count_period += ((MODULO_TIMER + 1)*local_count_period_ov);	

		/* low pass filtered version of period */		
		local_count_period_filtered = (int32_t)(local_count_period_filtered*0.8 + local_count_period*0.2);
	}
		
	/* if not valid pulse return */
	if(!local_valid_pulse){
		BEACON_NOTICE("non valid pulse\r\n\n");
		goto error;
	}
	
	/* continue with the calculus ... */

	/* reset valid flag */
	IRQ_LOCK(flags);
	valid_pulse[sensor]=0;
	IRQ_UNLOCK(flags);		

	/* calculate total edges counts */
	local_count_edge[EDGE_RISING] = local_count_edge[EDGE_RISING] + (MODULO_TIMER + 1)*local_count_edge_ov[EDGE_RISING];
	local_count_edge[EDGE_FALLING] = local_count_edge[EDGE_FALLING] + (MODULO_TIMER + 1)*local_count_edge_ov[EDGE_FALLING];

	/* calcule pulse size and the middle */	
	if(local_count_edge[EDGE_RISING]> local_count_edge[EDGE_FALLING])
	{
		count_size = local_count_period_filtered - local_count_edge[EDGE_RISING] + local_count_edge[EDGE_FALLING];
		count_middle = (local_count_edge[EDGE_RISING] + (int32_t)(count_size/2) + local_count_period_filtered) % local_count_period_filtered;			
	}
	else
	{
		count_size = local_count_edge[EDGE_FALLING] - local_count_edge[EDGE_RISING];
		count_middle = local_count_edge[EDGE_RISING] + (int32_t)(count_size/2);
	}
	
	/* if pulse width is out of range return */
/*		if(count_size > 5000){
			BEACON_DEBUG("count_size_discarted = %ld", count_size);
			return;
		}
*/		

	/* calcule angle */
	if(sensor == IR_SENSOR_180_DEG)
		local_angle = get_angle(count_middle, local_count_period_filtered, 180);
	else
		local_angle = get_angle(count_middle, local_count_period_filtered, 0);

	/* if angle is out of range return */
	if(local_angle > (85*MULT_ANGLE)){
		if(local_angle < (275*MULT_ANGLE)){
			//BEACON_DEBUG("angle_discarted = %ld", local_angle);	
			//return;
			goto error;
		}
	}


	/* ??? ceil count_size to multiple of 10 value ??? */
	count_size = ((int32_t)ceil(count_size/10.0))*10;	

	/* filter version of size and middle counts */
	count_size_filtered   = (int32_t)(count_size_filtered*0.8 + count_size*0.2); 
	count_middle_filtered = (int32_t)(count_middle_filtered*0.75 + count_middle*0.25);


	/* calculate distance in cm */
	local_dist = get_dist(count_size_filtered, local_count_period_filtered);
	local_dist_total = local_dist*10;


	/* calculate (x,y) coordenates relative to (0,0) */
	beacon_angle_dist_to_x_y((int32_t)(local_angle/MULT_ANGLE), local_dist, &result_x, &result_y);

	/* translate depends on beacon coordenates */
	result_x = COLOR_X(result_x*10 + BEACON_X_OFFSET);
	result_y = COLOR_Y((-result_y*10)+BEACON_Y_OFFSET);
			
	/* translate (x,y) coodinates to (d,a) coordinates relative to robot */
	local_angle /= MULT_ANGLE;
	abs_xy_to_rel_da(local_robot_x, local_robot_y, local_robot_a,
									 result_x, result_y, &local_dist, &local_angle);

	/* TODO: best debug */
	//BEACON_DEBUG(" ");
	//BEACON_DEBUG("count_period = %ld", local_count_period_filtered);
	//BEACON_DEBUG("count_edge[EDGE_RISING]  = %ld", local_count_edge[EDGE_RISING]);
	//BEACON_DEBUG("count_edge[EDGE_FALLING] = %ld", local_count_edge[EDGE_FALLING]);
	//BEACON_DEBUG("count_size = %ld", count_size_filtered);
	//BEACON_DEBUG("count_middle = %ld\r\n", count_middle_filtered);
	//BEACON_NOTICE("opponent rel beacon angle= %f\t",(double)(local_angle*1.0/MULT_ANGLE));		
	//BEACON_NOTICE("opponent rel beacon dist= %ld\r\n",local_dist_total);


	/* if opponent is in our robot area return */ 
	if(is_in_margin((result_x-local_robot_x), (result_y-local_robot_y), 255)){ // 255 robot_length/2 + error_baliza_max(100mm)
		BEACON_NOTICE("discard xy (%ld %ld) in robot (%ld %ld) margin\r\n",
								 result_x, result_y, local_robot_x, local_robot_y);	
		goto error;
	}
	
	/* reset timeout */
	invalid_count = 0;
	
	/* update results */	
	IRQ_LOCK(flags);			
	beacon.opponent_x = result_x;
	beacon.opponent_y = result_y;
	beacon.opponent_angle = local_angle;
	beacon.opponent_dist = local_dist;
	IRQ_UNLOCK(flags);

	BEACON_NOTICE("opponent angle= %ld\t",beacon.opponent_angle);
	BEACON_NOTICE("opponent dist= %ld\r\n",beacon.opponent_dist);
	BEACON_NOTICE("opponent x= %ld\t",beacon.opponent_x);
	BEACON_NOTICE("opponent y= %ld\r\n\n",beacon.opponent_y);

	return;

	error:
		/* 0.5 second timeout */
		if (invalid_count < 25)
			invalid_count++;
		else {
			IRQ_LOCK(flags);
			beacon.opponent_x = I2C_OPPONENT_NOT_THERE;
			IRQ_UNLOCK(flags);
			
			BEACON_NOTICE("opponent not there");
		}	
}

/* beacon calculus event */
void beacon_calc(void *dummy)
{
	if (beaconboard.flags & DO_BEACON){
		//sensor_calc(IR_SENSOR_0_DEG);
		sensor_calc(IR_SENSOR_180_DEG);
	}	
}






