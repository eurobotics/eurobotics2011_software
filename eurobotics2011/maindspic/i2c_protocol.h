/*
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: i2c_protocol.h,v 1.5 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Bali�as Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  i2c_protocol.h,v 1.5 2009/05/27 20:04:07 zer0 Exp.
 */

#ifndef _I2C_PROTOCOL_H_
#define _I2C_PROTOCOL_H_


#include "i2c_mem.h"

void i2c_protocol_init(void);
void i2c_protocol_debug(void);

void i2cproto_wait_update(void);
void i2c_poll_slaves(void *dummy);

void i2c_read_event(uint8_t *rBuff, uint16_t size);
void i2c_recvbyteevent(uint8_t hwstatus, uint8_t i, uint8_t c);
void i2c_write_event(uint16_t size);

//int8_t i2c_set_color(uint8_t addr, uint8_t color);
int8_t i2c_led_control(uint8_t addr, uint8_t led, uint8_t state);

int8_t i2c_slavedspic_mode_exit(void);
int8_t i2c_slavedspic_mode_wait(void);
int8_t i2c_slavedspic_mode_init(void);
int8_t i2c_slavedspic_mode_hide_arm(void);
int8_t i2c_slavedspic_mode_show_arm(void);
int8_t i2c_slavedspic_mode_prepare_harvest_ball(void);
int8_t i2c_slavedspic_mode_harvest_tomato(void);

int8_t i2c_slavedspic_mode_putin_finger_ball(void);
int8_t i2c_slavedspic_mode_arm_pump_on(void);
int8_t i2c_slavedspic_mode_arm_pump_off(void);
int8_t i2c_slavedspic_mode_corn_rolls_in(void);
int8_t i2c_slavedspic_mode_corn_rolls_out(void);
int8_t i2c_slavedspic_mode_corn_rolls_stop(void);
int8_t i2c_slavedspic_mode_harvest_corn(void);
int8_t i2c_slavedspic_mode_out_corns(void);
int8_t i2c_slavedspic_mode_harvest_orange(int8_t angle, uint8_t height, uint8_t vacuum_time_div10);
int8_t i2c_slavedspic_mode_arm_goto_ah(int8_t angle, uint8_t height);


//int8_t i2c_mechboard_mode_manual(void);
//int8_t i2c_mechboard_mode_harvest(void);
//int8_t i2c_mechboard_mode_lazy_harvest(void);
//int8_t i2c_mechboard_mode_prepare_pickup(uint8_t side);
//int8_t i2c_mechboard_mode_prepare_pickup_next(uint8_t side, uint8_t next_mode);
//int8_t i2c_mechboard_mode_pickup(void);
//int8_t i2c_mechboard_mode_eject(void);
//int8_t i2c_mechboard_mode_lazy_pickup(void);
//
//int8_t i2c_mechboard_mode_prepare_build_both(uint8_t level);
//int8_t i2c_mechboard_mode_prepare_build_select(int8_t level_l, int8_t level_r);
//int8_t i2c_mechboard_mode_prepare_inside_both(uint8_t level);
//int8_t i2c_mechboard_mode_prepare_inside_select(int8_t level_l, int8_t level_r);
//int8_t i2c_mechboard_mode_simple_autobuild(uint8_t level);
//int8_t i2c_mechboard_mode_autobuild(uint8_t level_l, uint8_t count_l,
//				    uint8_t dist_l, 
//				    uint8_t level_r, uint8_t count_r,
//				    uint8_t dist_r, 
//				    uint8_t do_lintel);
//int8_t i2c_mechboard_mode_init(void);
//int8_t i2c_mechboard_mode_eject(void);
//int8_t i2c_mechboard_mode_prepare_get_lintel(void);
//int8_t i2c_mechboard_mode_get_lintel(void);
//int8_t i2c_mechboard_mode_put_lintel(void);
//int8_t i2c_mechboard_mode_clear(void);
//int8_t i2c_mechboard_mode_loaded(void);
//int8_t i2c_mechboard_mode_store(void);
//int8_t i2c_mechboard_mode_manivelle(void);
//int8_t i2c_mechboard_mode_push_temple(void);
//int8_t i2c_mechboard_mode_push_temple_disc(uint8_t side);
//
//int8_t i2c_sensorboard_set_beacon(uint8_t enable);
//
//int8_t i2c_sensorboard_scanner_set(uint8_t mode);
//static inline int8_t i2c_sensorboard_scanner_stop(void) {
//	return i2c_sensorboard_scanner_set(I2C_SENSORBOARD_SCANNER_STOP);
//}
//static inline int8_t i2c_sensorboard_scanner_start(void) {
//	return i2c_sensorboard_scanner_set(I2C_SENSORBOARD_SCANNER_START);
//}
//static inline int8_t i2c_sensorboard_scanner_prepare(void) {
//	return i2c_sensorboard_scanner_set(I2C_SENSORBOARD_SCANNER_PREPARE);
//}
//
//int8_t i2c_sensorboard_scanner_calib(void);
//
//int8_t i2c_sensorboard_scanner_algo_column(uint8_t zone,
//					   int16_t x, int16_t y);
//int8_t i2c_sensorboard_scanner_algo_check(uint8_t level,
//					  int16_t x, int16_t y);
//int8_t i2c_sensorboard_scanner_algo_temple(uint8_t zone,
//					   int16_t x, int16_t y);

#endif
