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
 *  Revision : $Id: commands_mechboard.c,v 1.5 2009/05/27 20:04:07 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <time.h>
#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"

#include "sensor.h"
#include "cmdline.h"
#include "state.h"
#include "i2c_protocol.h"
#include "actuator.h"
#include "main.h"

extern uint16_t state_debug;

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = 0;
		if (!strcmp_P(res->arg2, PSTR("on")))
			slavedspic.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			slavedspic.flags &= bit;
		else { /* show */
			//printf_P(PSTR("encoders is %s\r\n"), 
			//	 (DO_ENCODERS & slavedspic.flags) ? "on":"off");
		}
		return;
	}

	//if (!strcmp_P(res->arg1, PSTR("encoders")))
	//	bit = DO_ENCODERS;

	if (!strcmp_P(res->arg2, PSTR("on")))
		slavedspic.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		slavedspic.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1, 
		      (bit & slavedspic.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all";
parse_pgm_token_string_t cmd_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
prog_char str_event_arg2[] = "on#off#show";
parse_pgm_token_string_t cmd_event_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

prog_char help_event[] = "Enable/disable events";
parse_pgm_inst_t cmd_event = {
	.f = cmd_event_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_event,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_event_arg0, 
		(prog_void *)&cmd_event_arg1, 
		(prog_void *)&cmd_event_arg2, 
		NULL,
	},
};

/**********************************************************/
/* Color */

/* this structure is filled when cmd_color is parsed successfully */
struct cmd_color_result {
	fixed_string_t arg0;
	fixed_string_t color;
};

/* function called when cmd_color is parsed successfully */
static void cmd_color_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
	if (!strcmp_P(res->color, PSTR("red"))) {
		slavedspic.our_color = I2C_COLOR_RED;
	}
	else if (!strcmp_P(res->color, PSTR("blue"))) {
		slavedspic.our_color = I2C_COLOR_BLUE;
	}
	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "red#blue";
parse_pgm_token_string_t cmd_color_color = TOKEN_STRING_INITIALIZER(struct cmd_color_result, color, str_color_color);

prog_char help_color[] = "Set our color";
parse_pgm_inst_t cmd_color = {
	.f = cmd_color_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_color,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_color_arg0, 
		(prog_void *)&cmd_color_color, 
		NULL,
	},
};

/**********************************************************/
/* belts */

/* this structure is filled when cmd_belts is parsed successfully */
struct cmd_belts_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	uint16_t arg3;
};

/* function called when cmd_belts is parsed successfully */
static void cmd_belts_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_belts_result *res = (struct cmd_belts_result *) parsed_result;

	uint8_t side, mode, sensor;
	microseconds time_us;

	/* side */
	if (!strcmp(res->arg1, "rear")){
		side = BELTS_SIDE_REAR;
		sensor = S_REAR_TOKEN_STOP;
	}
	else{
		side = BELTS_SIDE_FRONT;
		sensor = S_FRONT_TOKEN_STOP;
	}

	/* mode */
	if (!strcmp(res->arg2, "in"))
		mode = BELTS_MODE_IN;
	else if (!strcmp(res->arg2, "out"))
		mode = BELTS_MODE_OUT;
	else if (!strcmp(res->arg2, "left"))
		mode = BELTS_MODE_LEFT;
	else
		mode = BELTS_MODE_RIGHT;
	
	/* execute */
	time_us = time_get_us2();
	belts_mode_set(side, mode, res->arg3);

	/* test performance */
#if 0
	printf("press a key for end ...\n\r");
	do{
		printf("load = %d\n\r", (uint16_t)belts_load_get(side));
		
		/* stop if final carrier is reached */
		if(sensor_get(sensor) && (mode==BELTS_MODE_IN))
			belts_set_mode(side, BELTS_MODE_OUT, 0);
		
		wait_ms(50);
	}while(!cmdline_keypressed());
#else
	printf("press a key for end ...\n\r");
	if(mode==BELTS_MODE_IN){
		while(!cmdline_keypressed() && !sensor_get(sensor));
		time_us = time_get_us2() - time_us;
		printf("input time = %d ms\n\r", (int16_t)(time_us/1000));
	}
	else if(mode==BELTS_MODE_OUT)
		WAIT_COND_OR_TIMEOUT(cmdline_keypressed(), 700);
	else
		while(!cmdline_keypressed());			
#endif

	/* stop belts */
	belts_mode_set(side, BELTS_MODE_OUT, 0);
	printf("done\r\n");
}

prog_char str_belts_arg0[] = "belts";
parse_pgm_token_string_t cmd_belts_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_belts_result, arg0, str_belts_arg0);
prog_char str_belts_arg1[] = "rear#front";
parse_pgm_token_string_t cmd_belts_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_belts_result, arg1, str_belts_arg1);
prog_char str_belts_arg2[] = "in#out#left#right";
parse_pgm_token_string_t cmd_belts_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_belts_result, arg2, str_belts_arg2);
parse_pgm_token_num_t cmd_belts_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_belts_result, arg3, UINT16);


prog_char help_belts[] = "manage belts";
parse_pgm_inst_t cmd_belts = {
	.f = cmd_belts_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_belts,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_belts_arg0, 
		(prog_void *)&cmd_belts_arg1, 
		(prog_void *)&cmd_belts_arg2, 
		(prog_void *)&cmd_belts_arg3, 
		NULL,
	},
};


///**********************************************************/
///* State1 */
//
///* this structure is filled when cmd_state1 is parsed successfully */
//struct cmd_state1_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//};
//
///* function called when cmd_state1 is parsed successfully */
//static void cmd_state1_parsed(void *parsed_result,
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state1_result *res = parsed_result;
//	struct i2c_cmd_slavedspic_set_mode command;
//
//	if (!strcmp_P(res->arg1, PSTR("init"))) {
//		state_init();
//		return;
//	}
//
//	if (!strcmp_P(res->arg1, PSTR("hide_arm")))
//		command.mode = I2C_SLAVEDSPIC_MODE_HIDE_ARM;
//	else if (!strcmp_P(res->arg1, PSTR("show_arm")))
//		command.mode = I2C_SLAVEDSPIC_MODE_SHOW_ARM;
//	else if (!strcmp_P(res->arg1, PSTR("prepare_harvest_ball")))
//		command.mode = I2C_SLAVEDSPIC_MODE_PREPARE_HARVEST_BALL;
//	else if (!strcmp_P(res->arg1, PSTR("harvest_tomato")))
//		command.mode = I2C_SLAVEDSPIC_MODE_HARVEST_TOMATO;
//	else if (!strcmp_P(res->arg1, PSTR("finger_ball")))
//		command.mode = I2C_SLAVEDSPIC_MODE_PUTIN_FINGER_BALL;
//	
//	state_set_mode(&command);
//}
//
//prog_char str_state1_arg0[] = "slavedspic";
//parse_pgm_token_string_t cmd_state1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg0, str_state1_arg0);
//prog_char str_state1_arg1[] = "init#hide_arm#show_arm#prepare_harvest_ball#harvest_tomato#finger_ball";
//parse_pgm_token_string_t cmd_state1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg1, str_state1_arg1);
//
//prog_char help_state1[] = "set slavedspic mode";
//parse_pgm_inst_t cmd_state1 = {
//	.f = cmd_state1_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state1,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state1_arg0, 
//		(prog_void *)&cmd_state1_arg1, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* State2 */
//
///* this structure is filled when cmd_state2 is parsed successfully */
//struct cmd_state2_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//};
//
///* function called when cmd_state2 is parsed successfully */
//static void cmd_state2_parsed(void *parsed_result,
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state2_result *res = parsed_result;
//	struct i2c_cmd_slavedspic_set_mode command;
//
//	if (!strcmp_P(res->arg1, PSTR("init"))) {
//		state_init();
//		return;
//	}
//
//
//	if (!strcmp_P(res->arg1, PSTR("pump_on")))
//		command.mode = I2C_SLAVEDSPIC_MODE_ARM_PUMP_ON;
//	else if (!strcmp_P(res->arg1, PSTR("pump_off")))
//		command.mode = I2C_SLAVEDSPIC_MODE_ARM_PUMP_OFF;
//	else if (!strcmp_P(res->arg1, PSTR("rolls_in")))
//		command.mode = I2C_SLAVEDSPIC_MODE_CORN_ROLLS_IN;
//	else if (!strcmp_P(res->arg1, PSTR("rolls_out")))
//		command.mode = I2C_SLAVEDSPIC_MODE_CORN_ROLLS_OUT;
//	else if (!strcmp_P(res->arg1, PSTR("rolls_stop")))
//		command.mode = I2C_SLAVEDSPIC_MODE_CORN_ROLLS_STOP;
//	else if (!strcmp_P(res->arg1, PSTR("harvest_corn")))
//		command.mode = I2C_SLAVEDSPIC_MODE_HARVEST_CORN;
//	else if (!strcmp_P(res->arg1, PSTR("out_corns")))
//		command.mode = I2C_SLAVEDSPIC_MODE_OUT_CORNS;
//	
//	state_set_mode(&command);
//}
//
//prog_char str_state2_arg0[] = "slavedspic";
//parse_pgm_token_string_t cmd_state2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg0, str_state2_arg0);
//prog_char str_state2_arg1[] = "pump_on#pump_off#rolls_in#rolls_out#rolls_stop#harvest_corn#out_corns";
//parse_pgm_token_string_t cmd_state2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg1, str_state2_arg1);
//
//prog_char help_state2[] = "set slavedspic mode";
//parse_pgm_inst_t cmd_state2 = {
//	.f = cmd_state2_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state2,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state2_arg0, 
//		(prog_void *)&cmd_state2_arg1, 
//		NULL,
//	},
//};
//
//
///**********************************************************/
///* State3 */
//
///* this structure is filled when cmd_state3 is parsed successfully */
//struct cmd_state3_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//	int16_t arg2;
//	int16_t arg3;
//};
//
///* function called when cmd_state3 is parsed successfully */
//static void cmd_state3_parsed(void *parsed_result,
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state3_result *res = parsed_result;
//	struct i2c_cmd_slavedspic_set_mode command;
//
//	if (!strcmp_P(res->arg1, PSTR("init"))) {
//		state_init();
//		return;
//	}
//
//
//	if (!strcmp_P(res->arg1, PSTR("arm_goto_ah"))){
//		command.mode = I2C_SLAVEDSPIC_MODE_ARM_GOTO_AH;
//		command.arm_goto.angle = res->arg2;
//		command.arm_goto.height = res->arg3;
//	}
//	else if (!strcmp_P(res->arg1, PSTR("harvest_orange"))){
//		command.mode = I2C_SLAVEDSPIC_MODE_HARVEST_ORANGE;
//		command.harvest_orange.position = CUSTOM_POSITION;
//		command.harvest_orange.angle = res->arg2;
//		command.harvest_orange.height = res->arg3;
//		command.harvest_orange.vacuum_time_div10 = 0;
//		
//	}
//	
//	state_set_mode(&command);
//}
//
//prog_char str_state3_arg0[] = "slavedspic";
//parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
//prog_char str_state3_arg1[] = "arm_goto_ah#harvest_orange";
//parse_pgm_token_string_t cmd_state3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg1, str_state3_arg1);
//parse_pgm_token_num_t cmd_state3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, arg2, INT16);
//parse_pgm_token_num_t cmd_state3_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, arg3, INT16);
//
//prog_char help_state3[] = "set slavedspic mode";
//parse_pgm_inst_t cmd_state3 = {
//	.f = cmd_state3_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state3,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state3_arg0, 
//		(prog_void *)&cmd_state3_arg1, 
//		(prog_void *)&cmd_state3_arg2, 
//		(prog_void *)&cmd_state3_arg3, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* State2 */
//
///* this structure is filled when cmd_state2 is parsed successfully */
//struct cmd_state2_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//	fixed_string_t arg2;
//};
//
///* function called when cmd_state2 is parsed successfully */
//static void cmd_state2_parsed(void *parsed_result,
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state2_result *res = parsed_result;
//	struct i2c_cmd_mechboard_set_mode command;
//	uint8_t side;
//
//	if (!strcmp_P(res->arg2, PSTR("left")))
//		side = I2C_LEFT_SIDE;
//	else if (!strcmp_P(res->arg2, PSTR("right")))
//		side = I2C_RIGHT_SIDE;
//	else if (!strcmp_P(res->arg2, PSTR("center")))
//		side = I2C_CENTER_SIDE;
//	else
//		side = I2C_AUTO_SIDE;
//
//	if (!strcmp_P(res->arg1, PSTR("prepare_pickup"))) {
//		command.mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;
//		command.prep_pickup.side = side;
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;
//	}
//	else if (!strcmp_P(res->arg1, PSTR("push_temple_disc"))) {
//		command.mode = I2C_MECHBOARD_MODE_PUSH_TEMPLE_DISC;
//		command.push_temple_disc.side = side;
//	}
//
//
//	state_set_mode(&command);
//}
//
//prog_char str_state2_arg0[] = "mechboard";
//parse_pgm_token_string_t cmd_state2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg0, str_state2_arg0);
//prog_char str_state2_arg1[] = "prepare_pickup#push_temple_disc";
//parse_pgm_token_string_t cmd_state2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg1, str_state2_arg1);
//prog_char str_state2_arg2[] = "left#right#auto#center";
//parse_pgm_token_string_t cmd_state2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg2, str_state2_arg2);
//
//prog_char help_state2[] = "set mechboard mode";
//parse_pgm_inst_t cmd_state2 = {
//	.f = cmd_state2_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state2,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state2_arg0, 
//		(prog_void *)&cmd_state2_arg1, 
//		(prog_void *)&cmd_state2_arg2, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* State3 */
//
///* this structure is filled when cmd_state3 is parsed successfully */
//struct cmd_state3_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//	uint8_t level;
//};
//
///* function called when cmd_state3 is parsed successfully */
//static void cmd_state3_parsed(void *parsed_result, 
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state3_result *res = parsed_result;
//	struct i2c_cmd_mechboard_set_mode command;
//
//	if (!strcmp_P(res->arg1, PSTR("prepare_build"))) {
//		command.mode = I2C_MECHBOARD_MODE_PREPARE_BUILD;
//		command.prep_build.level_l = res->level;
//		command.prep_build.level_r = res->level;
//	}
//	else if (!strcmp_P(res->arg1, PSTR("prepare_inside"))) {
//		command.mode = I2C_MECHBOARD_MODE_PREPARE_INSIDE;
//		command.prep_inside.level_l = res->level;
//		command.prep_inside.level_r = res->level;
//	}
//	else if (!strcmp_P(res->arg1, PSTR("autobuild"))) {
//		command.mode = I2C_MECHBOARD_MODE_AUTOBUILD;
//		command.autobuild.level_left = res->level;
//		command.autobuild.level_right = res->level;
//		command.autobuild.count_left = 2;
//		command.autobuild.count_right = 2;
//		command.autobuild.distance_left = I2C_AUTOBUILD_DEFAULT_DIST;
//		command.autobuild.distance_right = I2C_AUTOBUILD_DEFAULT_DIST;
//		command.autobuild.do_lintel = 1;
//	}
//	state_set_mode(&command);
//}
//
//prog_char str_state3_arg0[] = "mechboard";
//parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
//prog_char str_state3_arg1[] = "prepare_build#autobuild#prepare_inside";
//parse_pgm_token_string_t cmd_state3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg1, str_state3_arg1);
//parse_pgm_token_num_t cmd_state3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, level, UINT8);
//
//prog_char help_state3[] = "set mechboard mode";
//parse_pgm_inst_t cmd_state3 = {
//	.f = cmd_state3_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state3,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state3_arg0, 
//		(prog_void *)&cmd_state3_arg1, 
//		(prog_void *)&cmd_state3_arg2, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* State4 */
//
///* this structure is filled when cmd_state4 is parsed successfully */
//struct cmd_state4_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//	uint8_t level_l;
//	uint8_t count_l;
//	uint8_t dist_l;
//	uint8_t level_r;
//	uint8_t count_r;
//	uint8_t dist_r;
//	uint8_t do_lintel;
//};
//
///* function called when cmd_state4 is parsed successfully */
//static void cmd_state4_parsed(void *parsed_result, 
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state4_result *res = parsed_result;
//	struct i2c_cmd_mechboard_set_mode command;
//
//	if (!strcmp_P(res->arg1, PSTR("autobuild"))) {
//		command.mode = I2C_MECHBOARD_MODE_AUTOBUILD;
//		command.autobuild.distance_left = res->dist_l;
//		command.autobuild.distance_right = res->dist_r;
//		command.autobuild.level_left = res->level_l;
//		command.autobuild.level_right = res->level_r;
//		command.autobuild.count_left = res->count_l;
//		command.autobuild.count_right = res->count_r;
//		command.autobuild.do_lintel = res->do_lintel;
//	}
//	state_set_mode(&command);
//}
//
//prog_char str_state4_arg0[] = "mechboard";
//parse_pgm_token_string_t cmd_state4_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state4_result, arg0, str_state4_arg0);
//prog_char str_state4_arg1[] = "autobuild";
//parse_pgm_token_string_t cmd_state4_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state4_result, arg1, str_state4_arg1);
//parse_pgm_token_num_t cmd_state4_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, level_l, UINT8);
//parse_pgm_token_num_t cmd_state4_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, count_l, UINT8);
//parse_pgm_token_num_t cmd_state4_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, dist_l, UINT8);
//parse_pgm_token_num_t cmd_state4_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, level_r, UINT8);
//parse_pgm_token_num_t cmd_state4_arg6 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, count_r, UINT8);
//parse_pgm_token_num_t cmd_state4_arg7 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, dist_r, UINT8);
//parse_pgm_token_num_t cmd_state4_arg8 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, do_lintel, UINT8);
//
//prog_char help_state4[] = "set mechboard mode (autobuild level_l count_l dist_l level_r count_r dist_r lintel)";
//parse_pgm_inst_t cmd_state4 = {
//	.f = cmd_state4_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state4,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state4_arg0, 
//		(prog_void *)&cmd_state4_arg1, 
//		(prog_void *)&cmd_state4_arg2, 
//		(prog_void *)&cmd_state4_arg3, 
//		(prog_void *)&cmd_state4_arg4, 
//		(prog_void *)&cmd_state4_arg5, 
//		(prog_void *)&cmd_state4_arg6, 
//		(prog_void *)&cmd_state4_arg7, 
//		(prog_void *)&cmd_state4_arg8, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* State5 */
//
///* this structure is filled when cmd_state5 is parsed successfully */
//struct cmd_state5_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//	fixed_string_t arg2;
//	fixed_string_t arg3;
//};
//
///* function called when cmd_state5 is parsed successfully */
//static void cmd_state5_parsed(void *parsed_result,
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state5_result *res = parsed_result;
//	struct i2c_cmd_mechboard_set_mode command;
//	uint8_t side;
//
//	if (!strcmp_P(res->arg2, PSTR("left")))
//		side = I2C_LEFT_SIDE;
//	else if (!strcmp_P(res->arg2, PSTR("right")))
//		side = I2C_RIGHT_SIDE;
//	else if (!strcmp_P(res->arg2, PSTR("center")))
//		side = I2C_CENTER_SIDE;
//	else
//		side = I2C_AUTO_SIDE;
//
//	command.mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;
//	command.prep_pickup.side = side;
//
//	if (!strcmp_P(res->arg3, PSTR("harvest")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_HARVEST;
//	else if (!strcmp_P(res->arg3, PSTR("lazy_harvest")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_LAZY_HARVEST;
//	else if (!strcmp_P(res->arg3, PSTR("pickup")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_PICKUP;
//	else if (!strcmp_P(res->arg3, PSTR("clear")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_CLEAR;
//	else if (!strcmp_P(res->arg3, PSTR("store")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_STORE;
//	else if (!strcmp_P(res->arg3, PSTR("lazy_pickup")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_LAZY_PICKUP;
//
//	state_set_mode(&command);
//}
//
//prog_char str_state5_arg0[] = "mechboard";
//parse_pgm_token_string_t cmd_state5_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state5_result, arg0, str_state5_arg0);
//prog_char str_state5_arg1[] = "prepare_pickup";
//parse_pgm_token_string_t cmd_state5_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state5_result, arg1, str_state5_arg1);
//prog_char str_state5_arg2[] = "left#right#auto#center";
//parse_pgm_token_string_t cmd_state5_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_state5_result, arg2, str_state5_arg2);
//prog_char str_state5_arg3[] = "harvest#pickup#store#lazy_harvest#lazy_pickup#clear";
//parse_pgm_token_string_t cmd_state5_arg3 = TOKEN_STRING_INITIALIZER(struct cmd_state5_result, arg3, str_state5_arg3);
//
//prog_char help_state5[] = "set mechboard mode 2";
//parse_pgm_inst_t cmd_state5 = {
//	.f = cmd_state5_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state5,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state5_arg0, 
//		(prog_void *)&cmd_state5_arg1, 
//		(prog_void *)&cmd_state5_arg2, 
//		(prog_void *)&cmd_state5_arg3, 
//		NULL,
//	},
//};




/**********************************************************/
/* State_Machine */

/* this structure is filled when cmd_state_machine is parsed successfully */
struct cmd_state_machine_result {
	fixed_string_t arg0;
};

/* function called when cmd_state_machine is parsed successfully */
static void cmd_state_machine_parsed(__attribute__((unused)) void *parsed_result,
				     __attribute__((unused)) void *data)
{
	state_machine();
}

prog_char str_state_machine_arg0[] = "state_machine";
parse_pgm_token_string_t cmd_state_machine_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_machine_result, arg0, str_state_machine_arg0);

prog_char help_state_machine[] = "launch state machine";
parse_pgm_inst_t cmd_state_machine = {
	.f = cmd_state_machine_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state_machine,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state_machine_arg0, 
		NULL,
	},
};

/**********************************************************/
/* State_Debug */

/* this structure is filled when cmd_state_debug is parsed successfully */
struct cmd_state_debug_result {
	fixed_string_t arg0;
	uint8_t on;
};

/* function called when cmd_state_debug is parsed successfully */
static void cmd_state_debug_parsed(void *parsed_result,
				   __attribute__((unused)) void *data)
{
	struct cmd_state_debug_result *res = parsed_result;
	state_debug = res->on;
}

prog_char str_state_debug_arg0[] = "state_debug";
parse_pgm_token_string_t cmd_state_debug_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_debug_result, arg0, str_state_debug_arg0);
parse_pgm_token_num_t cmd_state_debug_on = TOKEN_NUM_INITIALIZER(struct cmd_state_debug_result, on, UINT8);

prog_char help_state_debug[] = "Set debug timer for state machine";
parse_pgm_inst_t cmd_state_debug = {
	.f = cmd_state_debug_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state_debug,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state_debug_arg0, 
		(prog_void *)&cmd_state_debug_on, 
		NULL,
	},
};

/**********************************************************/
/* Test */

/* this structure is filled when cmd_test is parsed successfully */
struct cmd_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_test is parsed successfully */
static void cmd_test_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
}

prog_char str_test_arg0[] = "test";
parse_pgm_token_string_t cmd_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_test_result, arg0, str_test_arg0);

prog_char help_test[] = "Test function";
parse_pgm_inst_t cmd_test = {
	.f = cmd_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_test_arg0, 
		NULL,
	},
};


