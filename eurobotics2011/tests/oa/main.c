#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include <aversive.h>
#include <aversive/error.h>

#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>

#include "../../common/i2c_commands.h"

#include "../../maindspic/strat.h"
#include "../../maindspic/strat.c"
#include "../../maindspic/strat_avoid.h"
#include "../../maindspic/strat_avoid.c"

#ifndef HOST_VERSION
#error only for host
#endif

#define M_2PI (M_PI*2)
#define E_USER_STRAT 200

#define ROBOT_X 		1000
#define ROBOT_Y 		1000
#define ROBOT_A 		0.5   /* radian */
#define ROBOT_A_DEG ((int)((ROBOT_A*180)/3.14159))


/* log function, add a command to configure
 * it dynamically */
void mylog(struct error * e, ...) 
{
	va_list ap;
	va_start(ap, e);

	vfprintf(stdout, e->text, ap);
	printf_P(PSTR("\r\n"));
	va_end(ap);
}

#ifdef HOST_VERSION
int main(int argc, char **argv)
#else
int main(void)
#endif
{
   int16_t robot_x;
	int16_t robot_y;
	int16_t robot_a_deg;
	double  robot_a;
	int16_t opp_x;
	int16_t opp_y;
	int16_t dst_x;
	int16_t dst_y;
   
#ifdef HOST_VERSION
	if (argc != 8) {
		printf("bad args\n");
		return -1;
	}
	robot_x = atoi(argv[1]);
	robot_y = atoi(argv[2]);
	robot_a_deg = atoi(argv[3]);
	robot_a = (((double)robot_a_deg*M_PI)/3.14159);
	dst_x = atoi(argv[4]);
	dst_y = atoi(argv[5]);
	opp_x = atoi(argv[6]);
	opp_y = atoi(argv[7]);
#endif
   
	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);
	
	/* set playground boundingbox */
	strat_set_bounding_box();

	/* goto and avoid */
	DEBUG(E_USER_STRAT, "robot at: %d %d %d", robot_x, robot_y, robot_a_deg);
	goto_and_avoid(dst_x, dst_y,
					   robot_x, robot_y, robot_a,
					   opp_x, opp_y);

	return 0;
}
