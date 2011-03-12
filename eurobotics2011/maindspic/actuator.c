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

#include <aversive.h>
#include <encoders_dspic.h>
#include <dac_mc.h>

#include "actuator.h"
#include "main.h"

void dac_set_and_save(void *dac, int32_t val)
{
	/* we need to do the saturation here, before saving the
	 * value */
	if (val > 65535)
		val = 65535;
	if (val < -65535)
		val = -65535;
	
	/* save value */
	/* XXX DAC CHANNEL R has an offset of -2000 points */
	if (dac == LEFT_MOTOR)
		mainboard.dac_l = val;
	else if (dac == RIGHT_MOTOR){
		val += 2000;
		mainboard.dac_r = val;
	}

	/* set value */
	dac_mc_set(dac, val);
}

/* lasers off */
void lasers_set_on(void)
{
	_LATC7 = 1;
}

/* lasers on */
void lasers_set_off(void)
{
	_LATC7 = 0;
}
