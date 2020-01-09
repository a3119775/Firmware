/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file init.c
 *
 * BBBLUE specific initialization
 */
#include <stddef.h>
#include <px4_platform_common/log.h>

#include <robotcontrol.h>

#include "board_config.h"

// initialize roboticscape library similar to the deprecated rc_initialize()
int rc_init(void)
{
	if (rc_get_state() == RUNNING) {  return 0; }

	PX4_INFO("Initializing librobotcontrol ...");

	// make sure another instance isn't running
	rc_kill_existing_process(2.0f);

	// make PID file to indicate your project is running
	rc_make_pid_file();

	// start state as Uninitialized
	rc_set_state(UNINITIALIZED);

	if (rc_servo_init()) {  // Configures the PRU to send servo pulses
		PX4_ERR("rc_init failed to run rc_servo_init()");
		return -1;
	}

	if (rc_servo_power_rail_en(1)) { // Turning On 6V Servo Power Rail
		PX4_ERR("rc_init failed to run rc_servo_power_rail_en(1)");
		return -1;
	}

	rc_set_state(RUNNING);

	return 0;
}

void rc_cleaning(void)
{
	if (rc_get_state() == EXITING) { return; }

	rc_set_state(EXITING);

	rc_servo_power_rail_en(0);
	rc_servo_cleanup();

	rc_remove_pid_file();
}
