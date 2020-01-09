/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include "NavioSysRCInput.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

namespace navio_sysfs_rc_in
{

#define RCINPUT_DEVICE_PATH_BASE "/sys/kernel/rcio/rcin"

#define RCINPUT_MEASURE_INTERVAL_US 20000 // microseconds

NavioSysRCInput::NavioSysRCInput() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_isRunning = true;
};

NavioSysRCInput::~NavioSysRCInput()
{
	ScheduleClear();

	_isRunning = false;

	for (int i = 0; i < CHANNELS; ++i) {
		::close(_channel_fd[i]);
	}

	::close(_connected_fd);
}

int NavioSysRCInput::navio_rc_init()
{
	_connected_fd = ::open("/sys/kernel/rcio/rcin/connected", O_RDONLY);

	for (int i = 0; i < CHANNELS; ++i) {
		char buf[10] {};
		::snprintf(buf, sizeof(buf), "%s/ch%d", RCINPUT_DEVICE_PATH_BASE, i);
		int fd = ::open(buf, O_RDONLY);

		if (fd < 0) {
			PX4_ERR("open %s (%d) failed", buf, i);
			break;
		}

		_channel_fd[i] = fd;
	}

	return PX4_OK;
}

int NavioSysRCInput::start()
{
	navio_rc_init();

	_should_exit.store(false);

	ScheduleOnInterval(RCINPUT_MEASURE_INTERVAL_US);

	return PX4_OK;
}

void NavioSysRCInput::stop()
{
	_should_exit.store(true);
}

void NavioSysRCInput::Run()
{
	if (_should_exit.load()) {
		ScheduleClear();
		return;
	}

	char connected_buf[12] {};
	int ret_connected = ::pread(_connected_fd, connected_buf, sizeof(connected_buf) - 1, 0);

	if (ret_connected < 0) {
		return;
	}

	connected_buf[sizeof(connected_buf) - 1] = '\0';
	const bool connected = (atoi(connected_buf) == 1);

	if (!connected) {
		return;
	}

	uint64_t timestamp_sample = hrt_absolute_time();

	input_rc_s data{};

	for (int i = 0; i < CHANNELS; ++i) {
		char buf[12] {};
		int res = ::pread(_channel_fd[i], buf, sizeof(buf) - 1, 0);

		if (res < 0) {
			continue;
		}

		buf[sizeof(buf) - 1] = '\0';

		data.values[i] = atoi(buf);
	}

	// check if all channels are 0
	bool all_zero = true;

	for (int i = 0; i < CHANNELS; ++i) {
		if (data.values[i] != 0) {
			all_zero = false;
		}
	}

	if (all_zero) {
		return;
	}

	data.timestamp_last_signal = timestamp_sample;
	data.channel_count = CHANNELS;
	data.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
	data.timestamp = hrt_absolute_time();

	_input_rc_pub.publish(data);
}

}; // namespace navio_sysfs_rc_in
