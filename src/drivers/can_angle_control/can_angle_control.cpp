/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "can_angle_control.hpp"

#include <px4_platform_common/log.h>

#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <cmath>

#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)
#include <sys/socket.h>
#include <net/if.h>
#include <nuttx/can.h>
#include <netpacket/can.h>
#endif

CanAngleControl::CanAngleControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

CanAngleControl::~CanAngleControl()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

bool CanAngleControl::open_device()
{
#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)
	_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (_fd < 0) {
		PX4_ERR("Failed to create CAN socket");
		return false;
	}

	// Non-blocking mode so the periodic Run() loop is not blocked by recv().
	int flags = ::fcntl(_fd, F_GETFL, 0);

	if (flags < 0 || ::fcntl(_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
		PX4_ERR("Failed to set CAN socket non-blocking");
		return false;
	}

	// Bind to the configured CAN interface (e.g. can0).
	struct ifreq ifr {};
	snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "can%d", (int)_param_can_dev.get());

	if (::ioctl(_fd, SIOCGIFINDEX, &ifr) < 0) {
		PX4_ERR("Failed to get CAN interface index for can%d", (int)_param_can_dev.get());
		return false;
	}

	struct sockaddr_can addr {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (::bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("Failed to bind CAN socket");
		return false;
	}

	return true;
#else
	PX4_ERR("CAN angle control driver requires NuttX with CONFIG_NET_CAN");
	return false;
#endif
}

int CanAngleControl::start()
{
	ScheduleOnInterval(20'000, 500'000); // 50 Hz, 500 ms startup delay
	return PX4_OK;
}

void CanAngleControl::Run()
{
	if (should_exit()) {
		if (_initialized) {
			send_disable();
		}

		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		updateParams();

		if (!_param_enable.get()) {
			PX4_ERR("Not enabled (CANANG_ENABLE=0)");
			request_stop();
			return;
		}

		if (!open_device()) {
			request_stop();
			return;
		}

		_initialized = true;
	}

	updateParams();

	const hrt_abstime now = hrt_absolute_time();
	float dt = 0.f;

	if (_last_run_time != 0) {
		dt = (now - _last_run_time) * 1e-6f;
	}

	_last_run_time = now;

	update_setpoint();
	check_timeouts();
	validate_setpoint(dt);
	send_command();
	receive_feedback();
	publish_status();
}

void CanAngleControl::update_setpoint()
{
	can_angle_command_s cmd {};

	if (_cmd_sub.update(&cmd)) {
		_last_command_time = hrt_absolute_time();
		_cmd_angle_rad = cmd.angle_setpoint;
		_cmd_valid = PX4_ISFINITE(_cmd_angle_rad);
		_enable_cmd = cmd.enable;
	}
}

void CanAngleControl::check_timeouts()
{
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime timeout_us = (hrt_abstime)(_param_tout.get() * 1000.f);

	if ((now - _last_command_time) > timeout_us) {
		_enable_cmd = false;
		_cmd_valid = false;
	}
}

bool CanAngleControl::validate_setpoint(float dt)
{
	bool enable = _enable_cmd && _cmd_valid && PX4_ISFINITE(_cmd_angle_rad);

	if (!enable) {
		_enabled = false;
		_angle_setpoint_rad = 0.f;
		return false;
	}

	_enabled = true;

	float desired_deg = math::degrees(_cmd_angle_rad);
	desired_deg = math::constrain(desired_deg, _param_min.get(), _param_max.get());

	if ((_param_slew.get() > 0.f) && (dt > 0.f) && PX4_ISFINITE(_angle_setpoint_rad)) {
		const float max_delta = _param_slew.get() * dt;
		const float current_deg = math::degrees(_angle_setpoint_rad);
		desired_deg = math::constrain(desired_deg, current_deg - max_delta, current_deg + max_delta);
	}

	_angle_setpoint_rad = math::radians(math::constrain(desired_deg, _param_min.get(), _param_max.get()));
	return true;
}

void CanAngleControl::send_command()
{
#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)

	if (_fd < 0) {
		return;
	}

	const int direction = (_param_dir.get() == 0) ? 1 : -1;
	const float raw_deg = direction * (math::degrees(_angle_setpoint_rad) + _param_offset.get());

	// If the raw value overflows int16 range it will be clipped, which can cause
	// large commanded angles. Clamp the physical setpoint to a safe range first.
	const int16_t raw = math::constrainFloatToInt16(roundf(raw_deg * 100.0f));
	const uint16_t raw_u = static_cast<uint16_t>(raw);

	uint8_t data[FRAME_DLC] {};
	data[0] = raw_u & 0xFF;
	data[1] = (raw_u >> 8) & 0xFF;
	data[2] = _enabled ? 1 : 0;
	data[3] = _cmd_counter++;
	// data[4..5] reserved

	uint8_t checksum = 0;

	for (int i = 0; i < CHECKSUM_LEN; i++) {
		checksum += data[i];
	}

	data[CHECKSUM_LEN] = checksum;
	data[7] = 0;

	struct can_frame tx {};
	tx.can_id = static_cast<uint32_t>(_param_tx_id.get()) & CAN_SFF_MASK;
	tx.can_dlc = FRAME_DLC;
	memcpy(tx.data, data, FRAME_DLC);

	const ssize_t n = ::write(_fd, &tx, sizeof(tx));

	if (n == (ssize_t)sizeof(tx)) {
		_tx_count++;

	} else {
		_error_count++;
		PX4_DEBUG("CAN write failed: %d", (int)n);
	}
#endif // __PX4_NUTTX
}

void CanAngleControl::receive_feedback()
{
#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)

	if (_fd < 0) {
		return;
	}

	struct pollfd fds {};
	fds.fd = _fd;
	fds.events = POLLIN;

	while (::poll(&fds, 1, 0) > 0) {
		if (!(fds.revents & POLLIN)) {
			break;
		}

		struct can_frame rx {};
		const ssize_t n = ::read(_fd, &rx, sizeof(rx));

		if (n != (ssize_t)sizeof(rx)) {
			continue;
		}

		if ((rx.can_id & CAN_SFF_MASK) != static_cast<uint32_t>(_param_rx_id.get())) {
			continue;
		}

		if (rx.can_dlc < FRAME_DLC) {
			continue;
		}

		uint8_t checksum = 0;

		for (int i = 0; i < CHECKSUM_LEN; i++) {
			checksum += rx.data[i];
		}

		if (checksum != rx.data[CHECKSUM_LEN]) {
			_error_count++;
			PX4_DEBUG("feedback checksum error");
			continue;
		}

		const int16_t raw = static_cast<int16_t>(rx.data[0] | (rx.data[1] << 8));
		const float raw_deg = raw * 0.01f;
		const int direction = (_param_dir.get() == 0) ? 1 : -1;
		const float physical_deg = direction * raw_deg - _param_offset.get();
		_actual_angle_rad = math::radians(math::constrain(physical_deg, _param_min.get(), _param_max.get()));

		_device_error = static_cast<uint16_t>(rx.data[4] | (rx.data[5] << 8));

		_last_feedback_time = hrt_absolute_time();
		_rx_count++;
	}
#endif // __PX4_NUTTX
}

void CanAngleControl::publish_status()
{
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime timeout_us = (hrt_abstime)(_param_tout.get() * 1000.f);

	can_angle_status_s status {};
	status.timestamp = now;
	status.timestamp_sample = _last_feedback_time;
	status.angle_setpoint = _angle_setpoint_rad;
	status.angle = _actual_angle_rad;
	status.enabled = _enabled;
	status.communication_ok = ((now - _last_feedback_time) < timeout_us) && (_last_feedback_time != 0);
	status.device_error = _device_error;
	status.rx_count = _rx_count;
	status.tx_count = _tx_count;
	status.error_count = _error_count;

	_status_pub.publish(status);
}

void CanAngleControl::send_disable()
{
	_enabled = false;
	_angle_setpoint_rad = 0.f;
	send_command();
}

int CanAngleControl::task_spawn(int argc, char *argv[])
{
	CanAngleControl *instance = new CanAngleControl();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->start();

	return 0;
}

int CanAngleControl::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for controlling an angle actuator over a raw CAN bus.

It subscribes to the `can_angle_command` uORB topic and publishes the
actual angle/state on `can_angle_status`.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("can_angle_control", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int CanAngleControl::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int can_angle_control_main(int argc, char *argv[])
{
	return CanAngleControl::main(argc, argv);
}
