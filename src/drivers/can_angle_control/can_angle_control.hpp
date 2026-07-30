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

#pragma once

#include <stdint.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/can_angle_command.h>
#include <uORB/topics/can_angle_status.h>

using namespace time_literals;

class CanAngleControl : public ModuleBase<CanAngleControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CanAngleControl();
	~CanAngleControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int start();

private:
	void Run() override;

	bool open_device();
	void update_setpoint();
	void check_timeouts();
	bool validate_setpoint(float delta_time);
	void send_command();
	void send_disable();
	void receive_feedback();
	void publish_status();

	uORB::Subscription _cmd_sub{ORB_ID(can_angle_command)};
	uORB::Publication<can_angle_status_s> _status_pub{ORB_ID(can_angle_status)};

	orb_advert_t _mavlink_log_pub{nullptr};

	int _fd{-1};

	bool _initialized{false};
	bool _enable_cmd{false};
	bool _cmd_valid{false};
	bool _enabled{false};

	float _cmd_angle_rad{0.f};
	float _angle_setpoint_rad{0.f};
	float _actual_angle_rad{0.f};

	bool _receive_feedback_started{false};

	hrt_abstime _last_command_time{0};
	hrt_abstime _last_feedback_time{0};
	hrt_abstime _last_run_time{0};

	uint16_t _device_error{0};
	uint32_t _rx_count{0};
	uint32_t _tx_count{0};
	uint32_t _error_count{0};
	uint8_t _cmd_counter{0};

	static constexpr uint8_t FRAME_DLC = 8;
	static constexpr uint8_t CHECKSUM_LEN = 6;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_CTRL_ENABLE>) _param_enable,
		(ParamInt<px4::params::CA_CTRL_BITRATE>) _param_bitrate,
		(ParamInt<px4::params::CA_CTRL_TX_ID>) _param_tx_id,
		(ParamInt<px4::params::CA_CTRL_RX_ID>) _param_rx_id,
		(ParamInt<px4::params::CA_CTRL_DEV>) _param_dev,
		(ParamInt<px4::params::CA_CTRL_DIR>) _param_direction,
		(ParamFloat<px4::params::CA_CTRL_MIN>) _param_min,
		(ParamFloat<px4::params::CA_CTRL_MAX>) _param_max,
		(ParamFloat<px4::params::CA_CTRL_RATE>) _param_rate,
		(ParamFloat<px4::params::CA_CTRL_TIMEOUT>) _param_timeout,
		(ParamFloat<px4::params::CA_CTRL_OFFSET>) _param_offset
	)
};
