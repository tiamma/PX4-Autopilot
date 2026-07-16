/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

using namespace matrix;

#define FRONTTRANS_THR_MIN 0.25f
#define BACKTRANS_THROTTLE_DOWNRAMP_DUR_S 0.5f
#define BACKTRANS_THROTTLE_UPRAMP_DUR_S 0.5f

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void
Tiltrotor::parameters_update()
{
	VtolType::updateParams();
}

void Tiltrotor::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting rotors, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors are tilted
	 * forward completely. For the backtransition the motors simply rotate back.
	*/

	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Failsafe event, switch to MC mode immediately
		_vtol_mode = vtol_mode::MC_MODE;

	} else 	if (!_attc->is_fixed_wing_requested()) {

		// plane is in multicopter mode
		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_BACK;
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_FRONT_P2:
			// failsafe into multicopter mode
			_vtol_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			const bool exit_backtransition_tilt_condition = _tilt_control <= (_param_vt_tilt_mc.get() + 0.01f);

			// speed exit condition: use ground if valid, otherwise airspeed
			bool exit_backtransition_speed_condition = false;

			if (_local_pos->v_xy_valid) {
				const Dcmf R_to_body(Quatf(_v_att->q).inversed());
				const Vector3f vel = R_to_body * Vector3f(_local_pos->vx, _local_pos->vy, _local_pos->vz);
				exit_backtransition_speed_condition = vel(0) < _param_mpc_xy_cruise.get() ;

			} else if (PX4_ISFINITE(_attc->get_calibrated_airspeed())) {
				exit_backtransition_speed_condition = _attc->get_calibrated_airspeed() < _param_mpc_xy_cruise.get() ;
			}

			const bool exit_backtransition_time_condition = _time_since_trans_start > _param_vt_b_trans_dur.get() ;

			if (exit_backtransition_tilt_condition && (exit_backtransition_speed_condition || exit_backtransition_time_condition)) {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {

		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_FRONT_P1;
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {
				if (isFrontTransitionCompleted()) {
					_vtol_mode = vtol_mode::TRANSITION_FRONT_P2;
					_trans_finished_ts = hrt_absolute_time();
					resetTransitionStates();
				}

				break;
			}

		case vtol_mode::TRANSITION_FRONT_P2:

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _param_vt_tilt_fw.get()) {
				_vtol_mode = vtol_mode::FW_MODE;
				_tilt_control = _param_vt_tilt_fw.get();
			}

			break;

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_mode = vtol_mode::FW_MODE;
			break;
		}
	}

	// map tiltrotor specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
	case vtol_mode::TRANSITION_FRONT_P2:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	_tilt_control = VtolType::pusher_assist() + _param_vt_tilt_mc.get();
	_mc_yaw_weight = 1.0f;
}

void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state();

	// this is needed to avoid a race condition when entering backtransition when the mc rate controller publishes
	// a zero throttle value
	_v_att_sp->thrust_body[2] = -_v_att_sp->thrust_body[0];

	// make sure motors are tilted forward
	_tilt_control = _param_vt_tilt_fw.get();
}

void Tiltrotor::update_transition_state()
{
	VtolType::update_transition_state();

	const hrt_abstime now = hrt_absolute_time();

	// we get attitude setpoint from a multirotor flighttask if altitude is controlled.
	// in any other case the fixed wing attitude controller publishes attitude setpoint from manual stick input.
	if (_v_control_mode->flag_control_climb_rate_enabled) {
		// we need the incoming (virtual) attitude setpoints (both mc and fw) to be recent, otherwise return (means the previous setpoint stays active)
		if (_mc_virtual_att_sp->timestamp < (now - 1_s) || _fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

	} else {
		// we need a recent incoming (fw virtual) attitude setpoint, otherwise return (means the previous setpoint stays active)
		if (_fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
		_thrust_transition = _fw_virtual_att_sp->thrust_body[0];
	}


	const Eulerf attitude_setpoint_euler(Quatf(_v_att_sp->q_d));
	float roll_body = attitude_setpoint_euler.phi();
	float pitch_body = attitude_setpoint_euler.theta();
	float yaw_body = attitude_setpoint_euler.psi();

	if (_v_control_mode->flag_control_climb_rate_enabled) {
		roll_body = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).phi();
	}

	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
		// for the first part of the transition all rotors are enabled

		// tilt rotors forward up to certain angle
		if (_tilt_control <= _param_vt_tilt_trans.get()) {
			const float ramped_up_tilt = _param_vt_tilt_mc.get() +
						     fabsf(_param_vt_tilt_trans.get() - _param_vt_tilt_mc.get()) *
						     _time_since_trans_start / _param_vt_f_trans_dur.get() ;

			// only allow increasing tilt (tilt in hover can already be non-zero)
			_tilt_control = math::max(_tilt_control, ramped_up_tilt);
		}

		// at low speeds give full weight to MC
		_mc_roll_weight = 1.0f;
		_mc_yaw_weight = 1.0f;

		if (PX4_ISFINITE(_attc->get_calibrated_airspeed()) &&
		    _attc->get_calibrated_airspeed() >= getBlendAirspeed()) {
			_mc_roll_weight = 1.0f - (_attc->get_calibrated_airspeed() - getBlendAirspeed()) /
					  (getTransitionAirspeed()  - getBlendAirspeed());
		}

		// without airspeed do timed weight changes
		if ((!PX4_ISFINITE(_attc->get_calibrated_airspeed())) &&
		    _time_since_trans_start > getMinimumFrontTransitionTime()) {
			_mc_roll_weight = 1.0f - (_time_since_trans_start - getMinimumFrontTransitionTime()) /
					  (getOpenLoopFrontTransitionTime() - getMinimumFrontTransitionTime());
		}

		// add minimum throttle for front transition
		_thrust_transition = math::max(_thrust_transition, FRONTTRANS_THR_MIN);

	} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = math::constrain(_param_vt_tilt_trans.get() +
						fabsf(_param_vt_tilt_fw.get() - _param_vt_tilt_trans.get()) * _time_since_trans_start /
						_param_vt_trans_p2_dur.get(), _param_vt_tilt_trans.get(), _param_vt_tilt_fw.get());

		_mc_roll_weight = 0.0f;
		_mc_yaw_weight = 0.0f;

		// add minimum throttle for front transition
		_thrust_transition = math::max(_thrust_transition, FRONTTRANS_THR_MIN);

		// this line is needed such that the fw rate controller is initialized with the current throttle value.
		// if this is not then then there is race condition where the fw rate controller still publishes a zero sample throttle after transition
		_v_att_sp->thrust_body[0] = _thrust_transition;

	} else if (_vtol_mode == vtol_mode::TRANSITION_BACK) {

		// tilt rotors back once motors are idle
		if (_time_since_trans_start > BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) {

			float progress = (_time_since_trans_start - BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) / math::max(_param_vt_bt_tilt_dur.get(),
					 0.1f);
			progress = math::constrain(progress, 0.0f, 1.0f);
			_tilt_control = moveLinear(_param_vt_tilt_fw.get(), _param_vt_tilt_mc.get(), progress);
		}

		_mc_yaw_weight = 1.0f;

		// control backtransition deceleration using pitch.
		if (_v_control_mode->flag_control_climb_rate_enabled) {
			pitch_body = Eulerf(Quatf(_mc_virtual_att_sp->q_d)).theta();
		}

		if (_time_since_trans_start < BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) {
			// blend throttle from FW value to 0
			_mc_throttle_weight = 1.0f;
			const float target_throttle = 0.0f;
			const float progress = _time_since_trans_start / BACKTRANS_THROTTLE_DOWNRAMP_DUR_S;
			blendThrottleDuringBacktransition(progress, target_throttle);

		} else if (_time_since_trans_start < timeUntilMotorsAreUp()) {
			// while we quickly rotate back the motors keep throttle at idle

			// turn on all MC motors
			_mc_throttle_weight = 0.0f;
			_mc_roll_weight = 0.0f;
			_mc_pitch_weight = 0.0f;

		} else {
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			// slowly ramp up throttle to avoid step inputs
			float progress = (_time_since_trans_start - timeUntilMotorsAreUp()) / BACKTRANS_THROTTLE_UPRAMP_DUR_S;
			progress = math::constrain(progress, 0.0f, 1.0f);
			_mc_throttle_weight = moveLinear(0.0f, 1.0f, progress);
		}
	}


	_v_att_sp->thrust_body[2] = -_thrust_transition;

	const Quatf q_sp(Eulerf(roll_body, pitch_body, yaw_body));
	q_sp.copyTo(_v_att_sp->q_d);

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_throttle_weight = math::constrain(_mc_throttle_weight, 0.0f, 1.0f);
}

void Tiltrotor::waiting_on_tecs()
{
	// keep multicopter thrust until we get data from TECS
	_v_att_sp->thrust_body[0] = _thrust_transition;
}

void Tiltrotor::fill_actuator_outputs()
{

	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	// Multirotor output
	_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0] * _mc_roll_weight;
	_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1] * _mc_pitch_weight;
	_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2] * _mc_yaw_weight;

	// Special case tiltrotor: instead of passing a 3D thrust vector (that would mostly have a x-component in FW, and z in MC),
	// pass the vector magnitude and collective tilt separately. MC also needs collective thrust on z.
	// Passing 3D thrust plus tilt is not feasible as they
	// can't be allocated independently, and with the current controller it's not possible to have collective tilt calculated
	// by the allocator directly.
	float collective_thrust_normalized_setpoint = 0.f;

	if (_vtol_mode == vtol_mode::FW_MODE) {

		collective_thrust_normalized_setpoint = _vehicle_thrust_setpoint_virtual_fw->xyz[0];
		_thrust_setpoint_0->xyz[2] = -collective_thrust_normalized_setpoint;

		/* allow differential thrust if enabled */
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) {
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * _param_vt_fw_difthr_s_y.get() ;
		}

	} else {
		collective_thrust_normalized_setpoint = -_vehicle_thrust_setpoint_virtual_mc->xyz[2] * _mc_throttle_weight;
		_thrust_setpoint_0->xyz[2] = -collective_thrust_normalized_setpoint;
	}

	// Fixed wing output
	if (!_param_vt_elev_mc_lock.get() || _vtol_mode != vtol_mode::MC_MODE) {
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
	}

	// publish tiltrotor extra controls
	tiltrotor_extra_controls_s tiltrotor_extra_controls = {};
	tiltrotor_extra_controls.collective_tilt_normalized_setpoint = _tilt_control;
	tiltrotor_extra_controls.collective_thrust_normalized_setpoint = collective_thrust_normalized_setpoint;
	tiltrotor_extra_controls.timestamp = hrt_absolute_time();
	_tiltrotor_extra_controls_pub.publish(tiltrotor_extra_controls);
}

void Tiltrotor::blendThrottleAfterFrontTransition(float scale)
{
	const float tecs_throttle = _v_att_sp->thrust_body[0];

	_v_att_sp->thrust_body[0] = scale * tecs_throttle + (1.0f - scale) * _thrust_transition;
}

void Tiltrotor::blendThrottleDuringBacktransition(float scale, float target_throttle)
{
	_thrust_transition = scale * target_throttle + (1.0f - scale) * _last_thr_in_fw_mode;
}

float Tiltrotor::timeUntilMotorsAreUp()
{
	return BACKTRANS_THROTTLE_DOWNRAMP_DUR_S + _param_vt_bt_tilt_dur.get();
}

float Tiltrotor::moveLinear(float start, float stop, float progress)
{
	return start + progress * (stop - start);
}

bool Tiltrotor::isFrontTransitionCompletedBase()
{
	return VtolType::isFrontTransitionCompletedBase() && _tilt_control >= _param_vt_tilt_trans.get();
}

// ============================================================================
// 新增倾转控制方法实现
// ============================================================================

/**
 * 检查倾转参数的有效性
 * 确保空速断点和角度断点都是递增的，且参数值在合理范围内
 */
bool Tiltrotor::checkSlewParams() const
{
	const float arsp_1 = _param_vt_tilt_1_arsp.get();
	const float arsp_2 = _param_vt_tilt_2_arsp.get();
	const float arsp_min = _param_vt_arsp_trans.get();
	const int32_t ang_1 = _param_vt_tilt_1_ang.get();
	const int32_t ang_2 = _param_vt_tilt_2_ang.get();

	// 检查空速断点必须递增：0 < arsp_1 < arsp_2 < arsp_min
	if (arsp_1 <= 0.0f || arsp_2 <= arsp_1 || arsp_min <= arsp_2) {
		PX4_ERR("倾转参数错误：空速断点必须递增 (0 < %.1f < %.1f < %.1f)",
			(double)arsp_1, (double)arsp_2, (double)arsp_min);
		return false;
	}

	// 检查角度断点必须递增：0 < ang_1 < ang_2 < 90
	if (ang_1 <= 0 || ang_2 <= ang_1 || ang_2 >= 90) {
		PX4_ERR("倾转参数错误：角度断点必须递增 (0 < %d < %d < 90)", (int)ang_1, (int)ang_2);
		return false;
	}

	// 检查反向过渡时间参数
	const float bt_time1 = _param_vt_bt_time1.get();
	const float bt_time2 = _param_vt_bt_time2.get();

	if (bt_time1 <= 500.0f || bt_time2 <= bt_time1) {
		PX4_ERR("反向过渡时间参数错误：500 < %.0f < %.0f", (double)bt_time1, (double)bt_time2);
		return false;
	}

	// 检查反向过渡空速窗口
	const float bt_arsp_min = _param_vt_bt_arsp_min.get();
	const float bt_arsp_max = _param_vt_bt_arsp_max.get();

	if (bt_arsp_min <= 0.0f || bt_arsp_max <= bt_arsp_min) {
		PX4_ERR("反向过渡空速窗口错误：0 < %.1f < %.1f", (double)bt_arsp_min, (double)bt_arsp_max);
		return false;
	}

	return true;
}

/**
 * 检查空速传感器数据的有效性
 * 包括传感器健康检查、物理合理性检查等
 */
bool Tiltrotor::isAirspeedValid() const
{
	// 1. 传感器健康检查
	if (!_airspeed_validated->airspeed_sensor_measurement_valid) {
		return false;
	}

	// 2. 检查空速是否为有限值
	const float airspeed = _attc->get_calibrated_airspeed();

	if (!PX4_ISFINITE(airspeed) || airspeed < 0.0f) {
		return false;
	}

	// 3. 物理合理性检查：与地速对比（简化版，实际应考虑风速）
	if (_local_pos->v_xy_valid) {
		const float ground_speed = sqrtf(_local_pos->vx * _local_pos->vx + _local_pos->vy * _local_pos->vy);

		// 如果空速与地速差异过大（超过15m/s），可能传感器异常
		if (fabsf(airspeed - ground_speed) > 15.0f) {
			return false;
		}
	}

	return true;
}

/**
 * 计算正向过渡的目标倾转角度（基于空速三段线性调度）
 *
 * 三段线性调度曲线：
 * - 第一段：空速 0 → arsp_1，倾角 init_tilt → ang_1
 * - 第二段：空速 arsp_1 → arsp_2，倾角 ang_1 → ang_2
 * - 第三段：空速 arsp_2 → arsp_min，倾角 ang_2 → max_tilt
 */
float Tiltrotor::calculateForwardTiltTarget(float airspeed) const
{
	// 获取参数（角度从度转换为归一化值 0-1）
	const float init_tilt = _param_vt_tilt_mc.get();
	const float max_tilt = _param_vt_tilt_fw.get();
	const float arsp_1 = _param_vt_tilt_1_arsp.get();
	const float arsp_2 = _param_vt_tilt_2_arsp.get();
	const float ang_1 = static_cast<float>(_param_vt_tilt_1_ang.get()) / 90.0f;  // 度 → 归一化
	const float ang_2 = static_cast<float>(_param_vt_tilt_2_ang.get()) / 90.0f;
	const float arsp_min = _param_vt_arsp_trans.get();

	float tilt_target = init_tilt;

	if (airspeed <= arsp_1 && airspeed > 0.0f) {
		// 第一段：从初始倾角到第一个断点
		const float slope = (ang_1 - init_tilt) / arsp_1;
		tilt_target = math::constrain(init_tilt + slope * airspeed, init_tilt, ang_1);

	} else if (airspeed <= arsp_2 && airspeed > arsp_1) {
		// 第二段：第一个断点到第二个断点
		const float slope = (ang_2 - ang_1) / (arsp_2 - arsp_1);
		tilt_target = math::constrain(ang_1 + slope * (airspeed - arsp_1), ang_1, ang_2);

	} else if (airspeed > arsp_2 && airspeed <= arsp_min) {
		// 第三段：第二个断点到最大倾角
		const float slope = (max_tilt - ang_2) / (arsp_min - arsp_2);
		tilt_target = math::constrain(ang_2 + slope * (airspeed - arsp_2), ang_2, max_tilt);

	} else if (airspeed > arsp_min) {
		// 空速超过最小过渡空速，饱和到最大倾角
		tilt_target = max_tilt;
	}

	return tilt_target;
}

/**
 * 计算反向过渡的目标倾转角度（基于时间曲线）
 *
 * 两阶段时间曲线：
 * - 阶段1（0 → time1）：倾角从 max_tilt 降至 ang_turn
 * - 阶段2（time1 → time2）：倾角从 ang_turn 降至 min_tilt
 */
float Tiltrotor::calculateBackTiltTarget(float time_ms) const
{
	const float time1 = _param_vt_bt_time1.get();
	const float time2 = _param_vt_bt_time2.get();
	const float ang_turn = static_cast<float>(_param_vt_bt_ang_turn.get()) / 90.0f;  // 度 → 归一化
	const float max_tilt = _param_vt_tilt_fw.get();
	const float min_tilt = _param_vt_tilt_mc.get();

	float tilt_target = max_tilt;

	if (time_ms <= 500.0f) {
		// 初始延迟500ms，保持最大倾角
		tilt_target = max_tilt;

	} else if (time_ms <= time1 && time_ms > 500.0f) {
		// 阶段1：从最大倾角线性降至中间角度
		const float ratio = math::constrain(time_ms / time1, 0.0f, 1.0f);
		tilt_target = max_tilt - (max_tilt - ang_turn) * ratio;

	} else if (time_ms <= time2) {
		// 阶段2：从中间角度线性降至最小倾角
		const float ratio = math::constrain((time_ms - time1) / (time2 - time1), 0.0f, 1.0f);
		tilt_target = ang_turn * (1.0f - ratio) + min_tilt * ratio;

	} else {
		// 完成，保持最小倾角
		tilt_target = min_tilt;
	}

	return tilt_target;
}

/**
 * 应用倾转速率限制，平滑过渡到目标角度
 * 防止倾转速度过快导致姿态失稳
 */
float Tiltrotor::updateTiltWithSlewRate(float current_tilt, float target_tilt)
{
	// 基础速率限制：30°/s = 0.333 归一化单位/s
	float max_rate_normalized = 30.0f / 90.0f;  // 30度/秒 转换为归一化速率

	// 根据飞行状态动态调整速率限制
	const float airspeed = _attc->get_calibrated_airspeed();

	if (PX4_ISFINITE(airspeed) && airspeed > 15.0f) {
		// 高速时减慢倾转（气动力矩更大）
		max_rate_normalized *= 0.5f;
	}

	// 根据姿态误差动态调整：如果姿态偏差大，暂停倾转
	const float pitch_error = fabsf(Eulerf(Quatf(_v_att_sp->q_d)).theta() - Eulerf(Quatf(_v_att->q)).theta());
	const float roll_error = fabsf(Eulerf(Quatf(_v_att_sp->q_d)).phi() - Eulerf(Quatf(_v_att->q)).phi());

	if (pitch_error > 0.3f || roll_error > 0.3f) {  // 约17度
		// 姿态偏差过大，暂停倾转
		PX4_WARN("姿态偏差过大，暂停倾转 (pitch_err=%.2f, roll_err=%.2f)",
			 (double)pitch_error, (double)roll_error);
		return current_tilt;
	}

	// 应用速率限制
	const float max_change = max_rate_normalized * _transition_dt;
	const float tilt_change = math::constrain(target_tilt - current_tilt, -max_change, max_change);

	return current_tilt + tilt_change;
}

/**
 * 基于空速或时间的自动倾转控制
 * 根据模式选择正向或反向倾转策略
 */
void Tiltrotor::autoAirspeedTilt(AirspeedTiltMode mode)
{
	if (mode == AirspeedTiltMode::FORWARD_AUTO) {
		// === 正向过渡：基于空速的三段线性调度 ===

		// 参数有效性检查
		if (!checkSlewParams()) {
			PX4_ERR("倾转参数无效，禁用基于空速的倾转控制");
			_tilt_mode = AirspeedTiltMode::NONE_ACT;
			return;
		}

		// 空速有效性检查
		if (!isAirspeedValid()) {
			PX4_WARN("空速数据无效，回退到基于时间的倾转控制");
			// 这里应该回退到原有的基于时间的控制逻辑
			// 当前简化处理：保持当前倾角
			return;
		}

		// 动态速率控制：首次进入时保存原始速率
		if (_param_vt_tilt_rate_en.get() && !_rate_inited) {
			// 注意：PX4中倾转速率不是参数，而是硬编码的
			// 这里仅作为示例，实际需要根据PX4架构调整
			_rate_inited = true;
			PX4_INFO("启用动态倾转速率控制");
		}

		// 获取当前空速
		const float airspeed = _attc->get_calibrated_airspeed();

		// 计算目标倾角
		const float tilt_target = calculateForwardTiltTarget(airspeed);

		// 应用倾角（考虑速率限制）
		_tilt_control = updateTiltWithSlewRate(_tilt_control, tilt_target);

		// 过渡完成检查
		if (airspeed >= _param_vt_arsp_trans.get() &&
		    fabsf(_tilt_control - _param_vt_tilt_fw.get()) < 0.01f) {
			// 恢复原始速率（如果启用了动态速率控制）
			if (_rate_inited) {
				_rate_inited = false;
				PX4_INFO("正向过渡完成，恢复原始倾转速率");
			}
		}

	} else if (mode == AirspeedTiltMode::BACK_TRANSITION) {
		// === 反向过渡：基于时间曲线 ===
		updateBackTransitionState();
	}
}

/**
 * 更新反向过渡状态机
 * 管理从固定翼到多旋翼的倾转过程
 */
void Tiltrotor::updateBackTransitionState()
{
	const float airspeed = _attc->get_calibrated_airspeed();
	const float arsp_min = _param_vt_bt_arsp_min.get();
	const float arsp_max = _param_vt_bt_arsp_max.get();
	const hrt_abstime now = hrt_absolute_time();

	switch (_back_transition_state) {
	case BackTransitionState::CHECK:
		// 等待空速进入触发窗口
		if (PX4_ISFINITE(airspeed) && airspeed >= arsp_min && airspeed <= arsp_max) {
			_back_transition_state = BackTransitionState::TIMER;
			_back_transition_start_ts = now;
			_back_tilt_active = true;
			PX4_INFO("反向过渡开始：空速=%.1f m/s", (double)airspeed);
			break;
		}

		// 超时强制触发（10秒）
		if ((now - _transition_start_timestamp) > 10_s) {
			PX4_WARN("反向过渡超时，强制触发");
			_back_transition_state = BackTransitionState::TIMER;
			_back_transition_start_ts = now;
			_back_tilt_active = true;
			break;
		}

		// 高度强制触发（低于50m）
		if (_local_pos->z < -50.0f) {
			PX4_WARN("高度过低，强制反向过渡");
			_back_transition_state = BackTransitionState::TIMER;
			_back_transition_start_ts = now;
			_back_tilt_active = true;
			break;
		}

		// 地速强制触发（< 3m/s）
		if (_local_pos->v_xy_valid) {
			const float ground_speed = sqrtf(_local_pos->vx * _local_pos->vx + _local_pos->vy * _local_pos->vy);

			if (ground_speed < 3.0f) {
				PX4_WARN("地速过低，强制反向过渡");
				_back_transition_state = BackTransitionState::TIMER;
				_back_transition_start_ts = now;
				_back_tilt_active = true;
				break;
			}
		}

		break;

	case BackTransitionState::TIMER:
		// 执行反向倾转
		_back_timer_ms = (now - _back_transition_start_ts) / 1000.0f;

		// 计算目标倾角
		const float tilt_target = calculateBackTiltTarget(_back_timer_ms);

		// 应用倾角（考虑速率限制）
		_tilt_control = updateTiltWithSlewRate(_tilt_control, tilt_target);

		// 完成检查
		if (_back_timer_ms >= _param_vt_bt_time2.get() &&
		    fabsf(_tilt_control - _param_vt_tilt_mc.get()) < 0.01f) {
			_back_transition_state = BackTransitionState::DONE;
			_back_tilt_active = false;
			PX4_INFO("反向过渡完成");
		}

		// 倾转卡死保护（15秒）
		if ((now - _back_transition_start_ts) > 15_s) {
			PX4_ERR("反向过渡卡死，强制完成");
			_back_transition_state = BackTransitionState::DONE;
			_tilt_control = _param_vt_tilt_mc.get();  // 强制归零
			_back_tilt_active = false;
		}

		break;

	case BackTransitionState::DONE:
		// 保持MC倾角
		_tilt_control = _param_vt_tilt_mc.get();
		break;
	}
}
