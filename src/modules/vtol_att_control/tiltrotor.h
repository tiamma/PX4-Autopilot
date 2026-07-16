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
* @file tiltrotor.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
*
*/

#ifndef TILTROTOR_H
#define TILTROTOR_H
#include "vtol_type.h"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/tiltrotor_extra_controls.h>

/**
 * 倾转模式枚举 - 定义不同的倾转控制策略
 */
enum class AirspeedTiltMode : uint8_t {
	FORWARD_AUTO = 0,       // 正向自动倾转：基于空速三段线性调度
	BACK_TRANSITION = 1,    // 反向倾转：基于时间曲线从前进倾转回垂直
	NONE_ACT = 2            // 不执行倾转动作
};

/**
 * 反向过渡状态机 - 管理从固定翼到多旋翼的过渡过程
 */
enum class BackTransitionState : uint8_t {
	CHECK = 0,      // 等待空速进入反向过渡窗口
	TIMER = 1,      // 执行反向倾转，逐步恢复 VTOL 控制
	DONE = 2        // 倾转归零，恢复完全 VTOL 姿态控制
};

class Tiltrotor : public VtolType
{

public:

	Tiltrotor(VtolAttitudeControl *_att_controller);
	~Tiltrotor() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void fill_actuator_outputs() override;
	void update_mc_state() override;
	void update_fw_state() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;

private:
	enum class vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_FRONT_P2,	/**< vtol is in front transition part 2 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	/**
	 * Specific to tiltrotor with vertical aligned rear engine/s.
	 * These engines need to be shut down in fw mode. During the back-transition
	 * they need to idle otherwise they need too much time to spin up for mc mode.
	 */

	vtol_mode _vtol_mode{vtol_mode::MC_MODE};			/**< vtol flight mode, defined by enum vtol_mode */

	uORB::Publication<tiltrotor_extra_controls_s>	_tiltrotor_extra_controls_pub{ORB_ID(tiltrotor_extra_controls)};

	float _tilt_control{0.0f};		/**< actuator value for the tilt servo */

	// === 正向过渡倾转控制状态 ===
	AirspeedTiltMode _tilt_mode{AirspeedTiltMode::NONE_ACT};  /**< 当前倾转模式 */

	// 动态速率控制状态变量
	bool _rate_inited{false};           /**< 是否已保存原始倾转速率 */
	float _rate_up_orig{0.0f};          /**< 保存的原始上升速率 */
	float _rate_dn_orig{0.0f};          /**< 保存的原始下降速率 */

	// === 反向过渡倾转控制状态 ===
	BackTransitionState _back_transition_state{BackTransitionState::CHECK};  /**< 反向过渡状态机当前状态 */
	hrt_abstime _back_transition_start_ts{0};  /**< 反向过渡开始时间戳 */
	float _back_timer_ms{0.0f};         /**< 反向过渡计时器（毫秒） */
	bool _back_tilt_active{false};      /**< 反向倾转是否激活 */
	float _last_tilt_control{0.0f};     /**< 上一次的倾转角度（用于速率限制） */

	void parameters_update() override;
	float timeUntilMotorsAreUp();
	float moveLinear(float start, float stop, float progress);

	void blendThrottleDuringBacktransition(const float scale, const float target_throttle);
	bool isFrontTransitionCompletedBase() override;

	// === 新增倾转控制方法 ===

	/**
	 * @brief 基于空速或时间的自动倾转控制
	 * @param mode 倾转模式（正向/反向/无动作）
	 */
	void autoAirspeedTilt(AirspeedTiltMode mode);

	/**
	 * @brief 反向过渡倾转控制（从固定翼到多旋翼）
	 */
	void backAirspeedTilt();

	/**
	 * @brief 检查倾转参数的有效性
	 * @return true 参数有效，false 参数无效
	 */
	bool checkSlewParams() const;

	/**
	 * @brief 计算正向过渡的目标倾转角度（基于空速三段线性调度）
	 * @param airspeed 当前校准空速 (m/s)
	 * @return 目标倾转角度（归一化 0-1）
	 */
	float calculateForwardTiltTarget(float airspeed) const;

	/**
	 * @brief 计算反向过渡的目标倾转角度（基于时间曲线）
	 * @param time_ms 反向过渡开始后的时间 (ms)
	 * @return 目标倾转角度（归一化 0-1）
	 */
	float calculateBackTiltTarget(float time_ms) const;

	/**
	 * @brief 更新反向过渡状态机
	 */
	void updateBackTransitionState();

	/**
	 * @brief 应用倾转速率限制，平滑过渡到目标角度
	 * @param current_tilt 当前倾转角度（归一化 0-1）
	 * @param target_tilt 目标倾转角度（归一化 0-1）
	 * @return 应用速率限制后的倾转角度
	 */
	float updateTiltWithSlewRate(float current_tilt, float target_tilt);

	/**
	 * @brief 检查空速传感器数据的有效性
	 * @return true 空速数据有效，false 空速数据无效
	 */
	bool isAirspeedValid() const;


	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					// === 现有参数 ===
					(ParamFloat<px4::params::VT_TILT_MC>) _param_vt_tilt_mc,
					(ParamFloat<px4::params::VT_TILT_TRANS>) _param_vt_tilt_trans,
					(ParamFloat<px4::params::VT_TILT_FW>) _param_vt_tilt_fw,
					(ParamFloat<px4::params::VT_TRANS_P2_DUR>) _param_vt_trans_p2_dur,
					(ParamFloat<px4::params::VT_BT_TILT_DUR>) _param_vt_bt_tilt_dur,

					// === 正向过渡新增参数 ===
					(ParamFloat<px4::params::VT_TILT_SLOW_R>) _param_vt_tilt_slow_r,     // 动态速率控制的上升速率倍率
					(ParamFloat<px4::params::VT_TILT_1_ARSP>) _param_vt_tilt_1_arsp,     // 第一段断点空速 (m/s)
					(ParamInt<px4::params::VT_TILT_1_ANG>) _param_vt_tilt_1_ang,         // 第一段断点角度 (deg)
					(ParamFloat<px4::params::VT_TILT_2_ARSP>) _param_vt_tilt_2_arsp,     // 第二段断点空速 (m/s)
					(ParamInt<px4::params::VT_TILT_2_ANG>) _param_vt_tilt_2_ang,         // 第二段断点角度 (deg)
					(ParamBool<px4::params::VT_TILT_RATE_EN>) _param_vt_tilt_rate_en,    // 动态速率控制使能

					// === 反向过渡新增参数 ===
					(ParamFloat<px4::params::VT_BT_ARSP_TURN>) _param_vt_bt_arsp_turn,   // 反向过渡空速转折点 (m/s)
					(ParamFloat<px4::params::VT_BT_ARSP_MAX>) _param_vt_bt_arsp_max,     // 反向过渡最大空速阈值 (m/s)
					(ParamFloat<px4::params::VT_BT_ARSP_MIN>) _param_vt_bt_arsp_min,     // 反向过渡最小空速阈值 (m/s)
					(ParamInt<px4::params::VT_BT_ANG_TURN>) _param_vt_bt_ang_turn,       // 反向过渡中间角度 (deg)
					(ParamFloat<px4::params::VT_BT_RATE_K>) _param_vt_bt_rate_k,         // 反向倾转 slew 速率倍率
					(ParamFloat<px4::params::VT_BT_TIME1>) _param_vt_bt_time1,           // 反向过渡第一阶段时长 (ms)
					(ParamFloat<px4::params::VT_BT_TIME2>) _param_vt_bt_time2            // 反向过渡第二阶段时长 (ms)
				       )

};
#endif
