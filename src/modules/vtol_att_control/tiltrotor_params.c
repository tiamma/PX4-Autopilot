/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file tiltrotor_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <roman@px4.io>
 */

/**
 * Normalized tilt in Hover
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_MC, 0.0f);

/**
 * Normalized tilt in transition to FW
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_TRANS, 0.4f);

/**
 * Normalized tilt in FW
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_FW, 1.0f);

/**
 * Duration of front transition phase 2
 *
 * Time in seconds it takes to tilt form VT_TILT_TRANS to VT_TILT_FW.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_P2_DUR, 0.5f);

/**
 * Duration motor tilt up in backtransition
 *
 * Time in seconds it takes to tilt form VT_TILT_FW to VT_TILT_MC.
 *
 * @unit s
 * @min 0.1
 * @max 10
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BT_TILT_DUR, 1.f);

// ============================================================================
// 正向过渡新增参数（基于空速的三段线性倾转调度）
// ============================================================================

/**
 * 正向过渡倾转速率缩放因子
 *
 * 正向过渡期间倾转上升速率的倍率。较小的值会减慢倾转速度，
 * 提供更平滑的过渡。仅在启用 VT_TILT_RATE_EN 时生效。
 *
 * @min 0.1
 * @max 1.0
 * @increment 0.05
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_SLOW_R, 0.5f);

/**
 * 倾转调度第一断点空速
 *
 * 正向过渡时，第一个倾转角度断点对应的空速值。
 * 空速从 0 到此值时，倾转角度从 VT_TILT_MC 线性增加到 VT_TILT_1_ANG。
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @increment 0.5
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_1_ARSP, 5.0f);

/**
 * 倾转调度第一断点角度
 *
 * 正向过渡时，第一个空速断点对应的倾转角度（度）。
 * 0度=多旋翼模式，90度=固定翼模式。
 *
 * @unit deg
 * @min 0
 * @max 90
 * @increment 5
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_TILT_1_ANG, 30);

/**
 * 倾转调度第二断点空速
 *
 * 正向过渡时，第二个倾转角度断点对应的空速值。
 * 空速从 VT_TILT_1_ARSP 到此值时，倾转角度从 VT_TILT_1_ANG
 * 线性增加到 VT_TILT_2_ANG。
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @increment 0.5
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_2_ARSP, 10.0f);

/**
 * 倾转调度第二断点角度
 *
 * 正向过渡时，第二个空速断点对应的倾转角度（度）。
 * 必须大于 VT_TILT_1_ANG 且小于 90 度。
 *
 * @unit deg
 * @min 0
 * @max 90
 * @increment 5
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_TILT_2_ANG, 60);

/**
 * 启用动态倾转速率控制
 *
 * 如果启用，正向过渡期间倾转速率会动态调整：
 * 上升速率减慢（乘以 VT_TILT_SLOW_R），下降速率加快（乘以 2.0）。
 * 过渡完成后恢复原始速率。
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_TILT_RATE_EN, 0);

// ============================================================================
// 反向过渡新增参数（基于时间曲线的倾转控制）
// ============================================================================

/**
 * 反向过渡空速转折点
 *
 * 反向过渡时的参考空速转折点（当前未使用，预留）。
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @increment 0.5
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BT_ARSP_TURN, 8.0f);

/**
 * 反向过渡最大空速阈值
 *
 * 反向过渡触发窗口的最大空速。只有当空速在
 * VT_BT_ARSP_MIN 和 VT_BT_ARSP_MAX 之间时，
 * 才会触发反向倾转。
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @increment 0.5
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BT_ARSP_MAX, 15.0f);

/**
 * 反向过渡最小空速阈值
 *
 * 反向过渡触发窗口的最小空速。空速低于此值时
 * 也会强制触发反向过渡（安全保护）。
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @increment 0.5
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BT_ARSP_MIN, 5.0f);

/**
 * 反向过渡中间倾转角度
 *
 * 反向过渡第一阶段结束时的倾转角度（度）。
 * 倾转角度会先从 90度（固定翼）降至此值，
 * 然后再降至 0度（多旋翼）。
 *
 * @unit deg
 * @min 0
 * @max 90
 * @increment 5
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_BT_ANG_TURN, 45);

/**
 * 反向倾转速率倍率
 *
 * 反向过渡时倾转速率的缩放因子。
 * 较大的值会加快反向倾转速度（当前未使用，预留）。
 *
 * @min 0.5
 * @max 3.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BT_RATE_K, 1.5f);

/**
 * 反向过渡第一阶段时长
 *
 * 反向过渡第一阶段的持续时间（毫秒）。
 * 在此阶段，倾转角度从 90度 降至 VT_BT_ANG_TURN。
 * 必须大于 500ms。
 *
 * @unit ms
 * @min 500.0
 * @max 10000.0
 * @increment 100.0
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BT_TIME1, 2000.0f);

/**
 * 反向过渡第二阶段时长
 *
 * 反向过渡第二阶段的持续时间（毫秒）。
 * 在此阶段，倾转角度从 VT_BT_ANG_TURN 降至 0度。
 * 必须大于 VT_BT_TIME1。
 *
 * @unit ms
 * @min 1000.0
 * @max 15000.0
 * @increment 100.0
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BT_TIME2, 4000.0f);
