# 无空速传感器的 VTOL

<Badge type="warning" text="实验性功能" />

:::warning
对无空速传感器 VTOL 的支持仍属于实验性功能，仅应由经验丰富的飞手尝试。
建议使用空速传感器。
:::

固定翼飞行器使用[空速传感器](../sensor/airspeed.md)确定飞机相对于空气的运动速度。
受风影响，空速可能与地速不同。
每架飞机都有一个最低空速，低于该速度时飞机将会失速。
在天气温和，并且参数设置所对应的速度明显高于失速速度时，VTOL 可以不使用空速传感器运行。

本指南将介绍绕过 VTOL 飞机空速传感器所需的参数设置。

::: info
此处介绍的大多数设置也应适用于非 VTOL 固定翼飞行器，但目前尚未经过测试。
转换转弯和四旋翼紧急接管（Quad-chute）为 VTOL 专用功能。
:::

## 准备工作

尝试取消空速传感器之前，应先确定一个安全的油门水平。
同时还需要确定前向转换的持续时间。
为此，可以使用空速传感器执行一次参考飞行，也可以手动驾驶飞行器。
无论采用哪种方式，参考飞行都应在风力非常小的条件下进行。

飞行速度应足以应对大风条件，飞行过程应包括：

- 成功完成前向转换
- 直线平飞
- 大幅度转弯
- 快速爬升到更高高度

## 检查日志

参考飞行结束后，下载日志并使用 [FlightPlot](../log/flight_log_analysis.md#flightplot)（或其他分析工具）进行检查。
绘制高度（`GPOS.Alt`）、推力（`ATC1.Thrust`）、地速（表达式：`sqrt(GPS.VelN\^2 + GPS.VelE\^2)`）、俯仰角（`ATT.Pitch`）和横滚角（`AT.Roll`）。

检查飞行器平飞（俯仰和横滚为零或很小）、爬升（高度增加）以及倾斜转弯（横滚角较大）时的油门水平（推力）。
初始巡航油门应采用横滚或爬升期间施加的最大推力；如果之后决定进一步降低速度，平飞时的推力应作为最小值参考。

还要记录完成前向转换所用的时间。
该时间将用于设置最小转换时间。
出于安全考虑，应在此时间基础上增加约 30%。

最后，记录巡航飞行时的地速。
首次无空速传感器飞行后，可以使用该地速调整油门设置。

## 参数设置

要绕过飞行前空速检查，需要将 [SYS_HAS_NUM_ASPD](../advanced_config/parameter_reference.md#SYS_HAS_NUM_ASPD) 设置为 0。

要防止已安装的空速传感器用于反馈控制，请将 [FW_USE_AIRSPD](../advanced_config/parameter_reference.md#FW_USE_AIRSPD) 设置为 `False`。
这样可以在无空速反馈的设置下测试系统行为，同时仍保留实际空速读数，以便检查相对于失速速度的安全裕度等。

将配平油门（[FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM)）设置为根据参考飞行日志确定的百分比。
请注意，QGC 中该值的范围为 `1..100`，而日志中的推力值范围为 `0..1`。
因此，日志中的推力值 0.65 应输入为 65。
出于安全考虑，首次测试飞行时建议在确定值的基础上增加约 10% 的油门。

将最小前向转换时间（[VT_TRANS_MIN_TM](../advanced_config/parameter_reference.md#VT_TRANS_MIN_TM)）设置为参考飞行确定的秒数，并为安全起见增加约 30%。

### 建议设置的可选参数

由于确实存在失速风险，建议设置“固定翼最低高度”（也称为“Quad-chute”）阈值（[VT_FW_MIN_ALT](../advanced_config/parameter_reference.md#VT_FW_MIN_ALT)）。

低于特定高度时，这会使 VTOL 转换回多旋翼模式并启动[返航模式](../flight_modes_vtol/return.md)。
可以将其设置为 15 或 20 米，以便多旋翼有时间从失速中恢复。

此模式已测试的位置估计器为 EKF2，默认处于启用状态（更多信息请参阅[切换状态估计器](../advanced/switching_state_estimators.md#how-to-enable-different-estimators)和 [EKF2_EN](../advanced_config/parameter_reference.md#EKF2_EN)）。

## 首次无空速传感器飞行

这些参数值适用于位置控制飞行（例如[定点盘旋模式](../flight_modes_fw/hold.md)、[任务模式](../flight_modes_vtol/mission.md)）。
因此，建议在安全高度配置任务，该高度应比 Quad-chute 阈值高约 10 米。

与参考飞行一样，本次飞行也应在风力非常小的条件下进行。
首次飞行建议如下：

- 保持在同一高度
- 航点间距应足够大，并合理布置，避免急转弯
- 任务范围应足够小，确保需要手动接管时飞行器仍在视线范围内
- 如果空速非常高，可切换到高度模式，手动执行后向转换

如果任务成功完成，应继续检查日志中的以下内容：

- 地速应明显高于参考飞行时的地速。
- 高度不应明显低于参考飞行时的高度。
- 俯仰角不应持续偏离参考飞行时的俯仰角。

如果满足所有这些条件，可以开始小幅逐步降低巡航油门，直到地速与参考飞行时一致。

## 参数概览

相关参数如下：

- [FW_USE_AIRSPD](../advanced_config/parameter_reference.md#FW_USE_AIRSPD)
- [SYS_HAS_NUM_ASPD](../advanced_config/parameter_reference.md#SYS_HAS_NUM_ASPD)
- [EKF2_EN](../advanced_config/parameter_reference.md#EKF2_EN) (1)、[ATT_EN](../advanced_config/parameter_reference.md#ATT_EN) (0)、[LPE_EN](../advanced_config/parameter_reference.md#LPE_EN) (0)
- [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM)：根据飞行确定（例如 70%）
- [VT_TRANS_MIN_TM](../advanced_config/parameter_reference.md#VT_TRANS_MIN_TM)：根据飞行确定（例如 10 秒）
- [VT_FW_MIN_ALT](../advanced_config/parameter_reference.md#VT_FW_MIN_ALT)：15
