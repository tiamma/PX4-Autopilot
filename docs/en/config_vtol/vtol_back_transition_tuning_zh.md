# VTOL 后向转换调参

VTOL 执行后向转换（从固定翼模式转换到多旋翼模式）时，需要先降低速度，多旋翼才能正常接管控制。
为帮助减速，如果当前减速度低于预期减速度（[VT_B_DEC_MSS](../advanced_config/parameter_reference.md#VT_B_DEC_MSS)）设定值，控制器会使飞行器抬头。
该减速度控制器的响应可通过积分增益 `I` 进行调整：[VT_B_DEC_I](../advanced_config/parameter_reference.md#VT_B_DEC_I)。
增大 `I` 会使飞行器更积极地抬头，以达到设定的减速度。

当水平速度降至多旋翼巡航速度（[MPC_XY_CRUISE](../advanced_config/parameter_reference.md#MPC_XY_CRUISE)），或后向转换持续时间（[VT_B_TRANS_DUR](../advanced_config/parameter_reference.md#VT_B_TRANS_DUR)）结束时（以先发生者为准），飞行器将认为后向转换完成。

## 设置预期减速度

执行包含 [VTOL_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND) 航点的任务时，自动驾驶仪会尝试计算开始后向转换的适当距离。计算依据是当前速度（近似于地速）和预期减速度。
要使飞行器在非常接近着陆点的位置完成后向转换，可以调整预期减速度参数（[VT_B_DEC_MSS](../advanced_config/parameter_reference.md#VT_B_DEC_MSS)）。
请确保后向转换持续时间足够长，使飞行器能够在超时生效前到达预定位置。

## 后向转换持续时间

设置较长的后向转换时间（[VT_B_TRANS_DUR](../advanced_config/parameter_reference.md#VT_B_TRANS_DUR)），可让飞行器有更多时间减速。
在此期间，VTOL 会关闭固定翼电机，并在滑翔过程中缓慢提升多旋翼（MC）电机的输出。
时间设置得越长，飞行器为减速而滑翔的时间就越长。需要注意的是，在此期间飞行器只控制高度而不控制位置，因此可能发生一定程度的漂移。
