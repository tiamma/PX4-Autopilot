# VTOL 风向标功能

_风向标（weather vane）_ 功能会在悬停飞行期间自动转动 VTOL 飞行器，使机头朝向相对来风。
这可以提高稳定性（降低侧风抬起迎风侧机翼并使飞行器翻转的风险）。

在多旋翼模式下飞行的 VTOL 混合构型飞行器会[默认启用](#configuration)此功能。

::: info
纯多旋翼飞行器不支持风向标功能。
:::

## 手动模式下的行为

风向标功能仅在[位置模式](../flight_modes_mc/position.md)下生效（其他手动多旋翼模式下不生效）。

即使风向标控制器正尝试将机头转向来风方向，用户仍可通过偏航摇杆给出偏航速率指令。
目标偏航速率为风向标偏航速率与用户指令偏航速率之和。

## 任务模式下的行为

在[任务模式](../flight_modes_vtol/mission.md)下，只要参数已启用，风向标功能就会始终处于活动状态。
任务中指定的任何偏航角指令都将被忽略。

<a id="configuration"></a>

## 配置

该功能使用 [WV\_\* 参数](../advanced_config/parameter_reference.md#WV_EN)进行配置。

| 参数 | 说明 |
| --- | --- |
| [WV_EN](../advanced_config/parameter_reference.md#WV_EN) | 启用风向标功能。 |
| [WV_ROLL_MIN](../advanced_config/parameter_reference.md#WV_ROLL_MIN) | 风向标控制器开始要求偏航速率时的最小横滚角设定值。 |
| [WV_YRATE_MAX](../advanced_config/parameter_reference.md#WV_YRATE_MAX) | 风向标控制器允许要求的最大偏航速率。 |

## 工作原理

悬停飞行时，飞行器需要克服风产生的阻力才能保持位置。
实现这一点的唯一方法，是使推力矢量向相对来风方向倾斜（也就是让飞行器“顶风倾斜”）。
通过持续跟踪推力矢量，可以估算风向。
风向标控制器据此发出偏航速率指令，使飞行器机头转向估算出的来风方向。
