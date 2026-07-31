# CAN Angle Control 接口协议与 Python 调试指南

`can_angle_control` 驱动通过 NuttX socketCAN 与一个外部执行机构通信，实现单轴角度控制。

- 命令帧：飞控 → 执行机构
- 反馈帧：执行机构 → 飞控

---

## 1. 默认参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CA_CTRL_ENABLE` | `0` | 是否启用驱动 |
| `CA_CTRL_DEV` | `0` | CAN 接口：`0=can0/CAN1`，`1=can1/CAN2` |
| `CA_CTRL_TX_ID` | `512` | 命令帧 ID，`0x200` |
| `CA_CTRL_RX_ID` | `513` | 反馈帧 ID，`0x201` |
| `CA_CTRL_BITRATE` | `1000000` | 波特率，1 Mbps |
| `CA_CTRL_MIN` | `-90.0` | 最小角度 ° |
| `CA_CTRL_MAX` | `90.0` | 最大角度 ° |
| `CA_CTRL_RATE` | `360.0` | 最大变化速率 °/s |
| `CA_CTRL_TIMEOUT` | `200.0` | 超时 ms |
| `CA_CTRL_OFFSET` | `0.0` | 零偏 ° |
| `CA_CTRL_DIR` | `0` | 方向反转 |

---

## 2. 命令帧格式（飞控 → 执行机构）

**ID**：`CA_CTRL_TX_ID`（默认 `0x200`）
**类型**：标准帧，DLC = 8

| 字节 | 字段 | 说明 |
|------|------|------|
| D0 | 角度低字节 | 有符号 int16，单位 0.01° |
| D1 | 角度高字节 | 例：`0x01 0xF4` = 500 → 5.00° |
| D2 | 使能标志 | `0x01` 启用，`0x00` 禁用 |
| D3 | 帧计数器 | 每次发送递增 |
| D4 | 保留 | 0x00 |
| D5 | 保留 | 0x00 |
| D6 | 校验和 | `(D0 + D1 + D2 + D3 + D4 + D5) & 0xFF` |
| D7 | 保留 | 0x00 |

### 命令帧示例

目标角度 **5.00°**、启用、计数器 `0x6F`：

```text
D0 D1 D2 D3 D4 D5 D6 D7
F4 01 01 6F 00 00 65 00
```

校验和：

```text
0xF4 + 0x01 + 0x01 + 0x6F + 0x00 + 0x00 = 0x165
低 8 位 = 0x65
```

---

## 3. 反馈帧格式（执行机构 → 飞控）

**ID**：`CA_CTRL_RX_ID`（默认 `0x201`）
**类型**：标准帧，DLC = 8

| 字节 | 字段 | 说明 |
|------|------|------|
| D0 | 角度低字节 | 有符号 int16，单位 0.01° |
| D1 | 角度高字节 | 实际当前角度 |
| D2 | 保留 | 一般 0x00 |
| D3 | 计数器 | 执行机构可自由填充 |
| D4 | 错误码低字节 | 设备错误码 |
| D5 | 错误码高字节 | 设备错误码 |
| D6 | 校验和 | `(D0 + D1 + D2 + D3 + D4 + D5) & 0xFF` |
| D7 | 保留 | 0x00 |

### 反馈帧示例

实际角度 **0.00°**、计数器 `0x7A`、无错误：

```text
00 00 00 7A 00 00 7A 00
```

---

## 4. uORB 话题

### `can_angle_command`（飞控内部 → 驱动）

| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | uint64 | 时间戳 |
| `angle_setpoint` | float32 | 目标角度，**弧度** |
| `enable` | bool | 是否启用 |

### `can_angle_status`（驱动 → 飞控内部）

| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | uint64 | 时间戳 |
| `angle_setpoint` | float32 | 当前下发的目标角度，弧度 |
| `angle` | float32 | 从反馈帧读回的实际角度，弧度 |
| `enabled` | bool | 是否已使能 |
| `communication_ok` | bool | 通信是否正常 |
| `device_error` | uint16 | 设备错误码 |
| `rx_count` | uint32 | 接收帧计数 |
| `tx_count` | uint32 | 发送帧计数 |
| `error_count` | uint32 | 错误计数 |

---

## 5. 飞控端常用命令

```bash
# 设置参数
param set CA_CTRL_ENABLE 1
param set CA_CTRL_DEV 1        # 使用 can1 / CAN2 口
param set CA_CTRL_TX_ID 512
param set CA_CTRL_RX_ID 513
param save
reboot

# 启动/查看驱动
nsh> can_angle_control start
nsh> can_angle_control status

# 查看 uORB 数据
nsh> listener can_angle_status
nsh> listener can_angle_command

# 查看 CAN 接口
nsh> ifconfig
nsh> ifup can1
```

---

## 6. Python 调试

### 6.1 安装依赖

```bash
pip install python-can pyserial
```

### 6.2 直接通过串口（SLCAN）收发

如果你的 USB-CAN 适配器显示为 `/dev/ttyACM1`：

```python
import serial
import time

def send_frame(ser, can_id, data):
    """发送 SLCAN 标准帧。"""
    assert len(data) == 8
    dlc = len(data)
    hex_data = ''.join(f'{b:02X}' for b in data)
    msg = f't{can_id:03X}{dlc:X}{hex_data}\r'
    ser.write(msg.encode('ascii'))
    print('TX:', msg.strip())

def checksum(data):
    return sum(data[:6]) & 0xFF

def make_cmd(angle_deg, enable=True, counter=0):
    raw = int(round(angle_deg * 100))
    d0 = raw & 0xFF
    d1 = (raw >> 8) & 0xFF
    d2 = 1 if enable else 0
    d3 = counter & 0xFF
    d4, d5 = 0, 0
    d6 = checksum([d0, d1, d2, d3, d4, d5])
    d7 = 0
    return [d0, d1, d2, d3, d4, d5, d6, d7]

def make_feedback(angle_deg, error=0, counter=0):
    raw = int(round(angle_deg * 100))
    d0 = raw & 0xFF
    d1 = (raw >> 8) & 0xFF
    d2 = 0
    d3 = counter & 0xFF
    d4 = error & 0xFF
    d5 = (error >> 8) & 0xFF
    d6 = checksum([d0, d1, d2, d3, d4, d5])
    d7 = 0
    return [d0, d1, d2, d3, d4, d5, d6, d7]

ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)

# 1. 发送命令帧：5.00°，启用
counter = 0
send_frame(ser, 0x200, make_cmd(5.0, enable=True, counter=counter))

# 2. 发送反馈帧：0.00°
send_frame(ser, 0x201, make_feedback(0.0, counter=0))

# 3. 读取回显
ser.flushInput()
print('RX:', ser.read_all().decode('ascii', errors='ignore'))

ser.close()
```

### 6.3 使用 python-can（socketcan 方式）

先用 `slcand` 把 `/dev/ttyACM1` 映射成 `slcan0`：

```bash
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyACM1 slcan0
sudo ip link set slcan0 up type can bitrate 1000000
```

然后用 Python：

```python
import can
import time

bus = can.interface.Bus('slcan0', bustype='socketcan', bitrate=1000000)

def checksum(data):
    return sum(data[:6]) & 0xFF

def make_cmd(angle_deg, enable=True, counter=0):
    raw = int(round(angle_deg * 100))
    data = [raw & 0xFF, (raw >> 8) & 0xFF,
            1 if enable else 0, counter & 0xFF,
            0, 0, 0, 0]
    data[6] = checksum(data)
    return data

# 发送命令帧 5.00°
msg = can.Message(arbitration_id=0x200, data=make_cmd(5.0, counter=0), is_extended_id=False)
bus.send(msg)

# 监听反馈帧
for msg in bus:
    if msg.arbitration_id == 0x201:
        angle = (msg.data[0] | (msg.data[1] << 8))
        if angle > 32767:
            angle -= 65536
        print(f'Feedback: {angle / 100.0:.2f}°')
```

---

## 7. 常见问题

### 7.1 `can_angle_control status` 显示 `not running`

- 检查 `CA_CTRL_ENABLE` 是否为 `1`
- 检查 `CA_CTRL_DEV` 对应的 CAN 接口是否存在：`ifconfig`
- 检查 `dmesg` 是否有 `Failed to get CAN interface index` 报错

### 7.2 `Failed to get CAN interface index for can1`

说明当前固件没有启用 FDCAN2。需要在板级 `defconfig` 中加上：

```text
CONFIG_STM32H7_FDCAN2=y
```

然后重新编译并烧录。

### 7.3 `candump` 没有数据

- 确认 PC 端 `slcan0` 已 up：`ip link show slcan0`
- 确认飞控端 `can_angle_control` 已启动且运行在多旋翼模式（命令帧由 `update_mc_state()` 发布）
- 确认硬件接线、终端电阻、共地

### 7.4 校验和错误

- 角度字段按有符号 int16、小端、单位 0.01° 计算
- 校验和只累加 D0~D5，取低 8 位

---

## 8. 参考换算

| 角度 | 原始值 | D0 | D1 |
|------|--------|----|----|
| 0° | 0 | `00` | `00` |
| 1° | 100 | `64` | `00` |
| 5° | 500 | `F4` | `01` |
| 15° | 1500 | `DC` | `05` |
| -1° | -100 | `9C` | `FF` |

```python
angle_deg = 5.0
raw = int(round(angle_deg * 100))
d0 = raw & 0xFF
d1 = (raw >> 8) & 0xFF
```



RROR [can_angle_control] Failed to get CAN interface index for can1
RROR [can_angle_control] Failed to get CAN interface index for can1
