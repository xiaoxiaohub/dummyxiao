# DummyXiaoX 319 机械臂 ROS2 工作空间

6 轴机械臂的 ROS2 控制系统，支持 MoveIt2 运动规划 + USB-CAN 总线驱动 CtrlStep 电机。

## 系统要求

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- python-can (`pip3 install python-can`)
- USB-CAN 适配器（支持 SocketCAN）

## 工作空间结构

```
ros2/src/
├── dummyxiaox319_description   # 机器人 URDF 模型和 mesh
├── dummyxiaox_moveit_config    # MoveIt2 运动规划配置
├── dummyxiaox_interface        # ROS2 服务接口定义
└── dummyxiaox_usb2can          # USB-CAN 电机驱动节点
```

## 编译

```bash
cd ~/dummyxiao319/ros2
colcon build
source install/setup.bash
```

> 每次打开新终端都需要执行 `source install/setup.bash`，或者将其加入 `~/.bashrc`。

## 使用方法

### 方式一：仅仿真（不连接真实机械臂）

```bash
ros2 launch dummyxiaox_moveit_config demo.launch.py
```

启动后 RViz 会打开，可以拖动交互球规划运动，点击 Plan & Execute 查看虚拟机械臂运动。此模式不需要 CAN 硬件。

### 方式二：控制真实机械臂（两个终端）

**终端 1** — 启动 MoveIt + RViz：

```bash
ros2 launch dummyxiaox_moveit_config demo.launch.py
```

**终端 2** — 配置 CAN 并启动电机驱动：

```bash
# 配置 CAN 总线（每次开机执行一次）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 启动 USB-CAN 驱动节点
source ~/dummyxiao319/ros2/install/setup.bash
ros2 launch dummyxiaox_usb2can usb2can.launch.py
```

然后在 RViz 中规划并执行运动，真实机械臂会同步运动。

### 方式三：一条命令启动全部（MoveIt + 电机驱动）

```bash
# 先配好 CAN
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 一键启动
ros2 launch dummyxiaox_usb2can real_robot.launch.py
```

## 电机配置

6 个关节对应的电机参数（已在代码中配置好）：

| 关节 | CAN 节点 ID | 减速比 | 方向反转 |
|------|------------|--------|---------|
| Joint1 | 1 | 30 | 是 |
| Joint2 | 2 | 30 | 否 |
| Joint3 | 3 | 30 | 是 |
| Joint4 | 4 | 24 | 否 |
| Joint5 | 5 | 30 | 是 |
| Joint6 | 6 | 50 | 是 |

## ROS2 服务接口

`usb2can_node` 提供三个服务，可用于脚本控制：

```bash
# 使能所有电机
ros2 service call /init_usb2can dummyxiaox_interface/srv/InitUsb2Can "{action: 'start'}"

# 失能所有电机
ros2 service call /init_usb2can dummyxiaox_interface/srv/InitUsb2Can "{action: 'stop'}"

# 直接发送关节角度（单位：度）
ros2 service call /write_usb2can dummyxiaox_interface/srv/WriteUsb2Can \
  "{pos_commands: [0.0, 0.0, 90.0, 0.0, 0.0, 0.0], vel_commands: [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]}"

# 读取当前关节角度
ros2 service call /read_usb2can dummyxiaox_interface/srv/ReadUsb2Can "{}"
```

## CAN 通信协议

采用 CtrlStep 电机协议，CAN ID 格式为 `(node_id << 7) | cmd`，波特率 1Mbit/s。

常用命令：

| 命令 | 代码 | 数据格式 | 说明 |
|------|------|---------|------|
| 使能 | 0x01 | uint32 (1/0) | 使能或失能电机 |
| 位置+速度限制 | 0x07 | float pos + float vel | 发送目标位置和速度限制 |
| 读取位置 | 0x23 | 请求为空，响应 float pos + byte finished | 获取当前位置 |

## 注意事项

- CAN 总线**必须**在启动 `usb2can_node` 之前配好（bitrate 1000000）
- 当前系统没有真实反馈闭环：RViz 显示的是规划位置，不是电机编码器实际位置
- 首次使用前请确保电机已校准 home 位置
- `python-can` 需要单独安装：`pip3 install python-can`
