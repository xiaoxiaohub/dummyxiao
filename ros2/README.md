# DummyXiaoX 319 机械臂 ROS2 工作空间

6 轴机械臂的 ROS2 控制系统，支持 MoveIt2 运动规划 + USB 串口驱动 CtrlStep 电机。

## 系统架构

```
PC (ROS2/MoveIt) ──USB CDC串口──> REF Core主控板 (STM32F405) ──CAN总线──> CtrlStep电机 x6
```

REF Core 主控板通过 USB CDC 串口（`/dev/ttyACM0`）接收 PC 的 ASCII 命令，内部通过 CAN 总线控制 6 个 CtrlStep 电机。

## 系统要求

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- pyserial (`pip3 install pyserial`)
- REF Core 主控板（通过 USB 连接，显示为 `/dev/ttyACM0`）

## 工作空间结构

```
ros2/src/
├── dummyxiaox319_description   # 机器人 URDF 模型和 mesh
├── dummyxiaox_moveit_config    # MoveIt2 运动规划配置
├── dummyxiaox_interface        # ROS2 服务接口定义
└── dummyxiaox_usb2can          # USB 串口电机驱动节点
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

启动后 RViz 会打开，可以拖动交互球规划运动，点击 Plan & Execute 查看虚拟机械臂运动。此模式不需要硬件。

### 方式二：控制真实机械臂（两个终端）

**终端 1** — 启动 MoveIt + RViz：

```bash
ros2 launch dummyxiaox_moveit_config demo.launch.py
```

**终端 2** — 启动电机驱动（确保主控板已通过 USB 连接）：

```bash
source ~/dummyxiao319/ros2/install/setup.bash
ros2 launch dummyxiaox_usb2can usb2can.launch.py
```

然后在 RViz 中规划并执行运动，真实机械臂会同步运动。

### 方式三：一条命令启动全部（MoveIt + 电机驱动）

```bash
ros2 launch dummyxiaox_usb2can real_robot.launch.py
```

## 串口通信协议

通过 USB CDC 串口与 REF Core 主控板通信，使用 ASCII 文本命令：

| 命令 | 格式 | 说明 |
|------|------|------|
| 使能 | `!START` | 使能所有电机 |
| 急停 | `!STOP` | 紧急停止 |
| 失能 | `!DISABLE` | 关闭所有电机 |
| 回零 | `!HOME` | 回到零位 |
| 关节运动 | `>j1,j2,j3,j4,j5,j6[,speed]` | 发送6轴角度（度）+ 可选速度(0-100) |
| 读取位置 | `#GETJPOS` | 返回 `ok j1 j2 j3 j4 j5 j6` |
| 命令模式 | `#CMDMODE <mode>` | 1=顺序 2=可中断 3=轨迹 |

## ROS2 服务接口

`usb2can_node` 提供三个服务，可用于脚本控制：

```bash
# 使能所有电机
ros2 service call /init_usb2can dummyxiaox_interface/srv/InitUsb2Can "{action: 'start'}"

# 失能所有电机
ros2 service call /init_usb2can dummyxiaox_interface/srv/InitUsb2Can "{action: 'stop'}"

# 直接发送关节角度（单位：度），速度通过 vel_commands[0] 设置 (0-100)
ros2 service call /write_usb2can dummyxiaox_interface/srv/WriteUsb2Can \
  "{pos_commands: [0.0, 0.0, 90.0, 0.0, 0.0, 0.0], vel_commands: [30.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 读取当前关节角度
ros2 service call /read_usb2can dummyxiaox_interface/srv/ReadUsb2Can "{}"
```

## 注意事项

- 确保 REF Core 主控板通过 USB 连接，且显示为 `/dev/ttyACM0`（可用 `ls /dev/ttyACM*` 检查）
- 如果串口权限不足，执行 `sudo chmod 666 /dev/ttyACM0` 或将用户加入 `dialout` 组
- 当前系统没有真实反馈闭环：RViz 显示的是规划位置，不是电机编码器实际位置
- 首次使用前请确保电机已校准 home 位置
- `pyserial` 需要单独安装：`pip3 install pyserial`
