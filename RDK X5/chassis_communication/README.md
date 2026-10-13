# chassis_communication

两个 Python 节点：键盘发布 `/cmd_vel`，串口节点映射为左右轮 rpm 并发送固定 12 字节数据。面向 Linux ROS2（例如 Humble）。不修改现有 STM32 程序。

```text
keyboard_node → /cmd_vel (geometry_msgs/msg/Twist) → serial_node → UART → STM32 → 电机
```

## 1. 复制与构建

把整个 `chassis_communication` 文件夹复制到目标板的 `~/ros2_ws/src/`。以下命令在目标板 Linux 终端执行，以 ROS2 Humble 为例；其他版本替换 `humble`。

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-serial python3-colcon-common-extensions python3-pytest
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select chassis_communication
source install/setup.bash
```

`rosdep` 需要已安装并初始化。若缺少它，先配置 ROS2 开发环境。`package.xml` 的维护者邮箱是占位值，发布包前换成自己的信息。

## 2. 先看组包结果，不打开串口

终端一：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch chassis_communication serial.launch.py dry_run:=true
```

终端二：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run chassis_communication keyboard_node
```

点击键盘终端再操作：W/S 前进/后退；A/D 原地左转/右转；Q/E 调整速度；空格停止；Ctrl+C 退出。

串口终端每 0.5 秒显示左右 rpm 和完整十六进制报文，内部仍按 50 Hz 处理。默认按 W 时是 `60,60`，按 A 时是 `-50,50`。

**终端没有真正的松键事件。** 本版在最后一次方向字符后 150 ms 归零；依赖系统按键重复。按住时若首次重复延迟大于 150 ms，会出现“先动一下、停一下、再连续动”，不等于串口故障。可调大 `stop_delay`，但停下响应也会相应变慢。它不支持真正的多键同时按住。空格是软件零速命令，不是硬件急停。

## 3. 使用真实串口

先结束预览节点，避免同时启动两个串口节点。确认参数文件中的串口名和波特率，再在终端一执行：

```bash
ros2 launch chassis_communication serial.launch.py
```

默认 `/dev/ttyS1`、115200、8N1；参数文件为 `config/params.yaml`。实际串口名要按板上连接确认。

也可以直接覆盖端口和方向，启动单个串口节点：

```bash
ros2 run chassis_communication serial_node --ros-args \
  -p port:=/dev/ttyUSB0 -p baudrate:=115200 -p right_direction:=-1
```

读取自定义参数文件：

```bash
ros2 launch chassis_communication serial.launch.py params_file:=/home/sunrise/chassis_params.yaml
ros2 run chassis_communication keyboard_node --ros-args --params-file /home/sunrise/chassis_params.yaml
```

把示例路径换成实际路径。参数在启动时读取，修改后重启节点；Q/E 调速不需要重启。使用 launch 时，`dry_run` 以启动参数为准，覆盖 YAML 中的同名值。启动文件只启动串口节点，键盘需要单独的交互终端。不要同时保留其他 `/cmd_vel` 控制源，键盘空闲时也会发布零速。

## 4. 映射与参数

```text
left_rpm  = kv × linear.x − kw × angular.z
right_rpm = kv × linear.x + kw × angular.z
```

两轮分别限幅、取整，最后乘各自方向参数。默认约定 rpm 是轮子/减速器输出轴转速；本包不额外乘减速比。若电机接口实际使用转子 rpm，应先统一单位。两个值使用 int32 并不自动保证单位与下位机一致。

| 参数 | 默认 | 用途 |
|---|---:|---|
| `kv` | 200.0 | 前后速度映射系数 |
| `kw` | 100.0 | 转向速度映射系数 |
| `rpm_limit` | 100 | 每轮 ±100 rpm，练习限幅，并非电机额定值 |
| `left_direction` / `right_direction` | 1 / 1 | 各自取 +1 或 -1，修正安装方向 |
| `send_hz` | 50.0 | 串口发送频率 |
| `command_timeout` | 0.3 | 未收到新 `/cmd_vel` 达 0.3 s 后发送零速 |
| `write_timeout` | 0.01 | 单次串口写入超时，单位秒 |
| `dry_run` | false | true 时只输出报文，不打开串口 |
| 键盘 `stop_delay` | 0.15 | 最后一个方向字符后的归零时间 |

这只是可调系数映射，没有通过轮径、轮距标定，`linear.x` 不是保证达到的真实底盘速度。运行周期也不是硬实时保证。

## 5. 串口协议（固定 12 字节）

| 字节偏移 | 内容 |
|---|---|
| 0～1 | `AA 55` 帧头 |
| 2～5 | 左轮 `int32_t`，小端序，rpm |
| 6～9 | 右轮 `int32_t`，小端序，rpm |
| 10～11 | `0D 0A` 帧尾 |

Python 使用 `struct.pack('<2sii2s', header, left_rpm, right_rpm, tail)`。负数采用二进制补码；不是 float，也不是可读数字字符串。

```text
左 +60，右 -60：AA 55 3C 00 00 00 C4 FF FF FF 0D 0A
左   0，右   0：AA 55 00 00 00 00 00 00 00 00 0D 0A
```

下位机找 `AA 55`，收满 12 字节，核对偏移 10/11 的帧尾，再解出偏移 2/6 开始的两个 int32。校验失败时从下一字节重新找帧头。一次接收可能是半包、多包；数据中也可能出现 `0D 0A`，**不能用“读到换行”为分包规则**。本版没有 CRC（循环冗余校验），帧尾只能辅助同步，不能验证载荷没有误码。

与旧 `ROS:` 开头的协议不兼容，下位机必须对应修改解包逻辑。本包不实现电机 CAN 协议或下位机速度环。

## 6. 超时和退出

- 启动后尚无命令时发送零速；新命令只保留最新一条。
- 键盘进程消失时，串口节点在命令超时后发送零速；非法 NaN/Inf 指令也转成零速。
- 串口写入失败会报错并退出，不静默继续。退出时尝试发送一次零速，但不保证下位机已收到。
- 串口断线、上位机断电或进程被强制终止时，上位机不能发送停止；STM32 仍需接收超时停车。

## 7. 测试

不依赖 ROS2 的离线测试，在包根目录执行：

```bash
python3 -m unittest discover -s test -v
```

覆盖报文字节序、int32 边界、正反转映射、限幅、超时归零、键盘状态，以及用模拟 ROS/串口对象验证订阅回调到报文的路径。模拟测试不能代替真实 ROS2 和电机联调。

在 ROS2 工作空间也可执行：

```bash
colcon test --packages-select chassis_communication
colcon test-result --verbose
```

可先退出键盘节点，在 **dry_run 模式**验证订阅：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'
```

最后一条命令到达后 300 ms 恢复零速。预览日志仅 2 Hz，可能错过这条短命令；需要持续观察时，把 `--once` 换成 `--rate 10`，结束发布后看归零。

参考：[ROS2 Python 节点示例](https://github.com/ros2/examples/tree/humble/rclpy/topics)、[pySerial 串口接口](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)。
