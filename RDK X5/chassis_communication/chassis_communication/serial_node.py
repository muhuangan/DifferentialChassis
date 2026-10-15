"""订阅 /cmd_vel，并以固定频率发送包含左右轮 int32 转速的串口帧。"""

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

from .protocol import CommandState, map_speeds, pack_speeds


class SerialNode(Node):
    """将 ROS 2 底盘速度指令转换为左右轮转速，并通过串口下发。"""

    def __init__(self):
        """
        @brief  初始化串口通信节点。
        @param  None
        @retval None
        @note   声明并校验参数，创建指令缓存、订阅器和发送定时器；
                dry_run 为 False 时打开真实串口，为 True 时只生成数据并打印。
                参数无效或串口打开失败时抛出异常。
        """
        super().__init__('serial_node')

        # 默认串口、速度映射和发送参数；频率单位为 Hz，超时时间单位为秒。
        defaults = {
            'port': '/dev/ttyS1', 'baudrate': 115200,
            'kv': 200.0, 'kw': 100.0, 'rpm_limit': 100,
            'left_direction': 1, 'right_direction': 1,
            'send_hz': 50.0, 'command_timeout': 0.3,
            'write_timeout': 0.01, 'dry_run': False,
            'first_repeat_delay': 0.80,
        }

        # 声明 ROS 2 参数，允许启动时覆盖默认配置。
        self.cfg = {
            name: self.declare_parameter(name, value).value
            for name, value in defaults.items()
        }

        # 换算系数、发送频率和超时时间必须为有限正数。
        for name in ('kv', 'kw', 'send_hz', 'command_timeout', 'write_timeout'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(name + ' must be finite and positive')

        # 转速上限不能超过有符号 32 位整数的最大正值。
        if not 0 < self.cfg['rpm_limit'] <= 2147483647:
            raise ValueError('rpm_limit must be in 1..2147483647')

        if self.cfg['baudrate'] <= 0:
            raise ValueError('baudrate must be positive')

        # 方向参数只能保持或反转符号，不能参与转速倍率调整。
        for name in ('left_direction', 'right_direction'):
            if self.cfg[name] not in (-1, 1):
                raise ValueError(name + ' must be +1 or -1')

        # 保存最新指令；超过 command_timeout 未更新时输出零转速。
        self.command = CommandState(self.cfg['command_timeout'])
        self.port = None

        # 负无穷初值使第一次状态日志和第一次异常警告可以立即输出。
        self.last_log = -float('inf')
        self.last_warning = -float('inf')

        # 订阅底盘速度消息，历史消息队列深度为 1。
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 1)

        # 使用稳态时钟定时发送，避免 use_sim_time 影响硬件发送节奏。
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.cfg['send_hz'], self.on_timer, clock=self.steady_clock)

        # 真实发送模式下打开串口：8 位数据、无校验、1 位停止位。
        # timeout=0 为非阻塞读取；write_timeout 限制单次写入等待时间。
        if not self.cfg['dry_run']:
            self.port = serial.Serial(
                port=self.cfg['port'], baudrate=self.cfg['baudrate'],
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0,
                write_timeout=self.cfg['write_timeout'], exclusive=True)

        # 打印当前协议格式、串口及运行模式。
        self.get_logger().info(
            f"12-byte int32 LE; port={self.cfg['port']}, "
            f"baud={self.cfg['baudrate']}, dry_run={self.cfg['dry_run']}")

    def on_cmd(self, msg):
        """
        @brief  处理一条 /cmd_vel 速度消息。
        @param  msg  Twist 消息，使用 linear.x 和 angular.z 两个分量。
        @retval None
        @note   将底盘速度换算为左右轮转速并更新缓存；
                遇到非有限速度值时缓存零转速，警告最多每秒打印一次。
        """
        now = time.monotonic()

        try:
            # 根据换算系数、转速上限和方向配置计算左右轮目标转速。
            speeds = map_speeds(
                msg.linear.x, msg.angular.z, self.cfg['kv'], self.cfg['kw'],
                self.cfg['rpm_limit'], self.cfg['left_direction'],
                self.cfg['right_direction'])

        except ValueError:
            # 无效速度指令转换成零转速，避免继续执行上一条运动指令。
            speeds = (0, 0)

            # 限制警告频率，防止异常消息持续到来时刷屏。
            if now - self.last_warning >= 1.0:
                self.get_logger().warning('Invalid cmd_vel: sending zero rpm')
                self.last_warning = now

        # 无论正常指令还是异常后的零速度，都更新缓存及接收时间。
        self.command.update(speeds, now)

    def on_timer(self):
        """
        @brief  按固定周期打包并发送最新轮速指令。
        @param  None
        @retval None
        @note   指令超时时发送零转速；真实模式下检查是否完整写出 12 字节，
                写入不完整时抛出 SerialException；两种模式均每 0.5 秒打印一次状态。
        """
        now = time.monotonic()

        # 获取当前有效指令，并打包为固定长度的串口数据帧。
        left, right = self.command.current(now)
        packet = pack_speeds(left, right)

        # 非模拟模式下真实写入串口，检查返回的写入字节数。
        if not self.cfg['dry_run']:
            if self.port.write(packet) != len(packet):
                raise serial.SerialException('Incomplete UART write')

        # 打印左右轮目标转速和十六进制帧内容，便于核对发送数据。
        if now - self.last_log >= 0.5:
            self.get_logger().info(
                f'L={left} R={right} rpm | {packet.hex(" ").upper()}')
            self.last_log = now

    def close_port(self):
        """
        @brief  停止周期发送并关闭串口。
        @param  None
        @retval None
        @note   串口仍打开时尝试发送最后一帧零转速；
                即使停车帧发送失败，也会关闭串口并记录相关错误。
        """
        # 先取消发送定时器，停止后续周期回调。
        self.timer.cancel()

        if self.port is not None and self.port.is_open:
            try:
                # 退出前尝试下发停车帧。
                packet = pack_speeds(0, 0)
                if self.port.write(packet) != len(packet):
                    self.get_logger().error('Final zero packet was incomplete')

            except (serial.SerialException, OSError) as exc:
                # 记录最终停车帧的发送异常。
                self.get_logger().error(f'Could not send final zero packet: {exc}')

            finally:
                # 无论最后一帧是否发送成功，都释放串口。
                self.port.close()


def main(args=None):
    """
    @brief  运行串口通信节点。
    @param  args  传给 ROS 2 初始化函数的参数，默认使用进程命令行参数。
    @retval None
    @note   创建节点并持续处理回调；退出时关闭串口、销毁节点并关闭 ROS 2。
            捕获到串口、操作系统或参数异常时，以退出码 1 结束进程。
    """
    rclpy.init(args=args)

    # 记录节点是否成功创建，以及是否发生需要报告的运行错误。
    node = None
    failed = False

    try:
        node = SerialNode()

        # 持续执行订阅回调和定时发送回调，直到关闭或发生异常。
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        # 用户中断或 ROS 2 外部关闭，按正常退出处理。
        pass

    except (serial.SerialException, OSError, ValueError) as exc:
        # 记录通信或参数错误，后续使用非零退出码报告失败。
        failed = True
        print(f'serial_node: {exc}', flush=True)

    finally:
        try:
            # 节点已创建时，停止发送并释放串口。
            if node is not None:
                node.close_port()
        finally:
            # 即使关闭串口发生异常，也继续释放 ROS 2 资源。
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    if failed:
        raise SystemExit(1)


# 直接运行本文件时进入主函数。
if __name__ == '__main__':
    main()