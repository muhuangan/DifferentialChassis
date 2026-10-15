"""实现底盘速度到左右轮转速的近似映射，以及固定 12 字节 UART 数据帧打包。"""

import math
import struct


# < 表示小端字节序；2s 为两个字节；i 为有符号 32 位整数。
# 帧结构：帧头 2 字节 + 左轮转速 4 字节 + 右轮转速 4 字节 + 帧尾 2 字节。
FRAME = struct.Struct('<2sii2s')
HEADER = b'\xAA\x55'
TAIL = b'\x0D\x0A'


def pack_speeds(left_rpm, right_rpm):
    """
    @brief  将左右轮转速打包为一帧串口数据。
    @param  left_rpm   左轮转速，有符号 32 位整数，单位 rpm（转/分）。
    @param  right_rpm  右轮转速，有符号 32 位整数，单位 rpm（转/分）。
    @retval bytes  固定长度为 12 字节的数据帧。
    @note   使用小端字节序；转速超出 int32 范围时由 struct.pack 抛出异常。
    """
    # 按预先定义的帧格式排列帧头、左右轮转速和帧尾。
    return FRAME.pack(HEADER, left_rpm, right_rpm, TAIL)


def map_speeds(linear, angular, kv, kw, rpm_limit, left_direction, right_direction):
    """
    @brief  将底盘线速度和角速度映射为左右轮转速。
    @param  linear           底盘线速度，单位 m/s。
    @param  angular          底盘角速度，单位 rad/s。
    @param  kv               线速度到轮转速的换算系数，单位 rpm/(m/s)。
    @param  kw               角速度到轮转速的换算系数，单位 rpm/(rad/s)。
    @param  rpm_limit        单轮转速绝对值上限，单位 rpm（转/分）。
    @param  left_direction   左轮方向修正系数，只能为 +1 或 -1。
    @param  right_direction  右轮方向修正系数，只能为 +1 或 -1。
    @retval tuple  左右轮整数转速二元组，单位 rpm（转/分）。
    @note   使用近似映射，未直接使用标定后的轮径和轮距；
            左右轮分别限幅后取整，遇到非有限速度值时抛出 ValueError。
    """
    # 差速映射：直行分量同向叠加，转向分量在左右轮上符号相反。
    left = kv * linear - kw * angular
    right = kv * linear + kw * angular

    # 拒绝 NaN 和无穷大，避免生成无效控制指令。
    if not all(math.isfinite(v) for v in (linear, angular, left, right)):
        raise ValueError('Non-finite velocity command')

    # 分别限幅、使用 Python round 取整，再根据实际电机安装方向修正符号。
    left = round(max(-rpm_limit, min(rpm_limit, left))) * left_direction
    right = round(max(-rpm_limit, min(rpm_limit, right))) * right_direction

    return int(left), int(right)


class CommandState:
    """保存最新左右轮指令；指令过期时返回零转速。"""

    def __init__(self, timeout):
        """
        @brief  初始化速度指令缓存。
        @param  timeout  指令有效时间，单位秒。
        @retval None
        @note   初始转速为零，接收时间为空，表示尚未收到有效更新。
        """
        self.timeout = timeout
        self.speeds = (0, 0)
        self.received_at = None

    def update(self, speeds, now):
        """
        @brief  保存最新速度指令及其接收时间。
        @param  speeds  左右轮转速二元组，单位 rpm（转/分）。
        @param  now     当前单调时间，单位秒。
        @retval None
        @note   每次更新直接覆盖上一条指令，不排队保存历史指令。
        """
        self.speeds = speeds
        self.received_at = now

    def current(self, now):
        """
        @brief  获取当前有效的左右轮速度指令。
        @param  now  当前单调时间，单位秒。
        @retval tuple  有效指令对应的左右轮转速；未收到指令或超时时返回 (0, 0)。
        @note   超时仅改变本次返回值，不清除缓存中的历史指令。
        """
        # 没收到过指令，或指令已过期，则要求停车。
        if self.received_at is None or now - self.received_at >= self.timeout:
            return (0, 0)

        return self.speeds