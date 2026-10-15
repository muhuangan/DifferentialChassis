"""终端键盘状态：区分首次按下等待和连续重复输入超时。"""


class KeyboardState:
    """维护运动方向、目标速度及按键输入超时状态。"""

    def __init__(self, linear_speed, angular_speed, linear_step, angular_step,
                 max_linear, max_angular, stop_delay,
                 first_repeat_delay=0.80):
        """
        @brief  初始化键盘控制状态。
        @param  linear_speed        初始线速度，单位 m/s。
        @param  angular_speed       初始角速度，单位 rad/s。
        @param  linear_step         线速度调节步长，单位 m/s。
        @param  angular_step        角速度调节步长，单位 rad/s。
        @param  max_linear          最大线速度，单位 m/s。
        @param  max_angular         最大角速度，单位 rad/s。
        @param  stop_delay          连续重复输入中断后的停车等待时间，单位秒。
        @param  first_repeat_delay  首次按下后等待自动重复的时间，单位秒。
        @retval None
        @note   首次等待时间不能短于连续输入超时时间；
                本类仍通过字符输入间隔推测松键，不是真正的松键检测。
        """
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.linear_step = linear_step
        self.angular_step = angular_step
        self.max_linear = max_linear
        self.max_angular = max_angular

        # 首次按下使用较长等待时间，避免键盘尚未开始重复就被判定停车。
        self.stop_delay = stop_delay
        self.first_repeat_delay = max(first_repeat_delay, stop_delay)
        self.motion_timeout = self.first_repeat_delay

        # 当前运动方向和最近一次方向输入时间。
        self.motion = None
        self.last_motion = None

        # ANSI（终端控制）转义序列解析状态。
        self.escape = None
        self.escape_at = 0.0

    def _clear_motion(self):
        """
        @brief  清除运动状态。
        @param  None
        @retval None
        @note   下一次方向输入重新按首次按下处理，保留目标速度设置。
        """
        self.motion = None
        self.last_motion = None
        self.motion_timeout = self.first_repeat_delay

    def feed(self, key, now):
        """
        @brief  处理一个键盘输入字符。
        @param  key  当前输入字符。
        @param  now  当前单调时间，单位秒。
        @retval True   继续运行。
        @retval False  收到 Ctrl+C，要求退出。
        @note   同一方向在有效期内再次输入时，切换为较短的重复输入超时；
                新方向或超时后的输入，重新使用首次等待时间。
        """
        # 原始终端模式下，Ctrl+C 对应字符 0x03。
        if key == '\x03':
            self._clear_motion()
            return False

        # 清除未及时完成的转义序列。
        if self.escape is not None and now - self.escape_at > 0.1:
            self.escape = None

        # ESC 表示可能开始了方向键或功能键的转义序列。
        if key == '\x1b':
            self.escape = 'start'
            self.escape_at = now
            return True

        # 忽略转义序列，避免将其中的 A/D 等字符当作运动指令。
        if self.escape is not None:
            if self.escape == 'start' and key in '[O':
                self.escape = 'sequence'
            elif self.escape == 'start' or '@' <= key <= '~':
                self.escape = None
            return True

        key = key.lower()

        if key in ('w', 's', 'a', 'd'):
            # 相同方向、且上一条指令尚未超时，视为连续重复输入。
            is_repeat = (
                self.motion == key
                and self.last_motion is not None
                and now - self.last_motion < self.motion_timeout
            )

            if is_repeat:
                # 已进入重复输入阶段，使用较短超时提高松键停车响应。
                self.motion_timeout = self.stop_delay
            else:
                # 首次按下、切换方向或重新按下时，等待键盘启动自动重复。
                self.motion_timeout = self.first_repeat_delay

            self.motion = key
            self.last_motion = now

        elif key == ' ':
            # 空格直接清除运动状态，下一次发布输出零速度。
            self._clear_motion()

        elif key == 'q':
            # 同时增加线速度和角速度设置，并限制上限。
            self.linear_speed = min(
                self.max_linear,
                self.linear_speed + self.linear_step
            )
            self.angular_speed = min(
                self.max_angular,
                self.angular_speed + self.angular_step
            )

        elif key == 'e':
            # 同时减小线速度和角速度设置，最小为零。
            self.linear_speed = max(
                0.0,
                self.linear_speed - self.linear_step
            )
            self.angular_speed = max(
                0.0,
                self.angular_speed - self.angular_step
            )

        return True

    def velocity(self, now):
        """
        @brief  获取当前应发布的底盘速度。
        @param  now  当前单调时间，单位秒。
        @retval tuple  (线速度, 角速度)，单位分别为 m/s、rad/s。
        @note   超过当前阶段的等待时间后停车；
                未超时时，根据当前方向输出对应速度。
        """
        # 根据首次等待或重复输入阶段，使用对应的超时阈值。
        if (
            self.last_motion is None
            or now - self.last_motion >= self.motion_timeout
        ):
            self._clear_motion()

        # 每次只保留一个运动方向，避免残留另一个轴的速度。
        return {
            'w': (self.linear_speed, 0.0),
            's': (-self.linear_speed, 0.0),
            'a': (0.0, self.angular_speed),
            'd': (0.0, -self.angular_speed),
        }.get(self.motion, (0.0, 0.0))