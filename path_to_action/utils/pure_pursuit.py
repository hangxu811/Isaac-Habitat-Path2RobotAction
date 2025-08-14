import math
import time
from typing import List, Tuple, Optional


class PurePursuitFollower2D:
    def __init__(
        self,
        path: List[Tuple[float, float]],
        lookahead_distance: float = 0.3,
        waypoint_radius: float = 0.15,
        control_dt: float = 0.3,
        max_speed: float = 0.5,
        min_speed: float = 0.05,
        angle_scale: float = 2.5,
        max_vyaw: float = 0.8,
        damping: float = 0.2,       # 平滑滤波系数，越大越平滑
        max_alpha: float = 0.8,    # 最大角加速度（rad/s²）
        max_ax: float = 0.5,  
        L_min: float = 0.20,   # 最小前瞻（m）
        L_max: float = 0.50,   # 最大前瞻（m）
        k_v: float = 0.80,     # 速度对前瞻的放大系数
        k_ang: float = 1.00,   # 角度对前瞻的抑制强度
        k_goal: float = 0.50   # 接近终点时的压缩强度   
    ):
        self.path = path
        self.lookahead_distance = lookahead_distance
        self.radius = waypoint_radius
        self.control_dt = control_dt
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.angle_scale = angle_scale
        self.max_vyaw = max_vyaw
        self.damping = damping
        self.max_alpha = max_alpha
        self.max_ax = max_ax
        self.L_min = L_min
        self.L_max = L_max
        self.k_v = k_v
        self.k_ang = k_ang
        self.k_goal = k_goal
        self.index = 0
        self.last_control_time = time.time()
        self.last_vx = 0.0
        self.last_vyaw = 0.0
        self.last_done = False

    def reset(self):
        self.index = 0
        self.last_control_time = time.time()
        self.last_vx = 0.0
        self.last_vyaw = 0.0
        self.last_done = False
    def _quadratic_blend(self,current, target, max_delta):
        """
        基于二次函数的平滑变化：
        current → 当前速度
        target  → 目标速度
        max_delta → 每步最大变化量（由 max_ax * dt 给出）
        """
        error = target - current
        # 权重（非线性推近，越靠近变化越慢）
        blend = min(1.0, (abs(error) / max_delta) ** 0.5)  # 平方根过渡
        return current + blend * error
    def _blend_velocity(self, current: float, target: float, max_delta: float) -> float:
        """
        平滑速度变换：
        - 加速：平方根形式的 quadratic 平滑
        - 减速：线性限幅
        """
        delta = target - current

        if delta > 0:
            # 加速：非线性平滑（sqrt）
            blend = min(1.0, (abs(delta) / max_delta) ** 0.5) if max_delta > 1e-6 else 1.0
            return current + blend * delta
        else:
            # 减速：线性限幅
            return current + max(delta, -max_delta)

    def is_done(self, current_pos: Tuple[float, float]) -> bool:
        if self.index >= len(self.path) - 1:
            last_point = self.path[-1]
            dist = math.hypot(current_pos[0] - last_point[0], current_pos[1] - last_point[1])
            return dist < 0.3
        return False
    def _distance_to_goal(self, current_pos: Tuple[float, float]) -> float:
        gx, gy = self.path[-1]
        return math.hypot(current_pos[0] - gx, current_pos[1] - gy)

    def _dynamic_lookahead(self,
                        current_pos: Tuple[float, float],
                        angle_diff: float,
                        vx_last: float) -> float:
        """
        返回基于速度、转弯角度与终点距离动态调整后的前瞻距离 L（单位：m）
        直觉：
        - 速度越快，看得越远（更稳）
        - 转弯越急，看得越近（更准）
        - 越接近终点，看得越近（防冲过）
        """
        # 1) 基础：速度项（速度越快，看得越远）
        L_speed = self.L_min + self.k_v * max(0.0, abs(vx_last))

        # 2) 角度项：用一个随角度单调减小的因子（0~1）
        #    例如 f_ang = 1 / (1 + k_ang * |angle|)，angle ∈ [0, π]
        f_ang = 1.0 / (1.0 + self.k_ang * abs(angle_diff))

        # 3) 终点项：越靠近终点压缩越多（0~1）
        #    用距离归一化：dist / (dist + 1)，越近越小；再抬个幂使其更敏感
        dist_goal = self._distance_to_goal(current_pos)
        f_goal = (dist_goal / (dist_goal + 1.0)) ** self.k_goal  # 0~1，越近越小

        # 4) 合成并夹紧
        L = L_speed * f_ang * f_goal
        return max(self.L_min, min(self.L_max, L))


    def find_lookahead_point(self, current_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        for i in range(self.index, len(self.path)):
            tx, ty = self.path[i]
            dx = tx - current_pos[0]
            dy = ty - current_pos[1]
            dist = math.hypot(dx, dy)
            if dist >= self.lookahead_distance:
                self.index = i
                return (tx, ty)
        return None

    def compute_control(
        self,
        current_pos: Tuple[float, float],
        current_heading: float
    ) -> Tuple[float, float, bool]:
        """
        返回:
            vx   : 线速度 (m/s)
            vyaw : 角速度 (rad/s)
            done : True 表示已到终点
        """

        # ===== 0. 终点判定 =====
        if self.is_done(current_pos):
            self.last_vx = self.last_vyaw = 0.0
            self.last_done = True
            self.index = len(self.path)
            return 0.0, 0.0, True

        # ===== 1. 查找前瞻点 =====
        target = self.find_lookahead_point(current_pos)
        if target is None:
            self.last_vx = self.last_vyaw = 0.0
            self.last_done = True
            return 0.0, 0.0, True

        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self._normalize_angle(angle_to_target - current_heading)

        # ===== 2. 线速度（根据角度调整）=====
        abs_angle = abs(angle_diff)
        speed_scale = max(0.0, 1.0 - self.angle_scale * abs_angle / math.pi)
        vx_desired = self.min_speed + (self.max_speed - self.min_speed) * speed_scale

        # ---- 限制线速度变化 ----
        dvx_max = self.max_ax * self.control_dt
        vx_limited = self._blend_velocity(self.last_vx, vx_desired, dvx_max)

        # ===== 角速度 =====
        vyaw_raw = max(-self.max_vyaw, min(self.max_vyaw, angle_diff))

        # ---- 限制角速度变化 ----
        dvyaw_max = self.max_alpha * self.control_dt
        vyaw_limited = self._blend_velocity(self.last_vyaw, vyaw_raw, dvyaw_max)

        # ---- 可选：角速度一阶低通滤波 ----
        if self.damping > 0:
            alpha = self.control_dt / (self.damping + self.control_dt)
            vyaw_smooth = (1 - alpha) * self.last_vyaw + alpha * vyaw_limited
        else:
            vyaw_smooth = vyaw_limited

        # ===== 4. 缓存并返回 =====
        self.last_vx = vx_limited
        self.last_vyaw = vyaw_smooth
        self.last_done = False
        return vx_limited, vyaw_smooth, False



    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
