import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import List, Tuple

class PathFollowerVisualizer2D:
    def __init__(
        self,
        path: List[Tuple[float, float]],
        follower,
        init_pos: Tuple[float, float],
        init_heading: float = 0.0,
        dt: float = 0.3,
        fig_size: Tuple[int, int] = (7, 7),
    ):
        """
        坐标系：x 向北（前），y 向西（左）
        path: 路径点 (x, y)
        follower: PurePursuitFollower2D 实例（atan2(dy, dx)）
        init_pos: 初始坐标 (x, y)
        init_heading: 初始朝向角（弧度，绕 +z，左转为正）
        dt: 时间步长
        """
        self.path = path
        self.follower = follower
        self.agent_pos = init_pos
        self.agent_heading = init_heading
        self.dt = dt
        self.trajectory = [init_pos]
        self.action = []
        self.position = []

        # ===== 可视化设置 =====
        self.fig, self.ax = plt.subplots(figsize=fig_size)
        self.ax.set_aspect('equal')
        self.ax.set_title("Pure Pursuit 2D Path Tracking")
        self.ax.set_xlabel("x (north +)")
        self.ax.set_ylabel("y (west +)")

        path_x, path_y = zip(*self.path)
        self.path_plot, = self.ax.plot(path_x, path_y, 'bo', markersize=3, label='Path')
        self.agent_plot, = self.ax.plot([], [], 'ro', label='Agent')
        self.traj_plot, = self.ax.plot([], [], 'r--', alpha=0.6, label='Trajectory')

        # 显示范围
        pad = 1.0
        self.ax.set_xlim(min(path_x) - pad, max(path_x) + pad)
        self.ax.set_ylim(min(path_y) - pad, max(path_y) + pad)
        self.ax.legend()

    def _update(self, frame):
        if self.follower.is_done(self.agent_pos):
            print("✅ 路径追踪完成")
            self.anim.event_source.stop()
            return self.agent_plot, self.traj_plot

        vx, vyaw, done = self.follower.compute_control(self.agent_pos, self.agent_heading)
        self.action.append((vx, vyaw, self.dt))

        # 打印状态
        print(f"[Frame {frame:03}] vx={vx:.3f} m/s, vyaw={vyaw:.3f} rad/s, heading={self.agent_heading:.3f} rad")

        # 更新朝向与位置（x: north+, y: west+）
        self.agent_heading += vyaw * self.dt
        dx = vx * self.dt * math.cos(self.agent_heading)
        dy = vx * self.dt * math.sin(self.agent_heading)
        self.agent_pos = (self.agent_pos[0] + dx, self.agent_pos[1] + dy)
        self.trajectory.append(self.agent_pos)
        self.position.append(self.agent_pos)

        # 刷新绘图
        traj_x, traj_y = zip(*self.trajectory)
        self.traj_plot.set_data(traj_x, traj_y)
        self.agent_plot.set_data([self.agent_pos[0]], [self.agent_pos[1]])
        return self.agent_plot, self.traj_plot

    def run(self, max_frames=500, interval_ms=333):
        self.anim = animation.FuncAnimation(
            self.fig,
            self._update,
            frames=max_frames,
            interval=interval_ms,
            blit=True
        )
        plt.show()

        print("\n📤 动作序列 (vx, vyaw, dt):")
        for i, act in enumerate(self.action):
            px, py = self.position[i]
            print(f"Step {i:03}: vx={act[0]:.3f}, vyaw={act[1]:.3f}, dt={act[2]:.2f} | pos(x,y)=({px:.3f},{py:.3f})")
