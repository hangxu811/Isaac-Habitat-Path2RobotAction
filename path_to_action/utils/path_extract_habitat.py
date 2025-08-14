import json
import os
import math
from typing import List, Tuple, Dict, Any


# ----------  坐标&角度工具  ----------

def add_yaw(q: Tuple[float, float, float, float],
            delta_rad: float) -> Tuple[float, float, float, float]:
    """在现有四元数上再绕 z 轴左转 delta_rad（右手系，CCW 为正）"""
    w, x, y, z = q
    half = 0.5 * delta_rad
    cw, cz = math.cos(half), math.sin(half)          # Δψ 的四元数 (cw,0,0,cz)

    # q_new = q ⊗ Δq
    nw = w*cw - z*cz
    nx = x*cw + y*cz
    ny = y*cw - x*cz
    nz = z*cw + w*cz

    # 归一化抵消数值误差
    norm = math.sqrt(nw*nw + nx*nx + ny*ny + nz*nz)
    return (nw/norm, nx/norm, ny/norm, nz/norm)


def _normalize_angle(theta: float) -> float:
    """把角度归一化到 (-π, π]"""
    return (theta + math.pi) % (2 * math.pi) - math.pi


def quat_yaw_about_old_y(w: float, x: float, y: float, z: float) -> float:
    """
    ⚠️ 旧数据坐标系：x→东，y→上，z→南
    这里计算围绕 “旧 y 轴(上)” 的偏航角（弧度），左转为正(CCW)。
    """
    siny_cosp = 2 * (w * y + z * x)
    cosy_cosp = 1 - 2 * (x * x + y * y)
    return math.atan2(siny_cosp, cosy_cosp)  # ∈ (-π, π]


# ----------  关键修改处  ----------
def _convert_point_old2new(pt: List[float]) -> Tuple[float, float, float]:
    """
    旧坐标 [x_east, y_up, z_south] →
    新坐标 [x_east, y_north, z_up]，其中
        x_new =  x_east
        y_new = -z_south   (南→北取反)
        z_new =  y_up
    """
    x_e, y_u, z_s = (pt + [0, 0, 0])[:3]
    return (x_e, -z_s, y_u)



# ----------  主类  ----------

class VLNCEPathLoader:
    """
    输入 JSON 坐标系:  x→东(+), y→上(+), z→南(+)
    输出坐标系:       x→北(+), y→西(+), z→上(+)

    返回内容:
      - "path":           2D 路径 [(x_north, y_west)]
      - "start_pos":      3D 起点 (x_north, y_west, z_up)
      - "start_pos_2d":   2D 起点
      - "start_heading":  绕 +z_up (新坐标系) 的航向角，左转(CCW)为正
    """

    YAW_OFFSET = math.pi / 2  # ← 加在原始 heading 上，让朝向更合理

    def __init__(self, file_path: str):
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"❌ 文件不存在: {file_path}")
        self.file_path = file_path
        self._load_json()

    def _load_json(self):
        with open(self.file_path, "r", encoding="utf-8") as f:
            data: Dict[str, Any] = json.load(f)
        self.episodes: List[Dict[str, Any]] = data.get("episodes", [])
        print(f"✅ 已加载 {len(self.episodes)} 个 episode")

    def get_all_paths_and_starts(self, invert_second: bool = False):
        results = {}
        for ep in self.episodes:
            ep_id = ep.get("episode_id", "N/A")

            # ---- 路径 ----
            path_xz = []
            for pt in ep.get("reference_path", []):
                if len(pt) < 3:
                    continue
                xn, y_new, z_new = _convert_point_old2new(pt)
                y_out = -y_new if invert_second else y_new
                path_xz.append((xn, y_out))

            # ---- 起点位置 ----
            sxn, sy_new, sz_new = _convert_point_old2new(
                ep.get("start_position", [-1, -1, -1])
            )
            sy_out = -sy_new if invert_second else sy_new
            start_pos = (sxn, sy_new, sz_new)
            start_pos_2d = (sxn, sy_out)

            # ---- 起始朝向 ----
            qw, qx, qy, qz = (ep.get("start_rotation", [1, 0, 0, 0]) + [0, 0, 0, 0])[:4]
            heading = _normalize_angle(quat_yaw_about_old_y(qw, qx, qy, qz))

            # ✅ 加上左转 90°（可视化 + 控制需要）
            heading = _normalize_angle(heading + self.YAW_OFFSET)

            results[ep_id] = {
                "path": path_xz,
                "start_pos": start_pos,
                "start_pos_2d": start_pos_2d,
                "start_heading": heading,
            }
        return results


    def print_all_paths(self, limit: int = 3):
        """快速查看前几个 episode 的前 5 个路径点（已是新坐标系）。"""
        paths = self.get_all_paths_and_starts()
        for i, (eid, info) in enumerate(paths.items()):
            if i >= limit:
                break
            print(f"Episode {eid} → {info['path'][:5]}")
