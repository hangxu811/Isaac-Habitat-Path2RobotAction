import json
import os
import math

def quat_yaw_wxyz(w, x, y, z):
    """返回绕 +z 轴的航向角（弧度），左转为正（CCW）。"""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def _normalize_angle(theta: float) -> float:
    return (theta + math.pi) % (2 * math.pi) - math.pi

class VLNCEPathLoader:
    """
    数据坐标系（来自 JSON）：
      - 点为 [x, y, z] 其中 x=北(+)、y=西(+)、z=上(+)
    输出：
      - path: 2D 路径 [(x, y2d)]，默认 y2d=y（与上面一致）
      - start_pos_2d: (x, y2d)
      - start_pos: 原始 3D (x, y, z)
      - start_heading: 绕 +z 的朝向（左转为正）
    """
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.episodes = []

        if not os.path.exists(self.file_path):
            raise FileNotFoundError(f"❌ 文件路径错误：{self.file_path}")
        self._load_json()

    def _load_json(self):
        with open(self.file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.episodes = data.get("episodes", [])
        print(f"✅ 成功加载 JSON，共找到 {len(self.episodes)} 个 episode。")

    def get_all_paths_and_starts(self, invert_y: bool = False):
        """
        返回:
          { episode_id: {
              "path": List[(x, y2d)],
              "start_pos_2d": (x, y2d),
              "start_pos": (x, y, z),
              "start_heading": float
          }}
        备注：
          - invert_y=True 时，y2d = -y （把“向西为正”改成“向东为正”）
        """
        results = {}
        for ep in self.episodes:
            ep_id = ep.get("episode_id", "N/A")

            # 2D 路径投影：x 前(北+)，y 左(西+)；按需反号
            ref_path = ep.get("reference_path", [])
            path_xy = []
            for pt in ref_path:
                if len(pt) >= 2:
                    x, y = pt[0], pt[1]
                    y2d = -y if invert_y else y
                    path_xy.append((x, y2d))
                else:
                    continue

            # 起点
            sp = ep.get("start_position", [-1, -1, -1])  # [x, y, z]
            sx, sy, sz = (sp + [0, 0, 0])[:3] if isinstance(sp, list) else (-1, -1, -1)
            start_pos = (sx, sy, sz)
            start_pos_2d = (sx, -sy if invert_y else sy)

            # 朝向（四元数 [w, x, y, z]）
            quat = ep.get("start_rotation", [1, 0, 0, 0])
            if not isinstance(quat, list) or len(quat) != 4:
                quat = [1, 0, 0, 0]
            qw, qx, qy, qz = quat
            heading = _normalize_angle(quat_yaw_wxyz(qw, qx, qy, qz))

            results[ep_id] = {
                "path": path_xy,
                "start_pos_2d": start_pos_2d,
                "start_pos": start_pos,
                "start_heading": heading
            }
        return results

    def print_all_paths(self, limit: int = 3):
        """打印前几个 episode 的 (x, y2d) 路径前几点，用于快速自检。"""
        paths = self.get_all_paths_and_starts()
        for i, (ep_id, info) in enumerate(paths.items()):
            if i >= limit:
                break
            print(f"Episode {ep_id} path head: {info['path'][:5]}")
