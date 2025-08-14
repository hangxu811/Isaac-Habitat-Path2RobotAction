import numpy as np
from typing import Iterable, Sequence, Tuple, Union

Quat = Tuple[float, float, float, float]
ArrayLike = Union[Sequence[float], np.ndarray]

class Isaac2HabitatConverter:
    """
    将 Isaac 的四元数 (w,x,y,z) 转为 Habitat 的 (x,y,z,w)。
    这里的坐标系差异按：Isaac (X前, Y左, Z上) → Habitat (X右, Y上, Z前)
    等效为：先绕 Z 轴旋转 angle（默认 -90°），再调整分量顺序。
    默认 只考虑左右转向 没有考虑抬头低头
    注意：根据你的原实现，最终返回顺序为 (x2, z2, y2, w2) —— 即把 y、z 对调后再附上 w。
    """

    def __init__(self, angle_rad: float = -np.pi / 2):
        self.angle = float(angle_rad)
        # 预计算坐标系变换四元数 q_S (绕 Z 轴 angle)
        self._qS = (
            float(np.cos(self.angle / 2.0)),  # w
            0.0,                               # x
            0.0,                               # y
            float(np.sin(self.angle / 2.0)),   # z
        )

    @staticmethod
    def _quat_mul(q1: Quat, q2: Quat) -> Quat:
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        )

    def convert(self, q_wxyz: ArrayLike) -> Quat:
        """
        单个转换：输入 Isaac (w,x,y,z) → 输出 Habitat (x,z,y,w) 〔按你原函数的返回顺序〕
        """
        w, x, y, z = map(float, q_wxyz)
        q_hab_wxyz = self._quat_mul(self._qS, (w, x, y, z))
        w2, x2, y2, z2 = q_hab_wxyz
        return (x2, z2, y2, w2)

    def convert_many(self, qs_wxyz: Iterable[ArrayLike]) -> np.ndarray:
        """
        批量转换：输入可迭代的 (w,x,y,z)，返回 shape=(N,4) 的 ndarray（每行是 (x,z,y,w)）
        """
        out = [self.convert(q) for q in qs_wxyz]
        return np.asarray(out, dtype=float)

    # 让实例可直接像函数一样调用
    #__call__ = convert


# # ===== 用法示例 =====
# if __name__ == "__main__":
#     conv = Isaac2HabitatConverter()  # 默认绕 Z 轴 -90°
#     q_isaac = (0.9659258127212524, 0.0, 0.0, -0.25881901383399963)
#     print(conv.convert(q_isaac))

#     qs = [
#         (0.7071067811865476, 0.0, 0.0,  0.7071067811865476),
#         (0.25881904510252096, 0.0, 0.0, 0.9659258262890682),
#         (-0.9999999403953552,0.0,0.0,0.0)
#     ]
#     print(conv.convert_many(qs))

# "main()"