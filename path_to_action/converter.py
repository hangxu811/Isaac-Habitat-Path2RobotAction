# converter.py
# -*- coding: utf-8 -*-
"""
通用路径转换与导出脚本（支持输入/输出坐标系 isaac/habitat），并写出目标 JSON 结构：
- scenes[*].scene_id：沿用源文件
- trajectories[*].trajectory_id：沿用源文件（无则退回 episode_id）
- points[*].orientation：仅做坐标系转换；out=habitat -> {x,y,z,w}；out=isaac -> {w,x,y,z}
- samples：同一 trajectory 的所有指令都会各自生成一条 sample
- camera_parameters：可由 --camera-json 指定数组，否则留空
- dataset_metadata.scene_source：输入文件名（例如 input.json）
- dataset_metadata.input_format / output_format：来自 --in-fmt / --out-fmt

用法示例：
  python converter.py --input input.json --output out.json --in-fmt isaac/habitat --out-fmt isaac/habitat --group-size 5 --tail keep
"""

import math
import json
import argparse
from typing import List, Tuple, Dict, Any
from pathlib import Path
from collections import defaultdict
import sys
import os
import importlib

# ================ 环境与依赖 ================
# 确保可以 import utils.*
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.pure_pursuit_dynamic_ld import PurePursuitFollower2D
from utils.rotation_isaac2habitat import Isaac2HabitatConverter  # isaac->habitat 四元数变换（绕 Z -90°）


# ================ 坐标系转换 ================
# 位置：isaac -> habitat : [x,y,z] -> [x, z, -y]
#      habitat -> isaac : [x,y,z] -> [x, -z, y]
def convert_position_dict(pos: Dict[str, float], in_fmt: str, out_fmt: str) -> Dict[str, float]:
    x = float(pos.get("x", 0.0)); y = float(pos.get("y", 0.0)); z = float(pos.get("z", 0.0))
    if in_fmt == out_fmt:
        return {"x": x, "y": y, "z": z}
    if in_fmt == "isaac" and out_fmt == "habitat":
        return {"x": x, "y": z, "z": -y}
    if in_fmt == "habitat" and out_fmt == "isaac":
        return {"x": x, "y": -z, "z": y}
    raise ValueError(f"Unsupported format conversion: {in_fmt} -> {out_fmt}")

# 旋转（保持四元数，做坐标系变换；不转欧拉，不做角度缩放）
_conv_i2h = Isaac2HabitatConverter(angle_rad=-math.pi/2)  # isaac->habitat

def convert_rotation_wxyz_for_orientation(rot: Dict[str, float], in_fmt: str, out_fmt: str):
    """
    输入 rot 为字典 {w,x,y,z}（in 坐标系），返回 [w,x,y,z]（out 坐标系）。
    说明：habitat->isaac 不再附加 +90°；直接透传（坐标系旋转由上游保证）。
    """
    w = float(rot.get("w", 1.0)); x = float(rot.get("x", 0.0))
    y = float(rot.get("y", 0.0)); z = float(rot.get("z", 0.0))

    if in_fmt == out_fmt:
        return [w, x, y, z]

    if in_fmt == "isaac" and out_fmt == "habitat":
        # 你的转换器：输入 (w,x,y,z) -> 输出 (x,z,y,w)
        x2, z2, y2, w2 = _conv_i2h.convert((w, x, y, z))
        return [w2, x2, y2, z2]  # 回到 wxyz

    if in_fmt == "habitat" and out_fmt == "isaac":
        # 去掉 +90°：直接透传
        return [w, x, y, z]

    raise ValueError(f"Unsupported rotation conversion: {in_fmt} -> {out_fmt}")

def format_orientation(rot_wxyz, out_fmt: str) -> Dict[str, float]:
    """
    rot_wxyz: [w,x,y,z] （已经是 out 坐标系下的四元数）
    返回用于输出 JSON 的 orientation 字段：
      - habitat: {"x": x, "y": y, "z": z, "w": w}
      - isaac  : {"w": w, "x": x, "y": y, "z": z}
    """
    w, x, y, z = rot_wxyz
    if out_fmt == "habitat":
        return {"x": float(x), "y": float(y), "z": float(z), "w": float(w)}
    return {"w": float(w), "x": float(x), "y": float(y), "z": float(z)}

# ================ Pure Pursuit 控制 ================
def _normalize_angle(theta: float) -> float:
    return (theta + math.pi) % (2 * math.pi) - math.pi

def run_pure_pursuit(path: List[Tuple[float, float]],
                     init_pos: Tuple[float, float],
                     init_heading: float,
                     init_y: float = 0.0,
                     dt: float = 0.3,
                     max_steps: int = 500):
    follower = PurePursuitFollower2D(path, control_dt=dt)
    agent_pos = init_pos
    agent_heading = init_heading
    actions = []

    for _ in range(max_steps):
        vx, vyaw, done = follower.compute_control(agent_pos, agent_heading)

        # “老 rotation”四元数（输入坐标系，仅 yaw）
        half = 0.5 * agent_heading
        qw = math.cos(half); qx = 0.0; qy = 0.0; qz = math.sin(half)

        actions.append({
            "vx": vx, "vy": 0.0, "vyaw": vyaw, "dt": dt,
            "pos": {"x": agent_pos[0], "y": agent_pos[1], "z": init_y},
            "rotation": {"w": qw, "x": qx, "y": qy, "z": qz},
        })

        dx = vx * dt * math.cos(agent_heading)
        dy = vx * dt * math.sin(agent_heading)
        agent_pos = (agent_pos[0] + dx, agent_pos[1] + dy)
        agent_heading = _normalize_angle(agent_heading + vyaw * dt)
        if done:
            break
    return actions

# ================ 分组为 points（orientation 映射；actions 为控制向量） ================
def group_actions_into_points(actions: List[Dict[str, Any]],
                              in_fmt: str,
                              out_fmt: str,
                              chunk_size: int = 5,
                              remainder: str = "keep"):
    """
    每个 point：
      - position：组首帧位置（in->out）
      - orientation：组首帧“老 rotation”（in->out；habitat输出用xyzw，isaac输出用wxyz）
      - action：
          * point   : 下一个 point 的首帧坐标（最后一个 point 无“下一个”时用自己的最后一帧）
          * straight/smooth: [[vx,vy,vyaw,dt], ...]
    列表末尾额外追加一个点：
      - {"point_id": "END", "position": 轨迹最后一帧位置（in->out）}
    """
    if not actions:
        return []

    if chunk_size is None or chunk_size < 1:
        chunk_size = 1

    def _vec(a):
        return [float(a.get("vx", 0.0)),
                float(a.get("vy", 0.0)),
                float(a.get("vyaw", 0.0)),
                float(a.get("dt", 0.0))]

    n = len(actions)
    full_end = (n // chunk_size) * chunk_size
    leftover = actions[full_end:]

    # —— 先切段（每段=一个 point）
    segments: List[List[Dict[str, Any]]] = []
    for start in range(0, full_end, chunk_size):
        segments.append(actions[start:start + chunk_size])

    if leftover:
        if remainder == "keep":
            segments.append(leftover)
        elif remainder == "merge":
            if segments:
                segments[-1].extend(leftover)
            else:
                segments.append(leftover)
        elif remainder == "drop":
            pass
        elif remainder == "pad":
            padded = list(leftover)
            zeros_needed = chunk_size - len(leftover)
            if zeros_needed > 0:
                last = leftover[-1]
                zact = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0, "dt": 0.0,
                        "pos": last.get("pos", {}), "rotation": last.get("rotation", {})}
                padded.extend([zact for _ in range(zeros_needed)])
            segments.append(padded)
        else:
            segments.append(leftover)

    # —— 预计算每段信息
    chunks_info = []
    for seg in segments:
        if not seg:
            continue
        first = seg[0]
        last  = seg[-1]
        pos_first_out = convert_position_dict(first.get("pos", {}), in_fmt, out_fmt)
        rot_wxyz      = convert_rotation_wxyz_for_orientation(first.get("rotation", {}), in_fmt, out_fmt)
        pos_last_out  = convert_position_dict(last.get("pos", {}),  in_fmt, out_fmt)
        step_vecs     = [_vec(a) for a in seg]
        chunks_info.append({
            "pos_first": pos_first_out,
            "rot_wxyz":  rot_wxyz,
            "pos_last":  pos_last_out,
            "vecs":      step_vecs,
        })

    # —— 生成普通 points（含“下一点坐标”的提示）
    pts = []
    for i, info in enumerate(chunks_info):
        pos_curr = info["pos_first"]
        rot_wxyz = info["rot_wxyz"]
        vecs     = info["vecs"]

        # 下一个点的首帧坐标；最后一个点回退到自己的最后一帧
        if i + 1 < len(chunks_info):
            next_pos = chunks_info[i + 1]["pos_first"]
        else:
            next_pos = info["pos_last"]

        pts.append({
            "point_id": i,
            "position": {"x": pos_curr["x"], "y": pos_curr["y"], "z": pos_curr["z"]},
            "orientation": format_orientation(rot_wxyz, out_fmt),
            "camera_images": [],
            "action": [
                {"type": "point",
                 "content": f"({next_pos['x']:.6f},{next_pos['y']:.6f},{next_pos['z']:.6f})"},
                {"type": "straight", "content": vecs},
                {"type": "smooth",   "content": vecs},
            ]
        })

    # —— 追加终点点：point_id="END"，只存整条轨迹的最后一帧位置
    traj_last_out = convert_position_dict(actions[-1].get("pos", {}), in_fmt, out_fmt)
    pts.append({
        "point_id": "END",
        "position": {"x": traj_last_out["x"], "y": traj_last_out["y"], "z": traj_last_out["z"]}
    })

    return pts




# ================ 指令收集 ================
def _collect_instructions(eps) -> List[str]:
    """从一组 episode 中收集全部 instruction 文本，去重保序。"""
    seen = set()
    result = []
    for ep in eps:
        instr = ep.get("instruction", None)
        # 常见格式：{"instruction_text": "..."}
        if isinstance(instr, dict):
            text = instr.get("instruction_text")
            if text and text not in seen:
                seen.add(text)
                result.append(text)
        # 兼容：直接给字符串
        elif isinstance(instr, str):
            if instr and instr not in seen:
                seen.add(instr)
                result.append(instr)
        # 兼容：列表里放字符串或 dict
        elif isinstance(instr, list):
            for item in instr:
                if isinstance(item, str):
                    if item and item not in seen:
                        seen.add(item)
                        result.append(item)
                elif isinstance(item, dict):
                    t = item.get("instruction_text")
                    if t and t not in seen:
                        seen.add(t)
                        result.append(t)
    return result

# ================ Loader & 动作生成 ================
def load_vlnce_loader(in_fmt: str):
    if in_fmt == "isaac":
        mod = importlib.import_module("utils.path_extract_isaac")
    elif in_fmt == "habitat":
        mod = importlib.import_module("utils.path_extract_habitat")
    else:
        raise ValueError('in_fmt 必须是 "isaac" 或 "habitat"')
    if not hasattr(mod, "VLNCEPathLoader"):
        raise ImportError(f"{mod.__name__} 中未找到 VLNCEPathLoader")
    return getattr(mod, "VLNCEPathLoader")

def generate_actions(dataset, input_path, in_fmt: str):
    Loader = load_vlnce_loader(in_fmt)
    loader = Loader(input_path)
    try:
        paths_dict = loader.get_all_paths_and_starts()
    except TypeError:
        paths_dict = loader.get_all_paths_and_starts()

    for ep in dataset.get("episodes", []):
        ep_id = ep.get("episode_id")
        if ep_id not in paths_dict:
            continue
        info = paths_dict[ep_id]
        path = info["path"]
        if "start_pos_2d" in info:
            init_pos = tuple(info["start_pos_2d"])
            init_y = float(info.get("start_pos", (0, 0, 0))[2])
        else:
            sx, sy, sz = info["start_pos"]
            init_pos = (sx, sy)
            init_y = sz
        init_heading = info["start_heading"]
        actions = run_pure_pursuit(path, init_pos, init_heading, init_y)
        ep["action"] = actions
    return dataset

# ================ 转为目标结构（scene_id/trajectory_id 沿用源值） ================
def convert_to_output(dataset, in_fmt: str, out_fmt: str,
                      chunk_size: int = 5, remainder: str = "keep",
                      camera_details: List[Dict[str, Any]] = None,
                      input_filename: str = ""):
    episodes = dataset.get("episodes", [])
    # 以源 scene_id 原值分组
    by_scene = defaultdict(lambda: defaultdict(list))
    for ep in episodes:
        scene_id_raw = str(ep.get("scene_id", "unknown_scene"))
        traj_id = str(ep.get("trajectory_id", ep.get("episode_id", "0")))
        by_scene[scene_id_raw][traj_id].append(ep)

    scenes_out = []
    total_traj = 0
    total_samples = 0

    for scene_id_raw in sorted(by_scene.keys()):
        trajectories_out = []
        for traj_id_raw, eps in sorted(by_scene[scene_id_raw].items(), key=lambda kv: kv[0]):
            base_ep = next((ep for ep in eps if ep.get("action")), eps[0])
            if not base_ep.get("action"):
                continue

            points = group_actions_into_points(
                base_ep["action"], in_fmt=in_fmt, out_fmt=out_fmt,
                chunk_size=chunk_size, remainder=remainder
            )

            # 收集该 trajectory 的全部指令，每条生成一个 sample
            all_instrs = _collect_instructions(eps)
            samples_list = [
                {
                    "type": "vlnce",
                    "instruction": txt,
                    "memory_per_point": []   # 可留空
                }
                for txt in all_instrs
            ]

            trajectories_out.append({
                "trajectory_id": str(traj_id_raw),  # 保留老 ID
                "points": points,
                "samples": samples_list
            })
            total_traj += 1
            total_samples += len(samples_list)

        scenes_out.append({
            "scene_id": scene_id_raw,      # 保留源 scene_id
            "trajectories": trajectories_out
        })

    meta = {
        "name": "R2R",                               # 如需改名可在此处调整
        "scene_source": input_filename or "",        # ✅ 使用输入文件名
        "input_format": in_fmt,                      # ✅ 新增
        "output_format": out_fmt,                    # ✅ 新增
        "num_scenes": len(scenes_out),
        "num_trajectories": total_traj,
        "num_samples": total_samples,
        "camera_parameters": {
            "camera_number": len(camera_details) if camera_details else 0,
            "camera_details": camera_details or []
        }
    }
    return {"dataset_metadata": meta, "scenes": scenes_out}

# ================ CLI ================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="输入 JSON 文件路径")
    parser.add_argument("--output", required=True, help="输出 JSON 文件路径")
    parser.add_argument("--in-fmt", choices=["isaac", "habitat"], default="isaac",
                        help="输入坐标系（决定使用哪个 VLNCEPathLoader）。默认 isaac。")
    parser.add_argument("--out-fmt", choices=["isaac", "habitat"], default="isaac",
                        help="输出坐标系（points 的 position/orientation）。默认 isaac。")
    parser.add_argument("--group-size", type=int, default=None,
                        help="每个 point 打包的动作数（>=1）。不填则交互输入，默认 5。")
    parser.add_argument("--tail", choices=["keep", "merge", "drop", "pad"], default="keep",
                        help="最后不足一组的处理策略。默认 keep。")
    parser.add_argument("--camera-json", type=str, default=None,
                        help="可选：相机详情 JSON 文件路径（数组）。不提供则 camera_details 留空。")
    args = parser.parse_args()

    # group-size
    if args.group_size is None:
        try:
            s = input("每个 point 打包多少个 action？(默认 5): ").strip()
            group_size = int(s) if s else 5
        except Exception:
            group_size = 5
    else:
        group_size = args.group_size
    if group_size < 1:
        print("⚠️ group-size 必须 >= 1，已改为 1")
        group_size = 1

    input_path = Path(args.input)
    output_path = Path(args.output)

    dataset = json.loads(input_path.read_text(encoding="utf-8"))

    # 可选相机详情
    camera_details = None
    if args.camera_json:
        try:
            camera_details = json.loads(Path(args.camera_json).read_text(encoding="utf-8"))
            if not isinstance(camera_details, list):
                print("⚠️ --camera-json 内容应为数组，已忽略。")
                camera_details = None
        except Exception as e:
            print(f"⚠️ 读取 --camera-json 失败：{e}，将留空。")
            camera_details = None

    print(f"🚀 运行 & 转换（in={args.in_fmt} → out={args.out_fmt}；group-size={group_size}；tail={args.tail}）...")
    dataset_with_actions = generate_actions(dataset, str(input_path), in_fmt=args.in_fmt)
    out_json = convert_to_output(dataset_with_actions,
                                 in_fmt=args.in_fmt, out_fmt=args.out_fmt,
                                 chunk_size=group_size, remainder=args.tail,
                                 camera_details=camera_details,
                                 input_filename=input_path.name)   # ✅ 传入输入文件名

    output_path.write_text(json.dumps(out_json, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"✅ 输出已保存至：{args.output}")
