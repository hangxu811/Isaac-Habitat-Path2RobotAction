# Isaac-Habitat-Path2RobotAction
This is a tool for converting paths (formed by some points with locations, and start rotation) to a series of smooth and relatively continued actions.
Trajectory Converter (Isaac/Habitat → Unified JSON)
将来自 Isaac / Habitat 的轨迹与指令，转换为统一结构的 JSON 文件；自动生成 pure_pursuit 动作序列，支持分组、尾部策略与相机元数据注入。

功能
支持 输入/输出坐标系 选择（--in-fmt isaac|habitat、--out-fmt isaac|habitat）

scene_id / trajectory_id 原样保留（沿用源文件）

orientation 只做坐标系转换：

输出为 habitat → {"x","y","z","w"}（xyzw）

输出为 isaac → {"w","x","y","z"}（wxyz）

pure_pursuit 生成逐步动作：[vx, vy, vyaw, dt]

straight / smooth 都保存完整控制序列

action 的 point 保存下一个 point 的首帧坐标（末段回退本段最后一帧）

points 末尾自动追加 终点点：{"point_id": "END", "position": ...}

同一条轨迹的 所有指令 都会各自写成一条 samples[*]

可配置 分组（--group-size）与 尾部策略（--tail keep|merge|drop|pad）

可选注入 camera_parameters（--camera-json）

dataset_metadata 里记录：
scene_source = 输入文件名、input_format、output_format、统计信息等

快速开始
环境要求
Python 3.8+

建议：numpy、argparse（随 Python 标配）、你的 utils 目录包含：

utils/path_extract_isaac.py、utils/path_extract_habitat.py（提供 VLNCEPathLoader）

utils/pure_pursuit_dynamic_ld.py（提供 PurePursuitFollower2D）

utils/rotation_isaac2habitat.py（提供 Isaac2HabitatConverter）

安装
bash
复制
编辑
git clone <your_repo_url>.git
cd <your_repo>
# 可选：如果有 requirements.txt
pip install -r requirements.txt
运行
方式一：Isaac → Habitat（常用）

bash
复制
编辑
python converter.py \
  --input input.json \
  --output out_hab.json \
  --in-fmt isaac \
  --out-fmt habitat \
  --group-size 5 \
  --tail keep
方式二：Habitat → Isaac

bash
复制
编辑
python converter.py --input input.json --output out_isaac.json --in-fmt habitat --out-fmt isaac
方式三：同系转换（坐标不变）

bash
复制
编辑
python converter.py --input input.json --output out.json --in-fmt isaac --out-fmt isaac
# 或
python converter.py --input input.json --output out.json --in-fmt habitat --out-fmt habitat
注入相机参数

bash
复制
编辑
python converter.py \
  --input input.json \
  --output out.json \
  --in-fmt isaac --out-fmt habitat \
  --camera-json camera_details.json
配置
命令行参数
--input：输入 JSON 文件（包含 episodes）

--output：输出 JSON 文件

--in-fmt：输入坐标系（isaac / habitat）

--out-fmt：输出坐标系（isaac / habitat）

--group-size：每个 point 打包的动作数（≥1；1 表示逐帧一个 point）

--tail：尾部不足一组的处理

keep（默认）：单独成组

merge：并入上一组

drop：丢弃尾巴

pad：用 [0,0,0,0] 补齐到 group-size

--camera-json：相机详情数组文件（可选），写入 dataset_metadata.camera_parameters

相机配置（可选）
camera_details.json（数组）示例：

json
复制
编辑
[
  {
    "camera_id": 0,
    "position": {"x": 0, "y": 0, "z": 0},
    "orientation": {"w": 0, "x": 0, "y": 0, "z": 0},
    "fov": {"H": 69, "V": 58},
    "resolution": {"width": 1920, "height": 1080}
  }
]
输入 / 输出示例
输入最小示例（节选）
json
复制
编辑
{
  "episodes": [
    {
      "scene_id": "mp3d/house_1/xxx.scene",
      "trajectory_id": "traj_0001",
      "episode_id": "0001",
      "instruction": {"instruction_text": "Go to the kitchen."}
      // 其余由 loader（VLNCEPathLoader）推断 path/start/heading
    }
  ]
}
输出结构（节选，字段关键位）
json
复制
编辑
{
  "dataset_metadata": {
    "name": "R2R",
    "scene_source": "input.json",
    "input_format": "isaac",
    "output_format": "habitat",
    "num_scenes": 1,
    "num_trajectories": 1,
    "num_samples": 1,
    "camera_parameters": {
      "camera_number": 1,
      "camera_details": [/* 来自 --camera-json */]
    }
  },
  "scenes": [
    {
      "scene_id": "mp3d/house_1/xxx.scene",
      "trajectories": [
        {
          "trajectory_id": "traj_0001",
          "points": [
            {
              "point_id": 0,
              "position": {"x": 1.234, "y": 0.000, "z": 2.345},
              "orientation": {"x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071},
              "camera_images": [],
              "action": [
                {"type": "point",    "content": "(1.567000,0.000000,2.890000)"},
                {"type": "straight", "content": [[vx, vy, vyaw, dt], "..."]},
                {"type": "smooth",   "content": [[vx, vy, vyaw, dt], "..."]}
              ]
            },
            /* ... 更多 point ... */
            {
              "point_id": "END",
              "position": {"x": 3.210, "y": 0.000, "z": 4.321}
            }
          ],
          "samples": [
            {
              "type": "vlnce",
              "instruction": "Go to the kitchen.",
              "memory_per_point": []
            }
            /* 同一条轨迹如有多条指令，会各自生成一条 sample */
          ]
        }
      ]
    }
  ]
}
示例文件
建议准备：

输入示例：examples/input_isaac.json、examples/input_habitat.json

相机示例：examples/camera_details.json

输出示例：examples/out_hab.json、examples/out_isaac.json

目录结构示例：

pgsql
复制
编辑
.
├── converter.py
├── utils/
│   ├── path_extract_isaac.py
│   ├── path_extract_habitat.py
│   ├── pure_pursuit_dynamic_ld.py
│   └── rotation_isaac2habitat.py
└── examples/
    ├── input_isaac.json
    ├── input_habitat.json
    ├── camera_details.json
    └── out_hab.json
常见问题
Q1. orientation 为什么有两种键名？
A1. 为了兼容不同引擎：输出为 habitat 时用 xyzw；输出为 isaac 时用 wxyz。数值仅做坐标系变换，不做欧拉角或角度缩放。

Q2. 终点怎么表示？
A2. 在 points 末尾追加 {"point_id": "END", "position": ...}，只存整条轨迹最后一帧的位置。

Q3. group-size 怎么选？

1：每帧一个 point（最细粒度）

N>1：每 N 帧合并为一个 point

想把整条轨迹合为一个 point，可把 group-size 设为足够大的数，并配合 --tail merge/keep。

Q4. --tail 有何区别？

keep：尾部不足 group-size 的帧单独成组

merge：并入上一组

drop：丢弃尾部

pad：用 [0,0,0,0] 补齐到 group-size

Q5. Habitat→Isaac 会不会多转 +90°？
A5. 不会。该分支现在不额外旋转，直接透传四元数（坐标系转换由上游/loader 保证）。

Q6. camera_images、memory_per_point 为什么是空的？
A6. 这两个字段预留给你后续填充图像路径和记忆/检索内容；当前版本不主动生成。

