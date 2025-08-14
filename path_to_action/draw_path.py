from utils.path_extract_isaac import VLNCEPathLoader
#from utils.path_extract_habitat import VLNCEPathLoader
#from utils.pure_pursuit import PurePursuitFollower2D
from utils.path_visualizer import PathFollowerVisualizer2D
from utils.pure_pursuit_dynamic_ld import PurePursuitFollower2D

# 可视化代码，文件格式转化不需要使用这个代码
# 没法通过SSH 可视化 因为机器人没有图形界面
# 对应的episode ID 为原json文件内的ID



if __name__ == "__main__":

    # 1. 加载路径
    loader = VLNCEPathLoader("vln_ce_isaac_v1.json")
    paths_dict = loader.get_all_paths_and_starts()
    print("可用 episode_id 列表：", list(paths_dict.keys())[:5])

    # 2. 选择一个 episode ID
    episode_id = int(input())
    if episode_id not in paths_dict:
        print(f"❌ Episode {episode_id} 不存在，请检查 ID")
        exit(1)
    info = paths_dict[episode_id]
    print(info)
    path = info["path"]
    init_pos = info["start_pos"]
    init_heading = info["start_heading"]


    # 3. 初始化 Pure Pursuit 控制器
    follower = PurePursuitFollower2D(path, control_dt=0.3)
    visualizer = PathFollowerVisualizer2D(
        path=path,
        follower=follower,
        init_pos=init_pos,
        init_heading=init_heading,  # ✅ 使用真实初始朝向
        dt=0.3
    )
    visualizer.run()  # 启动可视化