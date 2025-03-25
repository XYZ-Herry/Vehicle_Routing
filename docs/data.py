import pandas as pd
import random
import numpy as np
import argparse

# 创建参数解析器
parser = argparse.ArgumentParser(description="生成需求和边的模拟数据")

# 添加命令行参数
parser.add_argument("-vu", type=int, default=20, help="无人机速度 (默认 20)")
parser.add_argument("-vc", type=int, default=10, help="车辆速度 (默认 10)")
parser.add_argument("-cu", type=float, default=0.84, help="无人机运送货物的成本 (默认 0.84)")
parser.add_argument("-cc", type=float, default=0.62, help="货车运送货物的成本 (默认 0.62)")
parser.add_argument("-om", type=float, default=0.1, help="配送时间权重 (默认 0.1)")
parser.add_argument("-qu", type=int, default=20, help="无人机最大载重 (默认 20)")

# 解析命令行参数
args = parser.parse_args()

# 默认参数（可以通过命令行参数修改）
params = {
    "vu": args.vu,  # 无人机速度
    "vc": args.vc,  # 车辆速度
    "cu": args.cu,  # 无人机运送货物的成本
    "cc": args.cc,  # 货车运送货物的成本
    "om": args.om,  # 配送时间权重
    "qu": args.qu   # 无人机最大载重
}

# 帮助函数：如果有-help参数，显示参数说明
def print_help():
    print("""
    1. default_require_num: 默认需求数量，表示要随机选取多少个需求节点。
    2. additional_demand_num: 额外需求数量，表示额外随机选取的需求数量。
    3. car_num: 车辆数量，表示要生成的车辆数量。
    4. UAV_num: 无人机数量，表示要生成的无人机数量。
    5. 无人机速度 (vu): 无人机的运输速度。
    6. 车辆速度 (vc): 车辆的运输速度。
    7. 无人机运送货物的成本 (cu): 无人机运送货物的单位成本。
    8. 货车运送货物的成本 (cc): 货车运送货物的单位成本。
    9. 无人机最大载重 (qu): 无人机的最大载重。
    10. 配送时间权重 (om): 配送时间对整体成本的影响权重。
    """)

# 读取数据
require_df = pd.read_excel("require.xlsx")
road_new_df = pd.read_excel("road_new.xlsx")  # 使用新文件

# 处理道路数据
def clean_node_pair(pair):
    try:
        return tuple(map(int, pair.split('，')))  # 不进行排序，保留原始节点对
    except:
        return None

road_new_df['sorted_nodes'] = road_new_df['路段起终点编号'].apply(clean_node_pair)
road_new_df = road_new_df.dropna(subset=['sorted_nodes'])

# 获取所有边的信息
all_edges_df = road_new_df[['sorted_nodes', '路段长度（米）']].reset_index(drop=True)
all_edges_df[['节点1', '节点2']] = pd.DataFrame(all_edges_df['sorted_nodes'].tolist(), index=all_edges_df.index)

# 用户输入部分
default_require_num = int(input("请输入默认需求数量 (小于143): "))
additional_demand_num = int(input("请输入额外需求数量 (小于143): "))
car_num = int(input("请输入车辆数量: "))
UAV_num = int(input("请输入无人机数量: "))

# 获取需求节点
require_nodes = require_df[['序号', 'X', 'Y坐标', '需求量']]

# 根据需求量作为权重进行节点选择
require_nodes['权重'] = require_nodes['需求量'] / require_nodes['需求量'].sum()

# 根据权重进行采样，确保按照需求量为权重来选择节点
selected_require_nodes_weighted = np.random.choice(require_nodes.index, size=default_require_num, p=require_nodes['权重'], replace=False)
selected_additional_nodes_weighted = np.random.choice(require_nodes.index, size=additional_demand_num, p=require_nodes['权重'], replace=False)

# 车辆起始位置分布
all_nodes = np.unique(all_edges_df[['节点1', '节点2']].values.flatten())
car_start_nodes = random.sample(list(all_nodes), car_num)
UAV_start_nodes = random.sample(list(all_nodes), UAV_num)

# 生成额外需求
additional_demand_weighted = []
for node_idx in selected_additional_nodes_weighted:
    demand_node = require_nodes.iloc[node_idx]
    task_time = random.randint(1, 1440)  # 任务生成时间
    additional_demand_weighted.append([demand_node['序号'], demand_node['X'], demand_node['Y坐标'], task_time])

# 对额外需求按任务生成时间排序
additional_demand_weighted.sort(key=lambda x: x[3])

# 生成输出文件内容
output_data_weighted = []

# 第一行：default_require_num Additional demand num car_num UAV_num
output_data_weighted.append(f"{default_require_num} {additional_demand_num} {car_num} {UAV_num}")

# 第二行：无人机速度 车辆速度 无人机运送货物的成本 货车运送货物的成本 无人机最大载重 配送时间权重
output_data_weighted.append(f"{params['vu']} {params['vc']} {params['cu']} {params['cc']} {params['qu']} {params['om']}")

# 第三行：边的数量 road_num
output_data_weighted.append(str(len(all_edges_df)))

# 输出所有边的信息，包括重复边
for _, row in all_edges_df.iterrows():
    output_data_weighted.append(f"{row['节点1']} {row['节点2']} {row['路段长度（米）']}")

# 输出default_require_num个需求
for node_idx in selected_require_nodes_weighted:
    demand_node = require_nodes.iloc[node_idx]
    # 保证序号为整数输出
    output_data_weighted.append(f"{int(demand_node['序号'])} {demand_node['X']} {demand_node['Y坐标']}")

# 输出车辆起始位置（包含坐标）并附加随机数
for node in car_start_nodes:
    car_node = require_nodes[require_nodes['序号'] == node].iloc[0]
    random_value = random.randint(1, 5)  # 生成1到5之间的随机数
    output_data_weighted.append(f"{node} {car_node['X']} {car_node['Y坐标']} {random_value}")

# 输出无人机起始位置（包含坐标）并附加随机数
for node in UAV_start_nodes:
    UAV_node = require_nodes[require_nodes['序号'] == node].iloc[0]
    random_value = random.randint(1, 5)  # 生成1到5之间的随机数
    output_data_weighted.append(f"{node} {UAV_node['X']} {UAV_node['Y坐标']} {random_value}")

# 输出额外需求信息
for demand in additional_demand_weighted:
    output_data_weighted.append(f"{int(demand[0])} {demand[1]} {demand[2]} {demand[3]}")

# 保存结果到更新后的文件
output_file_path_weighted = "output_data_weighted.txt"
with open(output_file_path_weighted, 'w') as f:
    for line in output_data_weighted:
        f.write(line + "\n")

print(f"文件已保存为 {output_file_path_weighted}")