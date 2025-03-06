import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import json
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description='Visualize delivery routes')
    parser.add_argument('--data', type=str, required=True, help='Path to the data file')
    parser.add_argument('--result', type=str, required=True, help='Path to the result file')
    parser.add_argument('--output', type=str, default='route_visualization.png', help='Output image path')
    return parser.parse_args()

def read_data_file(filepath):
    """读取原始数据文件，提取任务点和配送中心的坐标"""
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # 解析第一行的基本参数
    params = list(map(int, lines[0].strip().split()))
    initial_demand_count = params[0]
    extra_demand_count = params[1]
    vehicle_center_count = params[2]
    drone_center_count = params[3]
    
    # 跳过参数行和路网信息
    line_index = 3  # 跳过基本参数和路网参数
    
    # 跳过路网边信息
    edge_count = int(lines[2].strip())
    line_index += edge_count
    
    # 读取初始需求点
    initial_points = []
    for i in range(initial_demand_count):
        data = lines[line_index].strip().split()
        id, lon, lat = int(data[0]), float(data[1]), float(data[2])
        initial_points.append((id, lon, lat))
        line_index += 1
    
    # 读取配送中心
    centers = []
    # 先读车辆配送中心
    for i in range(vehicle_center_count):
        data = lines[line_index].strip().split()
        id, lon, lat, count = int(data[0]), float(data[1]), float(data[2]), int(data[3])
        centers.append((id, lon, lat, count, 'vehicle'))
        line_index += 1
    
    # 再读无人机配送中心
    for i in range(drone_center_count):
        data = lines[line_index].strip().split()
        id, lon, lat, count = int(data[0]), float(data[1]), float(data[2]), int(data[3])
        centers.append((id, lon, lat, count, 'drone'))
        line_index += 1
    
    # 读取额外需求点
    extra_points = []
    for i in range(extra_demand_count):
        data = lines[line_index].strip().split()
        id, lon, lat, time = int(data[0]), float(data[1]), float(data[2]), float(data[3])
        extra_points.append((id, lon, lat, time))
        line_index += 1
    
    return initial_points, extra_points, centers

def read_result_file(filepath):
    """读取结果文件，提取配送路径"""
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    routes = []
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("配送工具"):
            route_info = {}
            # 解析配送工具ID和类型
            parts = line.split("(")
            route_info["id"] = int(parts[0].split()[1])
            route_info["type"] = "drone" if "无人机" in parts[1] else "vehicle"
            
            # 读取任务序列
            i += 1
            if i < len(lines) and "任务序列" in lines[i]:
                tasks = list(map(int, lines[i].split(":")[1].strip().split()))
                route_info["tasks"] = tasks
            
            routes.append(route_info)
        i += 1
    
    return routes

def visualize_routes(initial_points, extra_points, centers, routes, output_path):
    """可视化配送路径"""
    plt.figure(figsize=(12, 10))
    
    # 绘制任务点
    for id, lon, lat in initial_points:
        plt.plot(lon, lat, 'bo', markersize=6)
        plt.text(lon, lat, str(id), fontsize=8)
    
    # 绘制额外需求点
    for id, lon, lat, _ in extra_points:
        plt.plot(lon, lat, 'go', markersize=6)
        plt.text(lon, lat, str(id), fontsize=8)
    
    # 绘制配送中心
    for id, lon, lat, _, center_type in centers:
        if center_type == 'vehicle':
            plt.plot(lon, lat, 'rs', markersize=10)
        else:
            plt.plot(lon, lat, 'ms', markersize=10)
        plt.text(lon, lat, str(id), fontsize=10, fontweight='bold')
    
    # 创建任务点和配送中心的坐标映射
    point_coords = {}
    for id, lon, lat in initial_points:
        point_coords[id] = (lon, lat)
    
    for id, lon, lat, _ in extra_points:
        point_coords[id] = (lon, lat)
    
    for id, lon, lat, _, _ in centers:
        point_coords[id] = (lon, lat)
    
    # 绘制配送路径
    colors = plt.cm.tab10.colors
    for i, route in enumerate(routes):
        if "tasks" in route and len(route["tasks"]) > 1:
            tasks = route["tasks"]
            for j in range(len(tasks) - 1):
                if tasks[j] in point_coords and tasks[j+1] in point_coords:
                    x1, y1 = point_coords[tasks[j]]
                    x2, y2 = point_coords[tasks[j+1]]
                    color = colors[i % len(colors)]
                    plt.plot([x1, x2], [y1, y2], '-', color=color, linewidth=1.5, alpha=0.7)
    
    # 添加图例和标题
    plt.plot([], [], 'bo', label='Initial Demand Points')
    plt.plot([], [], 'go', label='Extra Demand Points')
    plt.plot([], [], 'rs', label='Vehicle Centers')
    plt.plot([], [], 'ms', label='Drone Centers')
    plt.legend(loc='best')
    
    plt.title("Delivery Routes Visualization")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    
    # 保存图像
    plt.savefig(output_path, dpi=300)
    print(f"Visualization saved to {output_path}")
    
    # 显示图像
    plt.show()

def main():
    args = parse_arguments()
    
    # 检查文件是否存在
    if not os.path.exists(args.data):
        print(f"Error: Data file {args.data} not found.")
        return
    
    if not os.path.exists(args.result):
        print(f"Error: Result file {args.result} not found.")
        return
    
    # 读取数据
    initial_points, extra_points, centers = read_data_file(args.data)
    routes = read_result_file(args.result)
    
    # 可视化
    visualize_routes(initial_points, extra_points, centers, routes, args.output)

if __name__ == "__main__":
    main() 