"""
Generate random instances for the drone delivery network design problem.
"""
import numpy as np
import random, math, itertools
from typing import List, Tuple, Dict

import matplotlib.pyplot as plt

# Point类型定义为浮点数二维坐标
# Point type definition as 2D coordinates with float values
Point = Tuple[float, float]

def _rand_point(L: float) -> Point:
    """均匀产生一个二维坐标
    Uniformly generate a 2D coordinate
    
    Args:
        L (float): 正方形区域的边长 / Side length of the square area
        
    Returns:
        Point: 随机生成的坐标点 / Randomly generated coordinate point
    """
    return random.uniform(0, L), random.uniform(0, L)

def _dist(a: Point, b: Point) -> float:
    """计算两点间的欧氏距离
    Calculate Euclidean distance between two points
    
    Args:
        a (Point): 第一个点 / First point
        b (Point): 第二个点 / Second point
        
    Returns:
        float: 两点间的距离 / Distance between points
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])

def _connected(points: List[Point], R: int) -> bool:
    """判断在边长 ≤ 2R 的阈值内，点集是否连通（使用BFS算法）
    Check if the point set is connected within threshold ≤ 2R (using BFS algorithm)
    
    Args:
        points (List[Point]): 需要检查的点集 / Point set to check
        R (int): 半径参数 / Radius parameter
        
    Returns:
        bool: 是否连通 / Whether connected
    """
    n = len(points)
    reach = [False] * n  # 记录每个点是否可达 / Record if each point is reachable
    reach[0] = True
    stack = [0]
    thr = 2 * R + 1e-9  # 添加小量以处理浮点误差 / Add small value to handle float errors
    
    while stack:
        v = stack.pop()
        for u in range(n):
            if not reach[u] and _dist(points[v], points[u]) <= thr:
                reach[u] = True
                stack.append(u)
    return all(reach)

def _count_uncovered(cover: List[List[bool]]) -> int:
    """计算未被覆盖的配送点数量
    Calculate the number of uncovered delivery points
    
    Args:
        cover (List[List[bool]]): 覆盖矩阵 / Coverage matrix
        
    Returns:
        int: 未覆盖的配送点数量 / Number of uncovered delivery points
    """
    if not cover or not cover[0]:
        return 0
    num_charging_stations = len(cover)
    num_delivery_points = len(cover[0])
    uncovered = 0
    for j in range(num_delivery_points):
        if not any(cover[i][j] for i in range(num_charging_stations)):
            uncovered += 1
    return uncovered

def generate_instance(num_hubs: int,
                    num_candidates: int,
                    square_size: float = 100.0,
                    seed: int | None = None,
                    max_attempts: int = 1000) -> Dict:
    """生成一个网络覆盖问题算例
    Generate a network coverage problem instance
    
    Args:
        num_hubs (int): 枢纽数量 / Number of hubs
        num_candidates (int): 候选充电站数量 / Number of candidate charging stations
        square_size (float): 正方形区域边长 / Square area side length
        seed (int | None): 随机种子 / Random seed
        max_attempts (int): 最大尝试次数 / Maximum attempts
        
    Returns:
        Dict: 包含问题实例的字典 / Dictionary containing problem instance
            - hubs: 枢纽位置列表 / List of hub locations
            - charging_stations: 候选充电站位置列表 / List of candidate charging station locations
            - delivery_points: 配送点位置列表 / List of delivery point locations
            - R: 覆盖半径 / Coverage radius
            - cover: 覆盖矩阵 / Coverage matrix
    """
    if seed is not None:
        random.seed(seed)

    # 1. 生成枢纽点S和候选充电站I / Generate hub points S and candidate charging stations I
    hubs = [_rand_point(square_size) for _ in range(num_hubs)]
    charging_stations = [_rand_point(square_size) for _ in range(num_candidates)]
    gamma = hubs + charging_stations

    # 2. 搜索最小R使S∪I连通 / Search minimum R that makes S∪I connected
    R = 1
    while not _connected(gamma, R):
        R += 1

    # 3. 生成配送点直到找到足够的被覆盖点 / Generate delivery points until finding enough covered points
    for attempt in range(1, max_attempts + 1):
        # 生成2|I|个配送点 / Generate 2|I| delivery points
        temp_delivery_points = [_rand_point(square_size) for _ in range(2 * num_candidates)]
        
        # 计算覆盖矩阵C(i,n) / Calculate coverage matrix C(i,n)
        temp_cover = [[_dist(st, dp) <= R for dp in temp_delivery_points] for st in charging_stations]
        
        # 找出被覆盖的配送点 / Find covered delivery points
        covered_indices = []
        for j in range(len(temp_delivery_points)):
            if any(temp_cover[i][j] for i in range(len(charging_stations))):
                covered_indices.append(j)
        
        # 如果找到足够的被覆盖点 / If found enough covered points
        if len(covered_indices) >= num_candidates:
            # 保留前|I|个被覆盖的点 / Keep the first |I| covered points
            selected_indices = covered_indices[:num_candidates]
            delivery_points = [temp_delivery_points[i] for i in selected_indices]
            # 更新覆盖矩阵 / Update coverage matrix
            cover = [[temp_cover[i][j] for j in selected_indices] for i in range(len(charging_stations))]
            break
    else:
        raise RuntimeError(f"达到最大尝试次数仍未找到{num_candidates}个被覆盖的配送点（当前找到：{len(covered_indices)}个）/ "
                         f"Reached max_attempts without finding {num_candidates} covered delivery points (found: {len(covered_indices)})")

    return {
        "hubs": hubs,
        "charging_stations": charging_stations,
        "delivery_points": delivery_points,
        "R": R,
        "cover": cover,
        "uncovered": 0  # 由于我们只保留被覆盖的点，所以一定是0 / Since we only keep covered points, this is always 0
    }

def plot_instance(inst: Dict) -> None:
    """Visualize the network coverage problem instance
    
    Args:
        inst (Dict): Problem instance dictionary
    """
    plt.figure(figsize=(10, 10))
    
    delivery_points = np.array(inst['delivery_points'])
    charging_stations = np.array(inst['charging_stations'])
    hubs = np.array(inst['hubs'])
    cover_matrix = inst['cover']
    R = inst['R']

    # Plot covered delivery points (blue dots)
    covered_delivery_points_mask = np.array([any(cover_matrix[s_idx][d_idx] for s_idx in range(len(charging_stations))) for d_idx in range(len(delivery_points))])
    if np.any(covered_delivery_points_mask):
        plt.scatter(delivery_points[covered_delivery_points_mask, 0], delivery_points[covered_delivery_points_mask, 1], c='blue', marker='.', label='Covered Delivery Points')
    
    # Find charging stations that cover at least one delivery point
    active_charging_stations = np.array([any(cover_matrix[s_idx]) for s_idx in range(len(charging_stations))])
    
    # Plot candidate charging stations (green triangles) and coverage circles for active stations
    plt.scatter(charging_stations[:, 0], charging_stations[:, 1], c='green', marker='^', label='Charging Stations')
    for idx, station_pos in enumerate(charging_stations):
        if active_charging_stations[idx]:
            circle = plt.Circle(station_pos, R, color='green', fill=False, alpha=0.3)
            plt.gca().add_patch(circle)
    
    # Plot hubs (red stars)
    plt.scatter(hubs[:, 0], hubs[:, 1], c='red', marker='*', s=200, label='Hubs')
    
    # Draw lines between hubs and charging stations if distance <= 2R
    for hub in hubs:
        for station in charging_stations:
            distance = np.sqrt(np.sum((hub - station) ** 2))
            if distance <= 2 * R:
                plt.plot([hub[0], station[0]], [hub[1], station[1]], 'k--', alpha=0.3)
    
    plt.grid(True)
    plt.legend()
    plt.title(f'Network Coverage Instance\n(R={R}, |S|={len(hubs)}, |I|={len(charging_stations)}, |N|={len(delivery_points)}, Covered N={np.sum(covered_delivery_points_mask)})')
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.axis('equal')
    plt.show()

# ============ 示例调用 / Example usage ============
if __name__ == "__main__":
    import numpy as np
    inst = generate_instance(num_hubs=3, num_candidates=75, seed=2025)
    print(f"\n实例生成结果 / Instance generation results:")
    print(f"- 覆盖半径 R = {inst['R']} / Coverage radius")
    print(f"- 枢纽数量 |S| = {len(inst['hubs'])} / Number of hubs")
    print(f"- 候选充电站数量 |I| = {len(inst['charging_stations'])} / Number of candidate charging stations")
    print(f"- 配送点数量 |N| = {len(inst['delivery_points'])} / Number of delivery points")
    print(f"- 未覆盖点数 = {inst['uncovered']} ({inst['uncovered']/len(inst['delivery_points'])*100:.1f}%) / ")
    print(f"  Uncovered points = {inst['uncovered']} ({inst['uncovered']/len(inst['delivery_points'])*100:.1f}%)")
    
    # 可视化实例 / Visualize instance
    plot_instance(inst)