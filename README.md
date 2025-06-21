# 无人机配送网络设计求解器

这个项目实现了一个无人机配送网络的优化求解器，用于解决带有中间充电站的点对点无人机配送网络设计问题。

## 功能特点

- 支持多个配送枢纽和中间充电站的网络设计
- 使用整数规划模型进行优化求解
- 提供网络可视化功能
- 支持多种求解器选项（CPLEX, CBC等）

## 安装说明

1. 克隆仓库：
```bash
git clone [你的仓库URL]
cd UVA_Network
```

2. 安装依赖：
```bash
pip install -r requirements.txt
```

3. 安装求解器：
   - 如果使用CBC（开源）：系统会自动安装
   - 如果使用CPLEX：需要单独安装CPLEX优化器

## 使用方法

1. 运行示例：
```bash
python run_experiment.py
```

2. 主要参数说明：
   - `num_hubs`：配送枢纽数量
   - `num_candidates`：候选充电站数量
   - `theta`：成本权重参数
   - `m`：路径数量限制

## 项目结构

- `drone_net_solver/`：核心求解器模块
  - `model.py`：优化模型定义
  - `graph_utils.py`：图形处理工具
  - `path_enum.py`：路径枚举算法
- `network_generator.py`：网络实例生成器
- `run_experiment.py`：运行示例的主程序

## 引用

如果您在研究中使用了这个项目，请引用相关论文：[论文引用信息]

## 许可证

[选择合适的许可证]