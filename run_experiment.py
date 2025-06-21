"""
Demo: generate instance → solve with m-SP ILP → print results.
"""
from network_generator import generate_instance, plot_instance
from drone_net_solver import DroneNetworkModel

def main():
    inst = generate_instance(num_hubs=3,
                             num_candidates=60,
                             seed=20250615)
    plot_instance(inst)                      # 原脚本自带可视化
    model = DroneNetworkModel(inst, theta=0.5, m=50)
    model.solve(solver="CPLEX", msg=1)       # 无 CPLEX 时自动回退 CBC
    sol = model.extract()

    print("\n=== Solution summary ===")
    for k,v in sol.items():
        if k=="chosen_paths":
            print(f"{k:>24}: {len(v)} paths")
        else:
            print(f"{k:>24}: {v}")

if __name__ == "__main__":
    main()
