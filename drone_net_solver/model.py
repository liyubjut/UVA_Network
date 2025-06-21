"""
m-Shortest-Path heuristic ILP (Eq. (24)–(29) of the paper).
"""
from __future__ import annotations
print("Executing model.py: top")
from typing import Dict, List, Tuple
import itertools, pulp, networkx as nx
from .graph_utils import build_graph
from .path_enum import k_shortest_simple_paths

print("Executing model.py: before class DroneNetworkModel")
class DroneNetworkModel:
    def __init__(self,
                 inst: Dict,
                 theta: float = .5,
                 m: int = 50,
                 length_cap: float | None = None):
        self.inst = inst
        self.theta, self.m, self.length_cap = theta, m, length_cap
        self.G = build_graph(inst["hubs"], inst["charging_stations"], inst["R"])
        self._enumerate_paths()
        self._build_ilp()

    # ---------- step 1: path enumeration ----------
    def _enumerate_paths(self):
        hubs, stations = self.inst["hubs"], self.inst["charging_stations"]
        self.path_list: List[Tuple[int, int, List[str], float]] = []
        for s in range(len(hubs)):
            for i in range(len(stations)):
                s_node, i_node = f"S{s}", f"I{i}"
                if nx.has_path(self.G, s_node, i_node):
                    for p, plen in k_shortest_simple_paths(
                            self.G, s_node, i_node, self.m, self.length_cap):
                        self.path_list.append((s, i, p, plen))

        I = len(stations)
        self.delta = [[0]*I for _ in self.path_list]
        for k, (_, _, nodes, _) in enumerate(self.path_list):
            for node in nodes:
                if node.startswith("I"):
                    self.delta[k][int(node[1:])] = 1

    # ---------- step 2: ILP ----------
    def _build_ilp(self):
        hubs, stations = self.inst["hubs"], self.inst["charging_stations"]
        cover = self.inst["cover"]
        I, N = len(stations), len(self.inst["delivery_points"])

        beta1 = sum(
            nx.shortest_path_length(self.G, f"S{s}", f"I{i}", weight="weight")
            for s,i in itertools.product(range(len(hubs)), range(I))
            if nx.has_path(self.G, f"S{s}", f"I{i}")
        )
        beta2 = I

        self.prob = pulp.LpProblem("DroneNetwork", pulp.LpMinimize)
        self.tp = [pulp.LpVariable(f"tp_{k}", 0,1, cat="Binary")
                   for k in range(len(self.path_list))]
        self.qi = [pulp.LpVariable(f"q_{i}", 0,1, cat="Binary") for i in range(I)]
        self.xi = [pulp.LpVariable(f"x_{i}", 0,1, cat="Binary") for i in range(I)]

        # objective (24)
        self.prob += (
            self.theta / beta1 * pulp.lpSum(self.tp[k]*self.path_list[k][3]
                                            for k in range(len(self.tp)))
            + (1-self.theta) / beta2 * pulp.lpSum(self.xi)
        )
        # (25)
        for i in range(I):
            self.prob += pulp.lpSum(
                self.tp[k] for k,(_,i_idx,_,_) in enumerate(self.path_list)
                if i_idx==i) == self.qi[i]
        # (26)
        M_big = I
        for i in range(I):
            self.prob += pulp.lpSum(
                self.delta[k][i]*self.tp[k] for k in range(len(self.tp))
            ) <= M_big*self.xi[i]
        # (27)
        for n in range(N):
            self.prob += pulp.lpSum(int(cover[i][n])*self.qi[i] for i in range(I)) >= 1

    # ---------- step 3: solve ----------
    def solve(self, solver:str="CPLEX", msg:int=1, time_limit:int|None=None):
        solver_to_try = None
        if solver.upper() == "CPLEX":
            # Check if CPLEX_PY is available
            if pulp.CPLEX_PY().available():
                solver_to_try = pulp.CPLEX_PY(msg=msg, timeLimit=time_limit)
            # Else, check if CPLEX_CMD is available
            elif pulp.CPLEX_CMD().available():
                solver_to_try = pulp.CPLEX_CMD(msg=msg, timeLimit=time_limit)
            else:
                print("CPLEX solver not found, attempting to use CBC.")
                # Fallback to CBC if CPLEX is not available
                if pulp.PULP_CBC_CMD().available():
                    solver_to_try = pulp.PULP_CBC_CMD(msg=msg, timeLimit=time_limit)
                else:
                    print("CBC solver not found. Please install a solver or check your PuLP configuration.")
                    return # Or raise an error
        elif solver.upper() == "CBC":
            if pulp.PULP_CBC_CMD().available():
                solver_to_try = pulp.PULP_CBC_CMD(msg=msg, timeLimit=time_limit)
            else:
                print("CBC solver not found. Please install CBC or check your PuLP configuration.")
                return # Or raise an error
        else: # Try default solver if specified solver is not CPLEX or CBC
            print(f"Solver '{solver}' not explicitly supported, trying PuLP's default solver.")
            solver_to_try = pulp.LpSolverDefault
            if solver_to_try is None:
                 print("PuLP default solver not available. Please install a solver (e.g., CBC, GLPK) or check your PuLP configuration.")
                 return # Or raise an error

        if solver_to_try:
            try:
                self.prob.solve(solver_to_try)
                print(f"Successfully solved with {solver_to_try.name}")
            except pulp.PulpSolverError as e:
                print(f"Error solving with {solver_to_try.name}: {e}")
                # Attempt CBC as a last resort if not already tried and it's available
                if not (solver.upper() == "CBC" and isinstance(solver_to_try, pulp.apis.coin_api.PULP_CBC_CMD)) and pulp.PULP_CBC_CMD().available():
                    print("Attempting to solve with CBC as a fallback.")
                    try:
                        self.prob.solve(pulp.PULP_CBC_CMD(msg=msg, timeLimit=time_limit))
                        print("Successfully solved with CBC (fallback).")
                    except pulp.PulpSolverError as e_cbc:
                        print(f"Error solving with CBC (fallback): {e_cbc}")
                else:
                    print("No suitable solver could be used.")
        else:
            print("No solver was selected or available.")

    # ---------- step 4: extract ----------
    def extract(self)->Dict:
        chosen = [self.path_list[k] for k,v in enumerate(self.tp) if v.value()>0.5]
        active = [i for i,v in enumerate(self.xi) if v.value()>0.5]
        return {
            "status": pulp.LpStatus[self.prob.status],
            "objective": pulp.value(self.prob.objective),
            "total_path_length": sum(p[-1] for p in chosen),
            "num_active_stations": len(active),
            "active_station_indices": active,
            "chosen_paths": chosen,
        }
