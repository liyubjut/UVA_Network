"""
Graph construction utilities (matches Eq. (3) of the paper).

Nodes:  S (hubs) ∪ I (candidate charging stations)
Edges:
  • hub → station, if distance ≤ 2R
  • station ↔ station (both directions), if distance ≤ 2R
Demand points N are NOT part of Γ; they are handled by the coverage matrix c_{i,n}.
"""
print("Executing graph_utils.py: top")
from typing import List, Tuple
import math, itertools, networkx as nx

Point = Tuple[float, float]

def euclid(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

def build_graph(hubs: List[Point],
                stations: List[Point],
                R: float) -> nx.DiGraph:
    G = nx.DiGraph()
    for i, h in enumerate(hubs):
        G.add_node(f"S{i}", pos=h, kind="hub")
    for i, st in enumerate(stations):
        G.add_node(f"I{i}", pos=st, kind="station")

    thr = 2 * R + 1e-9            # feasible hop length

    # hub → station (single direction)
    for hs, h in enumerate(hubs):
        for is_, st in enumerate(stations):
            d = euclid(h, st)
            if d <= thr:
                G.add_edge(f"S{hs}", f"I{is_}", weight=d)

    # station ↔ station (both directions)
    for (i1, st1), (i2, st2) in itertools.permutations(enumerate(stations), 2):
        d = euclid(st1, st2)
        if d <= thr:
            G.add_edge(f"I{i1}", f"I{i2}", weight=d)
    return G
