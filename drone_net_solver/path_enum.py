print("Executing path_enum.py: top")
print(f"path_enum.py: dir() at top: {dir()}")

"""
Yen k-shortest loop-less paths wrapper using NetworkX.
"""

from typing import List, Tuple
import itertools, networkx as nx

def k_shortest_simple_paths(G: nx.DiGraph,
                            source: str,
                            target: str,
                            k: int,
                            length_cap: float | None = None
                           ) -> List[Tuple[List[str], float]]:
    print("Executing path_enum.py: inside k_shortest_simple_paths definition")
    paths = []
    try:
        gen = nx.shortest_simple_paths(G, source, target, weight="weight")  # Yen’s algo
        for path in itertools.islice(gen, k):
            length = sum(G[u][v]["weight"] for u, v in zip(path[:-1], path[1:]))
            if length_cap is None or length <= length_cap:
                paths.append((path, length))
    except nx.NetworkXNoPath:
        pass
    print(f"path_enum.py: dir() after k_shortest_simple_paths definition: {dir()}")
    return paths
