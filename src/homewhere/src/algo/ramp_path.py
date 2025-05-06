# ramp_path.py
from __future__ import annotations

import math, os, heapq, rospy
from typing import Dict, List, Tuple
import yaml

__all__ = ["path_find"]

# --------------------------------------------------------------------------- #
# YAML helpers (with simple caching)
# --------------------------------------------------------------------------- #

_CACHE: Dict[str, dict] = {}

def _load_yaml(path: str) -> dict:
    abs_path = os.path.abspath(path)
    if abs_path not in _CACHE:
        with open(abs_path, "r") as fh:
            _CACHE[abs_path] = yaml.safe_load(fh) or {}
    return _CACHE[abs_path]

def _centres(map_table_yaml: str) -> Dict[str, Tuple[float, float]]:
    tbl = _load_yaml(map_table_yaml).get("map_table", {})
    return {str(k): tuple(v["center"]) for k, v in tbl.items() if "center" in v}

def _graph(ramps_yaml: str) -> List[Tuple[str, str, str]]:
    """Return list of (id, from, to) ramps (edges are one‑way)."""
    return [(r["id"], str(r["from"]), str(r["to"]))
            for r in _load_yaml(ramps_yaml).get("ramps", [])]

# --------------------------------------------------------------------------- #
# Cost function
# --------------------------------------------------------------------------- #

def _edge_cost(frm: str, to: str,
               centres: Dict[str, Tuple[float, float]]) -> float:
    """Euclidean distance between room centres; fallback cost=1."""
    try:
        (x1, y1), (x2, y2) = centres[frm], centres[to]
        return math.hypot(x2 - x1, y2 - y1)
    except KeyError:
        return 1.0

# --------------------------------------------------------------------------- #
# Search algorithms
# --------------------------------------------------------------------------- #

def _dijkstra(start: str, goal: str, edges, centres) -> List[str]:
    q, seen = [(0.0, start, None)], {}        # (cost, node, parent)
    parent: Dict[str, str | None] = {start: None}

    while q:
        cost, node, par = heapq.heappop(q)
        if node in seen:                      # already expanded with lower cost
            continue
        seen[node] = cost
        parent[node] = par
        if node == goal:                      # rebuild path
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            return list(reversed(path))

        for _, frm, to in edges:
            if frm != node:
                continue
            new_cost = cost + _edge_cost(frm, to, centres)
            heapq.heappush(q, (new_cost, to, frm))
    return []

def _astar(start: str, goal: str, edges, centres) -> List[str]:
    def h(n):                                # admissible & consistent
        return _edge_cost(n, goal, centres)

    openq = [(h(start), 0.0, start, None)]   # (f, g, node, parent)
    g_best: Dict[str, float] = {start: 0.0}
    parent: Dict[str, str | None] = {start: None}

    while openq:
        _, g, node, par = heapq.heappop(openq)
        if node == goal:                     # reconstruct
            parent[node] = par
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            return list(reversed(path))

        if parent.get(node) is not None:     # already processed
            continue
        parent[node] = par

        for _, frm, to in edges:
            if frm != node:
                continue
            tentative_g = g + _edge_cost(frm, to, centres)
            if tentative_g < g_best.get(to, float("inf")):
                g_best[to] = tentative_g
                f = tentative_g + h(to)
                heapq.heappush(openq, (f, tentative_g, to, node))
    return []

# --------------------------------------------------------------------------- #
# Public API
# --------------------------------------------------------------------------- #

def path_find(
    start_room: int | str,
    end_room: int | str,
    ramps_yaml: str = "/home/gems/Documents/senior/Senior_Project_v2/src/homewhere/config/ramps.yaml",
    map_table_yaml: str = "/home/gems/Documents/senior/Senior_Project_v2/src/homewhere/config/map_table.yaml",
    algo: str = "astar",
) -> List[str]:
    """
    Compute the lowest cost route of room IDs.

    Parameters
    ----------
    start_room, end_room : str | int
        Room identifiers
    ramps_yaml : str
        Location of ramps definition.
    map_table_yaml : str
        Location of map table (for room centres).
    algo : {'astar', 'dijkstra'}, default 'astar'
        Search strategy.

    Returns
    -------
    list[str]  (empty when no path exists)
    """
    start, goal = str(start_room), str(end_room)
    if start == goal:
        return [start]

    centres = _centres(map_table_yaml)
    edges = _graph(ramps_yaml)

    if algo == "dijkstra":
        return _dijkstra(start, goal, edges, centres)
    elif algo == "astar":
        return _astar(start, goal, edges, centres)
    else:
        raise ValueError("algo must be 'astar' or 'dijkstra'")
    
if __name__ == "__main__":
    print(path_find(2,1,
              ramps_yaml="/home/gems/Documents/senior/Senior_Project_v2/src/homewhere/config/ramps.yaml",
              map_table_yaml="/home/gems/Documents/senior/Senior_Project_v2/src/homewhere/config/map_table.yaml"))
