import heapq
import math
import random
import time
from typing import Dict, List, Tuple, Optional

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

# --------------- Graph Model -----------------

class Edge:
    def __init__(self, to_node: int, length: float, speed_limit: float):
        self.to_node = to_node
        self.length = length  # meters
        self.speed_limit = speed_limit  # m/s
        self.base_travel_time = self.length / self.speed_limit  # seconds
        self.congestion_factor = 1.0  # multiplier >= 1
        self.closed = False

    @property
    def travel_time(self) -> float:
        if self.closed:
            # Represent closed edge as very high cost
            return float('inf')
        return self.base_travel_time * self.congestion_factor

    def __repr__(self):
        return f"Edge(to={self.to_node}, len={self.length}, t={self.travel_time:.2f}, closed={self.closed})"


class Graph:
    def __init__(self):
        # adjacency list: node -> list[Edge]
        self.adj: Dict[int, List[Edge]] = {}
        # coordinates for heuristic: node -> (x, y)
        self.coords: Dict[int, Tuple[float, float]] = {}

    def add_node(self, node_id: int, x: float, y: float):
        if node_id not in self.adj:
            self.adj[node_id] = []
        self.coords[node_id] = (x, y)

    def add_edge(self, u: int, v: int, length: float, speed_limit: float, bidirectional: bool = True):
        if u not in self.adj:
            self.adj[u] = []
        if v not in self.adj:
            self.adj[v] = []
        self.adj[u].append(Edge(v, length, speed_limit))
        if bidirectional:
            self.adj[v].append(Edge(u, length, speed_limit))

    def neighbors(self, u: int) -> List[Edge]:
        return self.adj.get(u, [])

    def set_congestion_factor(self, u: int, v: int, factor: float, bidirectional: bool = True):
        """Update congestion factor of edge(s) between u and v."""
        for e in self.adj.get(u, []):
            if e.to_node == v:
                e.congestion_factor = factor
        if bidirectional:
            for e in self.adj.get(v, []):
                if e.to_node == u:
                    e.congestion_factor = factor

    def close_edge(self, u: int, v: int, bidirectional: bool = True):
        for e in self.adj.get(u, []):
            if e.to_node == v:
                e.closed = True
        if bidirectional:
            for e in self.adj.get(v, []):
                if e.to_node == u:
                    e.closed = True

    def open_edge(self, u: int, v: int, bidirectional: bool = True):
        for e in self.adj.get(u, []):
            if e.to_node == v:
                e.closed = False
        if bidirectional:
            for e in self.adj.get(v, []):
                if e.to_node == u:
                    e.closed = False

    def euclidean_distance(self, u: int, v: int) -> float:
        x1, y1 = self.coords[u]
        x2, y2 = self.coords[v]
        return math.hypot(x2 - x1, y2 - y1)


# --------------- Algorithms: Dijkstra & A* -----------------

def reconstruct_path(prev: Dict[int, Optional[int]], start: int, goal: int) -> List[int]:
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        if cur == start:
            break
        cur = prev.get(cur)
    path.reverse()
    if path and path[0] == start:
        return path
    return []


def dijkstra(graph: Graph, start: int, goal: int):
    dist = {node: float('inf') for node in graph.adj}
    prev: Dict[int, Optional[int]] = {node: None for node in graph.adj}
    dist[start] = 0.0

    pq = [(0.0, start)]
    visited = set()
    expanded_nodes = 0

    while pq:
        cur_dist, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)
        expanded_nodes += 1

        if u == goal:
            break

        for edge in graph.neighbors(u):
            if edge.closed:
                continue
            v = edge.to_node
            alt = cur_dist + edge.travel_time
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(pq, (alt, v))

    path = reconstruct_path(prev, start, goal)
    return path, dist[goal], expanded_nodes


def a_star(graph: Graph, start: int, goal: int, vmax: float):
    def heuristic(n: int) -> float:
        # admissible: straight-line distance / vmax
        d = graph.euclidean_distance(n, goal)
        return d / vmax

    g_score = {node: float('inf') for node in graph.adj}
    f_score = {node: float('inf') for node in graph.adj}
    prev: Dict[int, Optional[int]] = {node: None for node in graph.adj}

    g_score[start] = 0.0
    f_score[start] = heuristic(start)

    open_set = [(f_score[start], start)]
    in_open = {start}
    expanded_nodes = 0

    while open_set:
        _, u = heapq.heappop(open_set)
        if u not in in_open:
            continue
        in_open.remove(u)
        expanded_nodes += 1

        if u == goal:
            break

        for edge in graph.neighbors(u):
            if edge.closed:
                continue
            v = edge.to_node
            tentative_g = g_score[u] + edge.travel_time
            if tentative_g < g_score[v]:
                prev[v] = u
                g_score[v] = tentative_g
                f_score[v] = tentative_g + heuristic(v)
                heapq.heappush(open_set, (f_score[v], v))
                in_open.add(v)

    path = reconstruct_path(prev, start, goal)
    return path, g_score[goal], expanded_nodes


# --------------- Traffic Simulation -----------------

class TrafficSimulator:
    def __init__(self, graph: Graph, min_factor=1.0, max_factor=3.0, closure_prob=0.05):
        self.graph = graph
        self.min_factor = min_factor
        self.max_factor = max_factor
        self.closure_prob = closure_prob

    def random_update(self):
        """Randomly update congestion and closures on some edges."""
        for u, edges in self.graph.adj.items():
            for e in edges:
                # With small probability, close/open edge
                if random.random() < self.closure_prob:
                    e.closed = not e.closed
                # Random congestion factor between min and max
                e.congestion_factor = random.uniform(self.min_factor, self.max_factor)


# --------------- Router with Re-routing -----------------

class Router:
    def __init__(self, graph: Graph, vmax: float):
        self.graph = graph
        self.vmax = vmax

    def compute_route(self, start: int, goal: int, algorithm: str = "astar"):
        if algorithm == "dijkstra":
            path, cost, expanded = dijkstra(self.graph, start, goal)
        else:
            path, cost, expanded = a_star(self.graph, start, goal, self.vmax)
        return {
            "algorithm": algorithm,
            "path": path,
            "cost": cost,
            "expanded": expanded
        }

    def simulate_travel_with_rerouting(
        self,
        start: int,
        goal: int,
        traffic_sim: TrafficSimulator,
        algorithm: str = "astar",
        recompute_interval: float = 10.0
    ):
        """
        Very simplified travel simulation:
        - Vehicle moves one edge per 'step'.
        - After each step, traffic is updated.
        - Every recompute_interval seconds of simulated time, route is recomputed.
        """
        result = self.compute_route(start, goal, algorithm=algorithm)
        path = result["path"]
        if not path:
            print("No initial path found")
            return None

        current_index = 0
        current_time = 0.0
        total_reroutes = 0

        print(f"Initial {algorithm} route: {path}, ETA={result['cost']:.1f}s, expanded={result['expanded']}")

        while current_index < len(path) - 1:
            u = path[current_index]
            v = path[current_index + 1]

            # Find edge travel time *at this moment*
            edge_time = None
            for e in self.graph.neighbors(u):
                if e.to_node == v:
                    edge_time = e.travel_time
                    break
            if edge_time is None or edge_time == float('inf'):
                # Edge blocked, need immediate reroute
                print(f"Edge ({u}->{v}) blocked, rerouting from node {u}...")
                result = self.compute_route(u, goal, algorithm=algorithm)
                new_path = result["path"]
                if not new_path:
                    print("No alternative route found, giving up.")
                    return None
                path = new_path
                current_index = 0
                total_reroutes += 1
                continue

            # "Travel" along the edge
            current_time += edge_time
            current_index += 1

            # Update traffic conditions after each edge
            traffic_sim.random_update()

            # Periodic reroute based on time
            if current_time % recompute_interval < edge_time:
                cur_node = path[current_index]
                result = self.compute_route(cur_node, goal, algorithm=algorithm)
                new_path = result["path"]
                if new_path and new_path[0] == cur_node:
                    path = new_path
                    current_index = 0
                    total_reroutes += 1
                    print(f"Recomputed route from node {cur_node}: {path}, new ETA from here={result['cost']:.1f}s")

        print(f"Arrived at destination {goal} in simulated time {current_time:.1f}s with {total_reroutes} reroutes.")
        return {
            "total_time": current_time,
            "reroutes": total_reroutes
        }


# --------------- Simple Visualization -----------------

def plot_graph(graph: Graph, path: Optional[List[int]] = None, title: str = "Road Network"):
    if plt is None:
        print("matplotlib not installed; skipping visualization.")
        return

    plt.figure(figsize=(6, 6))
    # Draw edges
    for u, edges in graph.adj.items():
        x1, y1 = graph.coords[u]
        for e in edges:
            v = e.to_node
            x2, y2 = graph.coords[v]
            color = 'gray'
            if e.closed:
                color = 'red'
            elif e.congestion_factor > 1.5:
                color = 'orange'
            plt.plot([x1, x2], [y1, y2], color=color, linewidth=1)

    # Draw nodes
    xs = [coord[0] for coord in graph.coords.values()]
    ys = [coord[1] for coord in graph.coords.values()]
    plt.scatter(xs, ys, c='black', s=10)

    # Highlight path
    if path and len(path) > 1:
        px = []
        py = []
        for node in path:
            x, y = graph.coords[node]
            px.append(x)
            py.append(y)
        plt.plot(px, py, color='blue', linewidth=3, label='Route')
        plt.scatter([px[0]], [py[0]], c='green', s=40, label='Start')
        plt.scatter([px[-1]], [py[-1]], c='red', s=40, label='Goal')
        plt.legend()

    plt.title(title)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.show()


# --------------- Demo graph & main -----------------

def build_demo_city_graph() -> Graph:
    """
    Build a small grid-like urban graph for testing.
    Nodes arranged in a 4x4 grid (16 nodes).
    """
    g = Graph()
    grid_size = 4
    spacing = 100.0  # meters
    speed_limit = 13.9  # ~50 km/h in m/s

    # Create nodes with coordinates
    node_id = 0
    for i in range(grid_size):
        for j in range(grid_size):
            g.add_node(node_id, x=j * spacing, y=i * spacing)
            node_id += 1

    def node(i, j):
        return i * grid_size + j

    # Add edges (4-neighbor grid)
    for i in range(grid_size):
        for j in range(grid_size):
            u = node(i, j)
            if j < grid_size - 1:
                v = node(i, j + 1)
                g.add_edge(u, v, length=spacing, speed_limit=speed_limit, bidirectional=True)
            if i < grid_size - 1:
                v = node(i + 1, j)
                g.add_edge(u, v, length=spacing, speed_limit=speed_limit, bidirectional=True)

    return g


def benchmark_algorithms(graph: Graph, pairs: List[Tuple[int, int]], vmax: float):
    router = Router(graph, vmax=vmax)
    print("=== Benchmark: A* vs Dijkstra on static graph ===")
    for (s, t) in pairs:
        start = time.time()
        res_a = router.compute_route(s, t, algorithm="astar")
        t_a = (time.time() - start) * 1000.0

        start = time.time()
        res_d = router.compute_route(s, t, algorithm="dijkstra")
        t_d = (time.time() - start) * 1000.0

        print(f"{s} -> {t}")
        print(f"  A*:       cost={res_a['cost']:.1f}s, expanded={res_a['expanded']}, time={t_a:.2f} ms")
        print(f"  Dijkstra: cost={res_d['cost']:.1f}s, expanded={res_d['expanded']}, time={t_d:.2f} ms")


def main():
    # Build demo graph
    graph = build_demo_city_graph()
    vmax = 13.9  # same as speed limit here

    # Compare A* vs Dijkstra
    benchmark_pairs = [(0, 15), (1, 14), (3, 12)]
    benchmark_algorithms(graph, benchmark_pairs, vmax)

    # Setup traffic simulator and router
    traffic_sim = TrafficSimulator(graph, min_factor=1.0, max_factor=3.0, closure_prob=0.02)
    router = Router(graph, vmax=vmax)

    # Initial static route (for visualization)
    res_static = router.compute_route(0, 15, algorithm="astar")
    print("\n=== Static A* route (no dynamic updates) ===")
    print(res_static)

    # Visualize static route if matplotlib available
    plot_graph(graph, res_static["path"], title="Initial Static A* Route")

    # Now simulate dynamic travel with re-routing
    print("\n=== Dynamic simulation with re-routing (A*) ===")
    sim_result = router.simulate_travel_with_rerouting(
        start=0,
        goal=15,
        traffic_sim=traffic_sim,
        algorithm="astar",
        recompute_interval=20.0
    )
    print("Simulation result:", sim_result)

    # Final visualization (last known route; traffic will have changed)
    final_route = router.compute_route(0, 15, algorithm="astar")
    plot_graph(graph, final_route["path"], title="Graph After Dynamic Simulation")


if __name__ == "__main__":
    main()
