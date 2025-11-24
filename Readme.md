# Dynamic A\*-Based Emergency Vehicle Routing

This project implements a dynamic routing engine for emergency vehicles (ambulances, fire trucks) on an urban road network, using A\* and Dijkstra algorithms on a weighted graph with traffic-sensitive edge weights. The goal is to minimize response time by continuously adapting routes to changing traffic conditions.

## Features

- Road network modeled as a weighted graph using adjacency lists.  
- Nodes represent intersections; edges represent road segments with length, speed limit, and dynamic congestion.  
- A\* algorithm with Euclidean-distance-based heuristic for minimum-time routing.  
- Dijkstra’s algorithm as a baseline for performance comparison.  
- Simulated real-time traffic updates:
  - Congestion factors change edge travel times.  
  - Random road closures and re-openings.  
- Re-routing logic that:
  - Detects blocked or heavily congested edges on the current route.  
  - Recomputes the route from the vehicle’s current node.  
- Simple 2D visualization of:
  - The road network.  
  - Current route (start, goal, and path).  
  - Traffic severity (colors for congestion and closures).  

## Project structure

If you split the earlier code into modules, a typical layout is:

```text
emergency_routing/
├─ main.py          # Entry point: builds graph, benchmarks, runs simulation
├─ graph.py         # Graph, Node, Edge data structures
├─ algorithms.py    # Dijkstra and A* implementations
├─ traffic.py       # TrafficSimulator for dynamic edge updates
├─ router.py        # Router class, routing API and re-routing logic
└─ visualize.py     # Matplotlib-based network and path visualization
```

For quick use, you can also keep everything in a single `main.py` file exactly as provided.

## Requirements

- Python 3.8+  
- Recommended packages:
  - `matplotlib` (for visualization)  

Install dependencies:

```bash
pip install matplotlib
```

The core routing and simulation will still work without `matplotlib`; only the visualization will be skipped.

## How to run

1. Place the provided Python code in `main.py` inside a folder (e.g., `emergency_routing/`).  
2. (Optional) Adjust parameters in `build_demo_city_graph()` and in `main()`:
   - Grid size, spacing, speed limits.  
   - Traffic severity range and closure probability in `TrafficSimulator`.  
   - `recompute_interval` for how often to re-route.  
3. Run the project:

```bash
python main.py
```

## What the demo does

The `main()` function:

1. Builds a synthetic 4×4 grid city graph with intersections as nodes and roads as edges.  
2. Benchmarks A\* vs Dijkstra on several source–destination pairs, printing:
   - Route cost (travel time).  
   - Number of expanded nodes.  
   - Wall-clock runtime in milliseconds.  
3. Computes and prints an initial static A\* route between two nodes (e.g., 0 → 15).  
4. Visualizes this route on a simple 2D map (if `matplotlib` is available).  
5. Runs a dynamic simulation where:
   - The vehicle “travels” along the route edge by edge.  
   - After each edge, traffic conditions are randomly updated.  
   - If an edge becomes blocked, or a periodic timer triggers, the route is recomputed from the current node.  
   - Total simulated travel time and number of re-routes are printed.  
6. Optionally visualizes the graph again after the simulation.

## Configuration

Key parameters you may want to tune:

- **Graph topology**:  
  - In `build_demo_city_graph()`, change `grid_size` and `spacing` to simulate larger or denser urban areas.  
- **Speed limits and vmax**:  
  - Adjust `speed_limit` and global `vmax` in `main()` to reflect realistic road and emergency vehicle speeds.  
- **Traffic behavior** (in `TrafficSimulator`):  
  - `min_factor`, `max_factor`: range of congestion multipliers (1.0 = free flow).  
  - `closure_prob`: probability that any edge toggles between open and closed at each update.  
- **Re-routing policy** (in `Router.simulate_travel_with_rerouting`):  
  - `recompute_interval`: simulated seconds between scheduled re-routes.

## Extending the project

Possible extensions for coursework or research:

- Load a real city network (e.g., from OpenStreetMap) instead of the synthetic grid.  
- Log all routes and edge usages to CSV, then analyze:
  - Most frequently used edges.  
  - Average and worst-case edge travel times.  
  - Identification of persistent choke points.  
- Implement time-dependent edge weights (e.g., rush-hour profiles).  
- Add multiple vehicles and dispatch policies (priority queues for calls, station locations).  
- Integrate with real-time traffic APIs in place of the simulator.  

## How to use in a report

You can describe this project as:

- A dynamic emergency routing engine using A\* and Dijkstra on a traffic-sensitive graph.  
- Demonstrating:
  - Improved search efficiency of A\* compared to Dijkstra.  
  - Impact of dynamic congestion on response times.  
  - Benefits of periodic and event-triggered re-routing in an urban road network.

If you want, a separate sectioned document (SRS, design document, or user manual style) can be drafted around this README.

[1](https://github.com/TanayMayee/intelligent-emergency-rerouting)
[2](https://github.com/StevenMHernandez/VANET-Emergency-Routing-Protocol)
[3](https://odr.chalmers.se/bitstreams/c93f2127-c492-431c-a2ad-e049cfa4e43f/download)
[4](https://sist.sathyabama.ac.in/sist_naac/aqar_2022_2023/documents/1.3.4/b.e-cse-batchno-22.pdf)
[5](https://www.pubnub.com/blog/smart-traffic-management-system-for-emergency-services/)
[6](https://here-location-services-python.readthedocs.io/_/downloads/en/latest/pdf/)
[7](https://git.ifas.rwth-aachen.de/templates/ifas-python-template/-/blob/master/README.md)
[8](https://www.masaischool.com/blog/20-python-project-ideas-for-college-students/)
