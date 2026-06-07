# Multi-Goal Robot Navigation in Webots R2025a

This project demonstrates a single robot navigating an indoor-like environment with static obstacles and multiple target locations. The robot must visit all targets in an efficient order while planning safe local paths between them.

The simulation was built in **Webots R2025a** using a Python controller.

## Demo Video

The simulation video is included as:

[Watch the demo video](./1.mp4)

> Note: place `1.mp4` in the root of this repository before uploading to GitHub.

## Problem Statement

The goal of this project was to design a robot that can:

- Start from an initial position inside an indoor-style arena.
- Visit 4-6 target locations.
- Avoid static obstacles.
- Decide a reasonable order for visiting the targets.
- Plan a collision-free path between each selected target.
- Report useful performance metrics after completing the route.

This combines two important robotics/navigation ideas:

- **Route ordering:** deciding which goal should be visited next.
- **Local path planning:** finding a safe path from the current position to the selected goal.

## My Main Challenge

The main challenge was not only writing the algorithm, but also making the Webots world load and run correctly.

Earlier versions of the project had issues where the `.wbt` file could fail to load or Webots could close unexpectedly. To avoid that, this version uses a clean standard Webots project structure and simple built-in Webots nodes instead of fragile custom assets.

Another issue was unstable movement near arena boundaries and obstacles. I solved this by simplifying the movement model and keeping the planned path inside safe map limits.

## Solution Approach

The final solution uses:

- **Nearest Neighbor heuristic** for choosing the next target.
- **A\* Search** for obstacle-aware path planning between waypoints.
- **Grid-based map representation** for planning.
- **Supervisor-based robot movement** for simple and stable simulation behavior.
- **Metric logging** in the Webots console.

## Algorithm Overview

### 1. Nearest Neighbor Route Ordering

The robot starts from its initial position and repeatedly chooses the closest unvisited target.

This is a simple TSP-style heuristic. It does not guarantee the mathematically optimal route, but it is easy to implement, fast, and suitable for small projects with 4-6 targets.

### 2. A\* Local Path Planning

After selecting the next target, the robot uses A\* search to find a safe path through the grid.

A\* was chosen because it uses a heuristic distance estimate toward the goal, making it more efficient than plain Dijkstra for this type of navigation task.

### 3. Metrics

The controller prints:

- Route order
- Planned travel distance
- Goal visit timestamps
- Total travel distance
- Total completion time
- Number of visited goals

Example output:

```text
Route order: G5 -> G1 -> G2 -> G3 -> G4
Planned travel distance: 20.08 m
Visited G5 at t=...
Visited G1 at t=...
Visited G2 at t=...
Visited G3 at t=...
Visited G4 at t=...
All goals visited.
Total travel distance: ...
Total completion time: ...
Number of visited goals: 5
```

## Project Structure

```text
.
├── README.md
├── 1.mp4
├── worlds
│   └── tsp_astar_indoor.wbt
└── controllers
    └── tsp_astar_controller
        ├── runtime.ini
        └── tsp_astar_controller.py
```

## How to Run

1. Install **Webots R2025a**.
2. Open Webots.
3. Open the world file:

```text
worlds/tsp_astar_indoor.wbt
```

4. Run the simulation.
5. Check the Webots console for route and metric output.

## Key Files

### `worlds/tsp_astar_indoor.wbt`

This is the Webots world file. It contains:

- Indoor-like floor arena
- Boundary walls
- Static obstacles
- Five visual target points
- One robot controlled by the Python controller

### `controllers/tsp_astar_controller/tsp_astar_controller.py`

This file contains the main logic:

- Target locations
- Obstacle definitions
- Grid conversion
- A\* search
- Nearest Neighbor ordering
- Robot movement
- Metric calculation and printing

## Technologies Used

- Webots R2025a
- Python
- A\* Search
- Nearest Neighbor heuristic
- TSP-style route ordering
- Grid-based path planning
- Robotics simulation

## Why This Project Matters

This project shows how route optimization and path planning can be combined in a robotics simulation. Instead of only moving a robot from one point to another, the robot must make a sequence of decisions and complete multiple goals efficiently.

It is a compact but practical demonstration of autonomous navigation concepts used in robotics, warehouse robots, indoor service robots, and delivery systems.

## Author

**Ameer Hamza**

AI/ML Engineer | Data Engineer | Data Analyst | AI Agents | SaaS & Data Pipelines | RAG
