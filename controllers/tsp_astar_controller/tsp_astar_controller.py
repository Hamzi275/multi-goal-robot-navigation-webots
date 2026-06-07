from controller import Supervisor
from heapq import heappop, heappush
from itertools import count
from math import atan2, hypot


TIME_STEP = 32
GRID_RESOLUTION = 0.20
ROBOT_Y = 0.11
ROBOT_SPEED = 0.60
GOAL_TOLERANCE = 0.12

X_LIMITS = (-4.35, 4.35)
Z_LIMITS = (-3.35, 3.35)
START = (-4.2, -3.2)
GOALS = [
    (-3.6, 2.8),
    (0.2, 2.9),
    (3.5, 2.2),
    (4.0, -3.0),
    (-3.7, -2.7),
]

# Rectangles are (center_x, center_z, size_x, size_z). They match the world boxes
# with extra clearance so the robot does not clip through obstacle corners.
OBSTACLES = [
    (-1.4, -1.2, 1.7, 0.7),
    (1.6, 0.9, 1.2, 1.8),
    (3.0, -1.8, 0.8, 1.4),
]
CLEARANCE = 0.34


def world_to_grid(point):
    x, z = point
    gx = round((x - X_LIMITS[0]) / GRID_RESOLUTION)
    gz = round((z - Z_LIMITS[0]) / GRID_RESOLUTION)
    return gx, gz


def grid_to_world(cell):
    gx, gz = cell
    x = X_LIMITS[0] + gx * GRID_RESOLUTION
    z = Z_LIMITS[0] + gz * GRID_RESOLUTION
    return x, z


def in_bounds(cell):
    x, z = grid_to_world(cell)
    return X_LIMITS[0] <= x <= X_LIMITS[1] and Z_LIMITS[0] <= z <= Z_LIMITS[1]


def blocked(cell):
    x, z = grid_to_world(cell)
    for cx, cz, sx, sz in OBSTACLES:
        half_x = sx / 2 + CLEARANCE
        half_z = sz / 2 + CLEARANCE
        if cx - half_x <= x <= cx + half_x and cz - half_z <= z <= cz + half_z:
            return True
    return False


def neighbors(cell):
    gx, gz = cell
    candidates = [
        (gx + 1, gz),
        (gx - 1, gz),
        (gx, gz + 1),
        (gx, gz - 1),
    ]
    for nxt in candidates:
        if in_bounds(nxt) and not blocked(nxt):
            yield nxt


def heuristic(a, b):
    ax, az = grid_to_world(a)
    bx, bz = grid_to_world(b)
    return hypot(ax - bx, az - bz)


def astar(start, goal):
    start_cell = world_to_grid(start)
    goal_cell = world_to_grid(goal)
    frontier = []
    tie = count()
    heappush(frontier, (0.0, next(tie), start_cell))
    came_from = {start_cell: None}
    cost_so_far = {start_cell: 0.0}

    while frontier:
        _, _, current = heappop(frontier)
        if current == goal_cell:
            break

        for nxt in neighbors(current):
            new_cost = cost_so_far[current] + heuristic(current, nxt)
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + heuristic(nxt, goal_cell)
                heappush(frontier, (priority, next(tie), nxt))
                came_from[nxt] = current

    if goal_cell not in came_from:
        raise RuntimeError(f"No A* path found from {start} to {goal}")

    path = []
    current = goal_cell
    while current is not None:
        path.append(grid_to_world(current))
        current = came_from[current]
    path.reverse()
    path[-1] = clamp_point(goal)
    return path


def path_length(path):
    return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def nearest_neighbor_order(start, goals):
    remaining = list(range(len(goals)))
    order = []
    current = start
    while remaining:
        best = min(remaining, key=lambda i: distance(current, goals[i]))
        order.append(best)
        current = goals[best]
        remaining.remove(best)
    return order


def distance(a, b):
    return hypot(b[0] - a[0], b[1] - a[1])


def clamp_point(point):
    x, z = point
    x = min(max(x, X_LIMITS[0]), X_LIMITS[1])
    z = min(max(z, Z_LIMITS[0]), Z_LIMITS[1])
    return x, z


def yaw_rotation_towards(current, target):
    dx = target[0] - current[0]
    dz = target[1] - current[1]
    yaw = atan2(dx, dz)
    return [0, 1, 0, yaw]


def move_towards(current, target, max_step):
    dx = target[0] - current[0]
    dz = target[1] - current[1]
    distance = hypot(dx, dz)
    if distance <= max_step:
        return clamp_point(target), distance, True
    ratio = max_step / distance
    return clamp_point((current[0] + dx * ratio, current[1] + dz * ratio)), max_step, False


def main():
    robot = Supervisor()
    robot_node = robot.getSelf()
    translation_field = robot_node.getField("translation")
    rotation_field = robot_node.getField("rotation")

    order = nearest_neighbor_order(START, GOALS)
    full_path = [START]
    route_distance = 0.0
    current = START
    for goal_index in order:
        segment = astar(current, GOALS[goal_index])
        route_distance += path_length(segment)
        full_path.extend(segment[1:])
        current = GOALS[goal_index]

    print("Route order:", " -> ".join(f"G{i + 1}" for i in order), flush=True)
    print(f"Planned travel distance: {route_distance:.2f} m", flush=True)

    waypoint_index = 1
    goal_targets = [GOALS[i] for i in order]
    visited_goals = 0
    travelled = 0.0
    position = START
    start_time = robot.getTime()
    translation_field.setSFVec3f([position[0], ROBOT_Y, position[1]])

    while robot.step(TIME_STEP) != -1:
        if waypoint_index >= len(full_path):
            elapsed = robot.getTime() - start_time
            print("All goals visited.", flush=True)
            print(f"Total travel distance: {travelled:.2f} m", flush=True)
            print(f"Total completion time: {elapsed:.2f} s", flush=True)
            print(f"Number of visited goals: {visited_goals}", flush=True)
            break

        target = full_path[waypoint_index]
        step_distance = ROBOT_SPEED * (TIME_STEP / 1000.0)
        position, moved, reached = move_towards(position, target, step_distance)
        travelled += moved
        translation_field.setSFVec3f([position[0], ROBOT_Y, position[1]])
        rotation_field.setSFRotation(yaw_rotation_towards(position, target))

        if reached or hypot(position[0] - target[0], position[1] - target[1]) <= GOAL_TOLERANCE:
            if visited_goals < len(goal_targets):
                next_goal = goal_targets[visited_goals]
                if hypot(position[0] - next_goal[0], position[1] - next_goal[1]) <= GOAL_TOLERANCE:
                    visited_goals += 1
                    print(f"Visited G{order[visited_goals - 1] + 1} at t={robot.getTime() - start_time:.2f}s", flush=True)
            waypoint_index += 1

    robot.step(TIME_STEP)


if __name__ == "__main__":
    main()
