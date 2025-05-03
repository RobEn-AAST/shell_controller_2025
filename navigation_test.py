import carla
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import threading
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from car_dreamer.toolkit.planner.agents.navigation.global_route_planner import GlobalRoutePlanner


def optimize_route_order(start_location, target_locations, grp: GlobalRoutePlanner):
    """
    Solve TSP for optimal target order using actual road distances
    returns list of locations, can be accessed with returned[0].x, .y, .z
    """

    # 1. Create distance matrix using CARLA's actual path distances
    all_points = [start_location] + target_locations
    n = len(all_points)
    distance_matrix = np.zeros((n, n))

    print("Calculating pairwise distances...")
    for i in range(n):
        for j in range(n):
            if i == j:
                distance_matrix[i][j] = 0
                continue
            # Get actual path distance through roads
            route = grp.trace_route(all_points[i], all_points[j])
            distance_matrix[i][j] = sum(wp[0].transform.location.distance(route[k + 1][0].transform.location) for k, wp in enumerate(route[:-1])) if route else float("inf")

    # 2. Solve TSP
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)  # 1 vehicle
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)

    # 3. Extract optimal order
    index = routing.Start(0)
    ordered_indices = []
    while not routing.IsEnd(index):
        ordered_indices.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    ordered_indices.append(0)  # Return to start (remove if not needed)

    # 4. Reorder targets based on optimization
    optimal_order = [all_points[i] for i in ordered_indices[1:-1]]  # Exclude start/end dupes

    return optimal_order


def generate_full_route(start_loc, targets, grp: GlobalRoutePlanner):
    """
    returnrs list of [waypoint, roundoff]
    can access locaation throguh full_route[0][0].transform.location.x, .y, .z
    """
    full_route = []
    current_start = start_loc

    for target in targets:
        # Get route segment
        segment = grp.trace_route(current_start, target)

        if not segment:
            print(f"⚠️ Couldn't find path to {target}! Skipping...")
            continue

        # Append to full route (skip first waypoint if not first segment)
        full_route += segment if not full_route else segment[1:]
        current_start = target  # Update start for next segment

    return full_route


def generate_simple_path(start_loc, targets, grp: GlobalRoutePlanner):
    """
    returns list of locations, access with full_path[0].x, .y, .z
    """
    full_path = []
    current_start = start_loc

    for target in targets:
        segment = grp.trace_route(current_start, target)
        if not segment:
            continue

        # Extract just the locations
        locations = [wp[0].transform.location for wp in (segment if not full_path else segment[1:])]
        full_path += locations
        current_start = target

    return full_path


client = carla.Client("localhost", 2000)
client.set_timeout(20)
world = client.get_world()
carla_map = world.get_map()

ego_vehicle = next(v for v in world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")
ego_location = ego_vehicle.get_location()

spawn_points = carla_map.get_spawn_points()
target_points = [
    [280.363739, 133.306351, 0.001746],
    [334.949799, 161.106171, 0.001736],
    [339.100037, 258.568939, 0.001679],
    [396.295319, 183.195740, 0.001678],
    [267.657074, 1.983160, 0.001678],
    [153.868896, 26.115866, 0.001678],
    [290.515564, 56.175072, 0.001677],
    [92.325722, 86.063644, 0.001677],
    [88.384346, 287.468567, 0.001728],
    [177.594101, 326.386902, 0.001677],
    [-1.646942, 197.501282, 0.001555],
    [59.701321, 1.970804, 0.001467],
    [122.100121, 55.142044, 0.001596],
    [161.030975, 129.313187, 0.001679],
    [184.758713, 199.424271, 0.001680],
]

target_locations = []
for x, y, z in target_points:
    loc = carla.Location(x=x, y=y, z=z)
    wp = carla_map.get_waypoint(loc, project_to_road=True)
    target_locations.append(wp.transform.location)

sampling_resolution = 1.0
grp = GlobalRoutePlanner(carla_map, sampling_resolution)

optimized_targets = optimize_route_order(
    ego_vehicle.get_location(),
    target_locations,
    grp,
)

full_route = generate_simple_path(ego_vehicle.get_location(), target_locations, grp)

# ========= VISUALIZE ===============
print(f"map name: {carla_map.name}")
topology = carla_map.get_topology()

plt.figure(figsize=(12, 12))

# Draw map topology
for edge in carla_map.get_topology():
    x = [edge[0].transform.location.x, edge[1].transform.location.x]
    y = [edge[0].transform.location.y, edge[1].transform.location.y]
    plt.plot(x, y, "k-", alpha=0.2)


# Draw car
plt.plot(ego_location.x, ego_location.y, "ro", markersize=10, label="Ego Vehicle")

# Mark positions
for target_loc in target_locations:
    plt.plot(target_loc.x, target_loc.y, "g*", markersize=8, label="Target")

# Draw full route with color progression
route_x = [wp.x for wp in full_route]
route_y = [wp.y for wp in full_route]
plt.plot(route_x, route_y, color="blue", linewidth=2)


plt.legend()
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("CARLA Map Topology")
plt.show()


# # Make vehicle follow path (pseudo-code)
# for location in simple_path:
#     vehicle.set_velocity(calculate_speed_to(location))
#     vehicle.steer(calculate_steering(location))
#     time.sleep(0.1)
