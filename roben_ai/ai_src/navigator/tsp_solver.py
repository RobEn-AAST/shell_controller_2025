from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from car_dreamer.toolkit.planner.agents.navigation.global_route_planner import GlobalRoutePlanner
import numpy as np

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
