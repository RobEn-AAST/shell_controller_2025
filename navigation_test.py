import carla
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import threading
from car_dreamer.toolkit.planner.agents.navigation.global_route_planner import GlobalRoutePlanner


def generate_full_route(start_loc, targets):
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

def generate_simple_path(start_loc, targets):
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
target_locations = [
    spawn_points[0].location,
    spawn_points[1].location,
    spawn_points[2].location,
    spawn_points[3].location,
]

sampling_resolution = 1.0
grp = GlobalRoutePlanner(carla_map, sampling_resolution)

full_route = generate_simple_path(ego_vehicle.get_location(), target_locations)

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
plt.plot(route_x, route_y, color='blue', linewidth=2)


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