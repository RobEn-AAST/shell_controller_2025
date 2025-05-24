import carla
import numpy as np
from car_dreamer.toolkit.planner.agents.navigation.global_route_planner import GlobalRoutePlanner
from car_dreamer.toolkit.planner.agents.navigation.basic_agent import BasicAgent
from navigator.wp_utils import xyz_to_wp
from navigator.visualize import CarlaVisualizer
from navigator.tsp_solver import optimize_route_order
from navigator.route_generator import generate_full_route
from navigator.test import spawn_traffic
# import torch
# import os  
from car_dreamer import create_task
from dreamerv3 import embodied
from dreamerv3.embodied.core import Driver  


client = carla.Client("localhost", 2000)
client.set_timeout(20)
world = client.get_world()
carla_map = world.get_map()
ego_vehicle = next(v for v in world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")

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
sampling_resolution = 1.0

target_locations = xyz_to_wp(target_points, carla_map)

grp = GlobalRoutePlanner(carla_map, sampling_resolution)

optimized_targets = optimize_route_order(
    ego_vehicle.get_location(),
    target_locations,
    grp,
)

full_route = generate_full_route(ego_vehicle.get_location(), optimized_targets, grp)


# VISUALIZER
visualizer = CarlaVisualizer(
    agent_route=full_route,
    targets=optimized_targets,  # List of carla.Location
    carla_map=carla_map,
    map_size=(1200, 1200)  # Adjust based on your map
)
visualizer.start()

# ========= SPAWN TRAFFIC ==============
spawn_traffic(client, num_vehicles=100, num_pedestrians=100)


# ======= MOVE VEHICLE ========
task_name = "carla_navigation"  
task, config = create_task(task_name)  

# Load the pretrained model  
checkpoint_path = "./navigator/navigation.ckpt"
config = embodied.Config()  
config = config.update({"logdir": "./logdir/eval"})  
agent = embodied.make_agent(config)  
agent.load(checkpoint_path)  

driver = Driver(agent)  
env = CarlaEnv(  
    client=client,  
    world=world,  
    ego_vehicle=ego_vehicle,  
    route=full_route,  
    config=config  
)  

while True:  
    obs = env.get_observation()  
    action = driver.step(obs)  
      
    control = carla.VehicleControl(  
        throttle=float(action['throttle']),  
        steer=float(action['steer']),  
        brake=float(action['brake'])  
    )  

    ego_vehicle.apply_control(control)  
    curr_location = ego_vehicle.get_location()  
    visualizer.update_position(curr_location)


# agent = BasicAgent(ego_vehicle)
# agent.ignore_traffic_lights(True) 
# agent.set_target_speed(13) 
# agent.set_global_plan(full_route)

# while True:
#     control = agent.run_step()  # Auto-generates throttle/brake/steering
#     ego_vehicle.apply_control(control)

#     print(control)
#     curr_location = ego_vehicle.get_location()
#     visualizer.update_position(curr_location)


# four_lane.ckpt
# lane_merge.ckpt
# left_turn_hard.ckpt
# left_turn_medium.ckpt
# left_turn_simple.ckpt
# navigation.ckpt
# overtake.ckpt
# right_turn_fov.ckpt
# right_turn_hard.ckpt
# right_turn_medium.ckpt
# right_turn_raw.ckpt
# right_turn_sfov.ckpt
# right_turn_simple.ckpt
# roundabout.ckpt
# stop_sign.ckpt
# traffic_light.ckpt