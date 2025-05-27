from typing import Dict, List, Tuple
from ai_src.carla_others.agents.navigation.behavior_agent import BehaviorAgent
import numpy as np
import carla
from .car_dreamer.car_dreamer.toolkit.carla_manager import WorldManager
from .car_dreamer.car_dreamer.toolkit.observer.handlers.renderer.birdeye_renderer import BirdeyeRenderer
from .car_dreamer.car_dreamer.toolkit.observer.handlers.renderer.constants import BirdeyeEntity


def set_agent(ego_vehicle):
    agent = BehaviorAgent(ego_vehicle, behavior="aggressive")  # cautious, normal, aggressive
    agent.ignore_traffic_lights(True)
    agent._behavior.max_speed = 47

    return agent


def visualize_agent_path(renderer: BirdeyeRenderer, ego_vehicle: carla.Actor, waypoints=None, screen_size=512) -> np.ndarray:
    """
    Create a top-down bird's-eye view image of the CARLA environment.

    Args:
        world_manager: CarDreamer WorldManager instance
        ego_vehicle: The ego vehicle to center the view on
        waypoints: Optional list of waypoints to render as path
        screen_size: Size of the output image (default: 512x512)
        obs_range: Observation range in meters (default: 50m)

    Returns:
        np.ndarray: RGB image of shape (screen_size, screen_size, 3)
    """
    # Calculate pixels per meter based on screen size and observation range
    renderer.set_ego(ego_vehicle)

    # Define what entities to render
    entities = [
        BirdeyeEntity.ROADMAP,
        BirdeyeEntity.EGO_VEHICLE,
        BirdeyeEntity.BACKGROUND_VEHICLES,
    ]

    # Add waypoints if provided
    if waypoints is not None:
        entities.append(BirdeyeEntity.WAYPOINTS)

    # Create environment state for rendering
    env_state = {}
    if waypoints is not None:
        # Convert waypoints to the format expected by the renderer
        ego_waypoints = [(wp.transform.location.x, wp.transform.location.y) for wp in waypoints] if hasattr(waypoints[0], "transform") else waypoints
        env_state["ego_waypoints"] = ego_waypoints

    # Create the display surface
    display = np.zeros((screen_size, screen_size, 3), dtype=np.uint8)

    # Render the scene
    renderer.render(display, entities, env_state)

    return display


def update_all_vehicles_polygons(world_manager: "WorldManager"):
    """
    Loops over all vehicles in the CARLA simulation, checks if their polygon data
    is registered with the WorldManager, and registers it if not.

    Args:
        world_manager: An instance of CarDreamer's WorldManager.
    """
    carla_world = world_manager._world  # Access the raw carla.World object

    # Get all vehicles currently in the CARLA simulation
    all_vehicles_in_world = carla_world.get_actors().filter("vehicle.*")

    # Access the WorldManager's internal dictionaries for polygons and transforms
    # These are typically populated when actors are spawned by WorldManager
    registered_polygons: Dict[int, List[Tuple[float, float]]] = world_manager.actor_polygons
    registered_transforms: Dict[int, carla.Transform] = world_manager.actor_transforms

    # print(f"Checking {len(all_vehicles_in_world)} vehicles in the CARLA world...")

    for vehicle in all_vehicles_in_world:
        vehicle_id = vehicle.id

        # Check if the vehicle's polygon is already registered
        if vehicle_id not in registered_polygons:
            print(f"Vehicle ID {vehicle_id} not registered. Calculating and adding polygon...")

            # Manually compute the polygon for the vehicle
            # This logic is derived from WorldManager._get_actor_polygons
            # (car_dreamer/toolkit/carla_manager/world_manager.py:280-304)
            actor_transform = vehicle.get_transform()
            x = actor_transform.location.x
            y = actor_transform.location.y
            yaw = actor_transform.rotation.yaw * np.pi / 180

            bb = vehicle.bounding_box
            l, w = bb.extent.x, bb.extent.y

            poly_local = np.array([[l, w], [l, -w], [-l, -w], [-l, w]]).T
            R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            poly = np.matmul(R, poly_local).T + np.repeat([[x, y]], 4, axis=0)

            # Register the polygon and transform
            registered_polygons[vehicle_id] = poly.tolist()
            registered_transforms[vehicle_id] = actor_transform
            print(f"Registered polygon for vehicle ID {vehicle_id}.")
        else:
            # Optionally, you can update the transform even if the polygon is registered
            # as vehicle positions change over time.
            registered_transforms[vehicle_id] = vehicle.get_transform()
            # print(f"Vehicle ID {vehicle_id} already registered. Updating transform.")

    # print("Finished checking and registering vehicle polygons.")


def delete_all_vehicles_safe(world_manager: WorldManager):  
    """  
    Loop over all vehicles in the CARLA world and safely delete them.  
    This function also ensures the WorldManager's internal actor tracking  
    and polygon caches are correctly updated.  
  
    Args:  
        world_manager: An instance of CarDreamer's WorldManager.  
    """  
    # Get all vehicles currently in the CARLA simulation, not just those managed by WorldManager  
    vehicles_in_world = world_manager._world.get_actors().filter("*vehicle*")  
    deleted_count = 0  
    total_vehicles_to_delete = len(vehicles_in_world)  
  
    # Prepare a batch command for destruction for efficiency  
    destroy_commands = []  
    for vehicle in vehicles_in_world:  
        destroy_commands.append(carla.command.DestroyActor(vehicle.id))  
  
    # Apply the batch destruction command  
    try:  
        response = world_manager._client.apply_batch_sync(destroy_commands, True)  
        for i, res in enumerate(response):  
            if not res.error:  
                deleted_count += 1  
            else:  
                print(f"Failed to delete vehicle {vehicles_in_world[i].id}: {res.error}")  
    except Exception as e:  
        print(f"Error during batch deletion: {e}")  
  
    # Clear the WorldManager's internal actor_dict  
    # This is the correct way to tell WorldManager that its managed actors are gone.  
    # The actor_polygons property will then reflect this change.  
    world_manager.actor_dict = {} # This clears the dictionary of managed actors (Shuangyu Cai (2024-05-12, cb8721cb) in car_dreamer/toolkit/carla_manager/world_manager.py:57)  
  
    # Optionally, clear the internal cache for actor_polygons and actor_transforms  
    # if you are directly manipulating them or if the WorldManager's reset()  
    # method is not called immediately after this function.  
    # The @cached_step_wise decorator (Shuangyu Cai (2024-05-12, cb8721cb) in car_dreamer/toolkit/carla_manager/world_manager.py:12-22)  
    # means these properties are cached per simulation step.  
    # If you want to force an immediate refresh, you can clear the cache.  
    if hasattr(world_manager, "_cache"):  
        world_manager._cache = {"step": world_manager._time_step} # Reset cache for current step  
  
    print(f"Successfully deleted {deleted_count} out of {total_vehicles_to_delete} vehicles.")
    

def find_ego_vehicle(world):
    """Loop over all vehicles and delete them with error handling"""
    try:
        return next(v for v in world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")
    except Exception:
        return None
