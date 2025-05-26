import carla
import queue
import numpy as np

def get_vehicle_speed(vehicle):
    """Get the speed of a vehicle in m/s"""
    velocity = vehicle.get_velocity()
    speed = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5
    return speed


def get_vehicle_position(vehicle):
    """Get the position (location) of a vehicle"""
    transform = vehicle.get_transform()
    location = transform.location
    return location  # Returns carla.Location with x, y, z coordinates


def spawn_vehicle(world, role_name, x, y, z, roll, pitch, yaw):
    """Spawn a vehicle with specified role_name and transform parameters"""
    # Get blueprint library and find a vehicle blueprint
    blueprint_library = world.get_blueprint_library()
    vehicle_blueprints = blueprint_library.filter("vehicle.*")

    # Choose a random vehicle blueprint (you can modify this to select specific ones)
    import random

    vehicle_bp = random.choice(vehicle_blueprints)

    # Set the role_name attribute
    if vehicle_bp.has_attribute("role_name"):
        vehicle_bp.set_attribute("role_name", role_name)

    # Create transform with specified position and rotation
    location = carla.Location(x=x, y=y, z=z)
    rotation = carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
    transform = carla.Transform(location, rotation)

    # Spawn the vehicle
    vehicle = world.try_spawn_actor(vehicle_bp, transform)
    return vehicle


def load_town(client, town_name="Town01_Opt", should_sync=True):
    """Load Town01_Opt map and set synchronous mode to True"""
    world = client.load_world(town_name)
    settings = world.get_settings()
    settings.synchronous_mode = should_sync
    world.apply_settings(settings)

    return world


def set_camera_position(world, x, y, z, pitch=-90, yaw=0, roll=0):  
    """Set the spectator camera to look at a specific x, y, z position"""  
    # Get the spectator actor  
    spectator = world.get_spectator()  
      
    # Create the transform with the desired location and rotation  
    location = carla.Location(x=x, y=y, z=z)  
    rotation = carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)  
    transform = carla.Transform(location, rotation)  
      
    # Set the spectator transform  
    spectator.set_transform(transform)

def delete_all_vehicles_safe(world):  
    """Loop over all vehicles and delete them with error handling"""  
    vehicles = world.get_actors().filter('*vehicle*')  
    deleted_count = 0  
      
    for vehicle in vehicles:  
        try:  
            if vehicle.destroy():  
                deleted_count += 1  
        except Exception as e:  
            print(f"Failed to delete vehicle {vehicle.id}: {e}")  
      
    print(f"Successfully deleted {deleted_count} out of {len(vehicles)} vehicles")


def find_ego_vehicle(world):  
    """Loop over all vehicles and delete them with error handling"""  
    try:
        return next(v for v in world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")
    except Exception:
        return None


def get_front_camera_image(world, vehicle, image_width=800, image_height=600, fov=90.0):  
    """Capture a front camera image from a vehicle"""  
    # Get the blueprint library and find the RGB camera sensor  
    blueprint_library = world.get_blueprint_library()  
    camera_bp = blueprint_library.find('sensor.camera.rgb')  
      
    # Set camera attributes  
    camera_bp.set_attribute('image_size_x', str(image_width))  
    camera_bp.set_attribute('image_size_y', str(image_height))  
    camera_bp.set_attribute('fov', str(fov))  
      
    # Create camera transform (front of vehicle, slightly elevated)  
    camera_transform = carla.Transform(carla.Location(x=2.0, z=1.5))  
      
    # Spawn the camera attached to the vehicle  
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)  
      
    # Create a queue to store the image  
    image_queue = queue.Queue()  
    camera.listen(image_queue.put)  
      
    # Wait for the first image  
    world.tick()  # Advance simulation to capture image  
    image = image_queue.get()  
      
    # Convert raw data to numpy array  
    array = np.frombuffer(image.raw_data, dtype=np.uint8)  
    array = np.reshape(array, (image.height, image.width, 4))  # BGRA format  
    array = array[:, :, :3]  # Remove alpha channel, keep BGR  
    array = array[:, :, ::-1]  # Convert BGR to RGB  
      
    # Clean up - destroy the camera  
    camera.destroy()  
      
    return array