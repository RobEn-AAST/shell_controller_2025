# todo make the pause work
# todo make changing destination work

import carla
import sys
import pygame
import numpy as np
from test_lib.world_utils import *
from test_lib.agent_utils import *
import time
import os
import yaml
from test_lib.car_dreamer.car_dreamer.toolkit.config import Config

pygame.init()

client = carla.Client("localhost", 2000)  # type: ignore
client.set_timeout(20)
world = client.get_world()
config_path = "shell_controller_2025/roben_ai/ai_src/navigator/test_lib/car_dreamer/car_dreamer/configs/common.yaml"
with open(config_path) as f:
    common_config_dict = yaml.safe_load(f)
env_config = Config(common_config_dict)
world_manager = WorldManager(client, world, env_config.env)

# rendered definition
obs_range = 50
screen_size = 512
pixels_per_meter = screen_size / (obs_range * 2)
pixels_ahead_vehicle = obs_range * pixels_per_meter / 2
obs_range = 50
renderer = BirdeyeRenderer(
    world_manager=world_manager,
    pixels_per_meter=pixels_per_meter,
    screen_size=screen_size,
    pixels_ahead_vehicle=int(pixels_ahead_vehicle),
    sight_fov=150,  # Field of view in degrees
    sight_range=obs_range,
)
carla_map = world.get_map()
competition_pos = {
    "x": 130,  # 280.363708,
    "y": 59.5,  # 129.306351,
    "z": 0.002264,
    "roll": 0,
    "pitch": 0,
    "yaw": 0,  # 180,
}
destination = carla.Location(184.758713, -199.424271, 0.001680)

ego_vehicle = find_ego_vehicle(world)
loading_town = False
status_text = ""
follow_ego = False
ego_pos = None
pause = False
enable_agent = False
agent = None
force_lane_switch = False

# Add these below the other global variables
input_active = False
input_text = {"x": "", "y": "", "z": ""}
selected_input = None
destination_input = [str(x) for x in [destination.x, destination.y, destination.z]]

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
font = pygame.font.SysFont(None, 24)
pygame.display.set_caption("Press Q to Quit")

clock = pygame.time.Clock()


def set_status(text: str):
    global status_text
    status_text = text


def spawn_ego():
    ego_vehicle = None
    while ego_vehicle is None:
        ego_vehicle = spawn_vehicle(
            world,
            "ego_vehicle",
            x=competition_pos["x"],
            y=competition_pos["y"],
            z=competition_pos["z"],
            roll=competition_pos["roll"],
            pitch=competition_pos["pitch"],
            yaw=competition_pos["yaw"],
        )
    ego_pos = get_vehicle_position(ego_vehicle)
    set_camera_position(world, ego_pos.x, ego_pos.y, ego_pos.z + 40)

    agent = set_agent(ego_vehicle)
    agent.ignore_traffic_lights(True)

    agent.set_destination(destination)
    return ego_vehicle, agent


def apply_agent(agent: BehaviorAgent, ego_vehicle: carla.Vehicle):
    control = agent.run_step(force_lane_switch=force_lane_switch)
    if control is not None:
        ego_vehicle.apply_control(control)

running = True
while running:
    # carla cam follow vehicle
    update_all_vehicles_polygons(world_manager)
    if follow_ego and ego_vehicle is not None:
        ego_pos = get_vehicle_position(ego_vehicle)
        set_camera_position(world, ego_pos.x, ego_pos.y, ego_pos.z + 40)

    if enable_agent and ego_vehicle is not None:
        if agent is None:
            set_status("Agent is None, operation failed...")
        else:
            apply_agent(agent, ego_vehicle)

    # Draw front-camera feed or fallback background
    if ego_vehicle:
        # img = get_front_camera_image(world, ego_vehicle)  # numpy 2D array
        waypoints = extract_agent_wps(agent)
        img = visualize_agent_path(renderer, ego_vehicle, waypoints, screen_size)
        surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
        surf = pygame.transform.scale(surf, (WIDTH, HEIGHT))
        screen.blit(surf, (0, 0))
    else:
        screen.fill((30, 30, 30))

    # Draw semi-transparent background for text overlays
    overlay = pygame.Surface((WIDTH, 30))
    overlay.fill((0, 0, 0))
    overlay.set_alpha(128)
    screen.blit(overlay, (0, 0))

    # Status text with semi-transparent background
    if status_text:
        status_surf = font.render(status_text, True, (255, 255, 0))
        status_rect = status_surf.get_rect(midtop=(WIDTH // 2, 10))
        screen.blit(status_surf, status_rect)

    # Vehicle count
    vehicle_count = len(world.get_actors().filter("vehicle.*"))
    count_text = f"Vehicles: {vehicle_count}"
    count_surf = font.render(count_text, True, (255, 255, 255))
    screen.blit(count_surf, (10, 10))

    # Agent status
    agent_status = "Agent: Enabled" if enable_agent else "Agent: Disabled"
    agent_surf = font.render(agent_status, True, (0, 255, 0) if enable_agent else (255, 0, 0))
    screen.blit(agent_surf, (10, 40))

    # Follow status
    follow_status = "Following: Yes" if follow_ego else "Following: No"
    follow_surf = font.render(follow_status, True, (0, 255, 0) if follow_ego else (255, 0, 0))
    screen.blit(follow_surf, (10, 70))

    # Force Lane Switch Status
    force_lane_status = "Force Lane Switch: Yes" if force_lane_switch else "Force Lane Switch: No"
    force_lane_surf = font.render(force_lane_status, True, (0, 255, 0) if force_lane_switch else (255, 0, 0))
    screen.blit(force_lane_surf, (10, 100))

    # Destination input fields
    input_y = HEIGHT - 150
    for i, (axis, value) in enumerate(zip(["X", "Y", "Z"], destination_input)):
        # Input field background
        pygame.draw.rect(screen, (50, 50, 50), (WIDTH - 200, input_y + i * 30, 150, 25))

        # Label
        label = font.render(f"Dest {axis}:", True, (255, 255, 255))
        screen.blit(label, (WIDTH - 280, input_y + i * 30 + 5))

        # Input text
        text = destination_input[i]
        text_surf = font.render(text, True, (255, 255, 255))
        screen.blit(text_surf, (WIDTH - 190, input_y + i * 30 + 5))

    # Current destination and distance
    if ego_vehicle and agent:
        current_pos = get_vehicle_position(ego_vehicle)
        distance = current_pos.distance(destination)

        dest_text = f"Current destination: ({destination.x:.1f}, {destination.y:.1f}, {destination.z:.1f})"
        dist_text = f"Distance to goal: {distance:.1f}m"

        overlay = pygame.Surface((400, 60))
        overlay.fill((0, 0, 0))
        overlay.set_alpha(128)
        screen.blit(overlay, (WIDTH - 410, 10))

        dest_surf = font.render(dest_text, True, (255, 255, 255))
        dist_surf = font.render(dist_text, True, (255, 255, 255))
        screen.blit(dest_surf, (WIDTH - 450, 15))
        screen.blit(dist_surf, (WIDTH - 450, 40))

    # Add event handling for input fields
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False

            # toggle agent driving
            elif event.key == pygame.K_a:
                enable_agent = not enable_agent

            # Input field handling
            elif input_active and selected_input is not None:
                if event.key == pygame.K_RETURN:
                    try:
                        if selected_input == 0:
                            destination.x = float(destination_input[selected_input])
                        elif selected_input == 1:
                            destination.y = float(destination_input[selected_input])
                        elif selected_input == 2:
                            destination.z = float(destination_input[selected_input])

                        if agent is not None:
                            agent.set_destination(destination)
                            set_status("Updated destination")
                        input_active = False
                        selected_input = None
                    except ValueError:
                        set_status("Invalid number format")
                elif event.key == pygame.K_BACKSPACE:
                    destination_input[selected_input] = destination_input[selected_input][:-1]
                else:
                    if event.unicode.isnumeric() or event.unicode in ".-":
                        destination_input[selected_input] += event.unicode

            # spawn ego vehicle
            elif event.key == pygame.K_e and ego_vehicle is None:
                set_status("Spawning ego vehicle...")
                ego_vehicle, agent = spawn_ego()
                set_status("Done spawning ego vehicle")

            # loaad town
            elif event.key == pygame.K_t and not loading_town:
                set_status("Loading town...")
                loading_town = True
                load_town(client, should_sync=True)
                ego_vehicle = None
                loading_town = False
                set_status("Done loading town")

            # delete vehicles
            elif event.key == pygame.K_d:
                delete_all_vehicles_safe(world_manager)
                ego_vehicle = None

            # delete all but ego
            elif event.key == pygame.K_k:
                delete_all_vehicles_safe(world_manager, keep_ego=True)

            # follow ego
            elif event.key == pygame.K_f:
                if ego_vehicle is None:
                    ego_vehicle = find_ego_vehicle(world)
                follow_ego = not follow_ego

            # Spawn vehicle infront of us
            elif event.key == pygame.K_s:
                if ego_vehicle is None:
                    set_status("ego vehicle is none")
                else:
                    spawn_background_infront(world, client.get_trafficmanager(), ego_vehicle)
                    set_status("spawned ego vehicle")

            # pause carla
            elif event.key == pygame.K_p:
                pause = not pause
            elif event.key == pygame.K_l and pause:
                world.tick()
            elif event.key == pygame.K_i:
                force_lane_switch = not force_lane_switch
            # reset scenario
            elif event.key == pygame.K_r:
                delete_all_vehicles_safe(world_manager)
                ego_vehicle, agent = spawn_ego()
                if ego_vehicle is None:
                    set_status("ego vehicle is none")
                else:
                    spawn_background_infront(world, client.get_trafficmanager(), ego_vehicle)
                    spawn_background_infront_opposite_lane(world, client.get_trafficmanager(), ego_vehicle)
                    set_status("spawned ego vehicle")
                follow_ego = True

        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = pygame.mouse.get_pos()
            # Check if click is in input field area
            input_y = HEIGHT - 150
            for i in range(3):
                input_rect = pygame.Rect(WIDTH - 200, input_y + i * 30, 150, 25)
                if input_rect.collidepoint(mouse_pos):
                    selected_input = i
                    input_active = True
                    break
            else:
                input_active = False
                selected_input = None

    # Bottom overlay: key instructions
    instructions = [
        "Q: Quit",
        "A: Toggle Agent Driving",
        "E: Spawn Ego Vehicle",
        "S: Spawn Infront of Ego",
        "R: Reset Scenario",
        "T: Load Town",
        "D: Delete Vehicles",
        "K: Delete Vehicles But Ego",
        "F: Follow Ego",
        "I: Register Polygons",
        "P: Toggle Pause",
        "L: Run Step",
    ]

    # Create smaller font for instructions
    instruction_font = pygame.font.SysFont(None, 20)  # Reduced from 24 to 20

    # Draw semi-transparent background for instructions
    instructions_height = len(instructions) * 15  # Reduced line height
    overlay = pygame.Surface((250, instructions_height + 10))
    overlay.fill((0, 0, 0))
    overlay.set_alpha(128)
    screen.blit(overlay, (10, HEIGHT - instructions_height - 10))

    # Draw instructions
    for i, text in enumerate(instructions):
        instr_surf = instruction_font.render(text, True, (200, 200, 200))
        y = HEIGHT - 5 - (len(instructions) - 1 - i) * 15  # Reduced spacing
        instr_rect = instr_surf.get_rect(bottomleft=(15, y))  # Changed to bottomleft
        screen.blit(instr_surf, instr_rect)

    # Right-side overlay: vehicle speed & position
    if ego_vehicle:
        speed = get_vehicle_speed(ego_vehicle)
        ego_pos = get_vehicle_position(ego_vehicle)
        info_lines = [
            f"Speed: {speed * 3.6:.2f} km/h",
            f"Pos X: {ego_pos.x:.2f}",
            f"Pos Y: {ego_pos.y:.2f}",
            f"Pos Z: {ego_pos.z:.2f}",
        ]
    else:
        info_lines = ["Speed: N/A", "Pos: N/A"]

    for i, line in enumerate(info_lines):
        info_surf = font.render(line, True, (255, 255, 255))
        info_rect = info_surf.get_rect(topright=(WIDTH - 10, 10 + i * 20))
        screen.blit(info_surf, info_rect)

    # Bottom-right overlay: agent state information
    if agent and ego_vehicle:
        agent_info_lines = [
            f"Vehicle Ahead: {'Yes' if agent.vehicle_ahead_state else 'No'}",
            f"Distance to Vehicle: {agent.distance_ahead:.1f}m" if agent.distance_ahead else "Distance to Vehicle: N/A"
        ]
        
        # Calculate dimensions for the overlay
        agent_info_height = len(agent_info_lines) * 25
        agent_info_width = 250
        
        # Create and position overlay
        agent_overlay = pygame.Surface((agent_info_width, agent_info_height))
        agent_overlay.fill((0, 0, 0))
        agent_overlay.set_alpha(128)
        screen.blit(agent_overlay, (WIDTH - agent_info_width - 10, HEIGHT - agent_info_height - 10))
        
        # Draw the information
        for i, line in enumerate(agent_info_lines):
            color = (0, 255, 0) if i == 0 and agent.vehicle_ahead_state else (255, 255, 255)
            info_surf = font.render(line, True, color)
            info_rect = info_surf.get_rect(bottomright=(WIDTH - 20, HEIGHT - 10 - (len(agent_info_lines) - 1 - i) * 25))
            screen.blit(info_surf, info_rect)

    pygame.display.flip()
    clock.tick(60)

    if not pause:
        # world.tick()
        world_manager.step()

pygame.quit()
sys.exit()
