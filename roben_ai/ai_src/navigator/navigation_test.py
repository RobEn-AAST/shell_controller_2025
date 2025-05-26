# todo make the pause work
# todo make changing destination work

import carla
import sys
import pygame
import numpy as np
import random
import string
from test_lib.world_utils import *
from test_lib.agent_utils import *
import time

pygame.init()

client = carla.Client("localhost", 2000)  # type: ignore
client.set_timeout(20)
world = client.get_world()
carla_map = world.get_map()
competition_pos = {
    "x": 280.363708,
    "y": 1.306351,
    "z": 0.015964,
    "roll": 0,
    "pitch": 0,
    "yaw": 180,
}
destination = [184.758713,-199.424271,0.001680]

ego_vehicle = find_ego_vehicle(world)
loading_town = False
status_text = ""
follow_ego = False
ego_pos = None
pause = False
enable_agent = False
agent = None

# Add these below the other global variables
input_active = False
input_text = {"x": "", "y": "", "z": ""}
selected_input = None
destination_input = [str(x) for x in destination]  # Convert initial destination to strings

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
    return ego_vehicle, agent



running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False

            # toggle agent driving
            elif event.key == pygame.K_a:
                enable_agent = not enable_agent
            
            # set destination
            elif event.key == pygame.K_b:
                if agent is None:
                    set_status("Agent is None, operation ailed")
                else:
                    agent.set_destination(destination) 
                    set_status("Succesfully set destination")

            # spawn ego vehicle
            elif event.key == pygame.K_e:
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
                delete_all_vehicles_safe(world)
                ego_vehicle = None

            # follow ego
            elif event.key == pygame.K_f:
                if ego_vehicle is None:
                    ego_vehicle = find_ego_vehicle(world)
                follow_ego = not follow_ego

            # Spawn vehicle infront of us
            elif event.key == pygame.K_s:
                spawn_background_infront(world, ego_vehicle)

            # pause carla
            elif event.key == pygame.K_p:
                pause = not pause
            elif event.key == pygame.K_l and pause:
                world.tick()
            # reset scenario
            elif event.key == pygame.K_r:
                delete_all_vehicles_safe(world)
                time.sleep(0.3)
                ego_vehicle, agent = spawn_ego()
                spawn_background_infront(world, ego_vehicle)
                follow_ego = True

    # carla cam follow vehicle
    if follow_ego and ego_vehicle is not None:
        ego_pos = get_vehicle_position(ego_vehicle)
        set_camera_position(world, ego_pos.x, ego_pos.y, ego_pos.z + 40)
    
    if enable_agent:
        if agent is None:
            set_status("Agent is None, operation failed...")
        else:
            control = agent.run_step()
            ego_vehicle.apply_control(control)

    # Draw front-camera feed or fallback background
    if ego_vehicle:
        img = get_front_camera_image(world, ego_vehicle)  # numpy 2D array
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
    vehicle_count = len(world.get_actors().filter('vehicle.*'))
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

    # Destination input fields
    input_y = HEIGHT - 150
    for i, (axis, value) in enumerate(zip(['X', 'Y', 'Z'], destination_input)):
        # Input field background
        pygame.draw.rect(screen, (50, 50, 50), (WIDTH - 200, input_y + i*30, 150, 25))
        
        # Label
        label = font.render(f"Dest {axis}:", True, (255, 255, 255))
        screen.blit(label, (WIDTH - 280, input_y + i*30 + 5))
        
        # Input text
        text = destination_input[i]
        text_surf = font.render(text, True, (255, 255, 255))
        screen.blit(text_surf, (WIDTH - 190, input_y + i*30 + 5))

    # Current destination and distance
    if ego_vehicle and agent:
        current_pos = get_vehicle_position(ego_vehicle)
        dest = carla.Location(float(destination[0]), float(destination[1]), float(destination[2]))
        distance = current_pos.distance(dest)
        
        dest_text = f"Current destination: ({destination[0]:.1f}, {destination[1]:.1f}, {destination[2]:.1f})"
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
                        destination[selected_input] = float(destination_input[selected_input])
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
                    if event.unicode.isnumeric() or event.unicode in '.-':
                        destination_input[selected_input] += event.unicode
            
            # spawn ego vehicle
            elif event.key == pygame.K_e:
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
                delete_all_vehicles_safe(world)
                ego_vehicle = None

            # follow ego
            elif event.key == pygame.K_f:
                if ego_vehicle is None:
                    ego_vehicle = find_ego_vehicle(world)
                follow_ego = not follow_ego

            # Spawn vehicle infront of us
            elif event.key == pygame.K_s:
                spawn_background_infront(world, ego_vehicle)

            # pause carla
            elif event.key == pygame.K_p:
                pause = not pause
            elif event.key == pygame.K_l and pause:
                world.tick()
            # reset scenario
            elif event.key == pygame.K_r:
                delete_all_vehicles_safe(world)
                ego_vehicle, agent = spawn_ego()
                spawn_background_infront(world, ego_vehicle)
                follow_ego = True

        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = pygame.mouse.get_pos()
            # Check if click is in input field area
            input_y = HEIGHT - 150
            for i in range(3):
                input_rect = pygame.Rect(WIDTH - 200, input_y + i*30, 150, 25)
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
        "B: Set Agent Destination",
        "E: Spawn Ego Vehicle",
        "S: Spawn Infront of Ego",
        "R: Reset Scenario",
        "T: Load Town",
        "D: Delete Vehicles",
        "F: Follow Ego",
        "P: Toggle Pause",
        "L: Run Step",
    ]
    for i, text in enumerate(instructions):
        instr_surf = font.render(text, True, (200, 200, 200))
        y = HEIGHT - 10 - (len(instructions) - 1 - i) * 20
        instr_rect = instr_surf.get_rect(midbottom=(WIDTH // 2, y))
        screen.blit(instr_surf, instr_rect)

    # Right-side overlay: vehicle speed & position
    if ego_vehicle:
        speed = get_vehicle_speed(ego_vehicle)
        ego_pos = get_vehicle_position(ego_vehicle)
        info_lines = [
            f"Speed: {speed:.2f} m/s",
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

    pygame.display.flip()
    clock.tick(60)

    if not pause:
        world.tick()

pygame.quit()
sys.exit()
