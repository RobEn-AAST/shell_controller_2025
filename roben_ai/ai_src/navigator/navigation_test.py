import carla
import sys
import pygame
import numpy as np
from test_lib.utils import *

pygame.init()

client = carla.Client("localhost", 2000)  # type: ignore
client.set_timeout(20)
world = client.get_world()
carla_map = world.get_map()

ego_vehicle = None
vehicles = []
loading_town = False
status_text = ""

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
font = pygame.font.SysFont(None, 24)
pygame.display.set_caption("Press Q to Quit")

clock = pygame.time.Clock()


def set_status(text: str):
    global status_text
    status_text = text


running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False

            elif event.key == pygame.K_e:
                set_status("Spawning ego vehicle...")
                ego_vehicle = spawn_vehicle(
                    world,
                    "ego_vehicle",
                    x=280.363708, y=1.306351, z=0.015964,
                    roll=0, pitch=0, yaw=180
                )
                vehicles.append(ego_vehicle)
                pos = get_vehicle_position(ego_vehicle)
                set_camera_position(world, pos.x, pos.y, pos.z + 40)
                set_status("Done spawning ego vehicle")

            elif event.key == pygame.K_t and not loading_town:
                set_status("Loading town...")
                loading_town = True
                load_town(client, should_sync=False)
                loading_town = False
                set_status("Done loading town")

            elif event.key == pygame.K_d:
                delete_all_vehicles_safe(world)

            elif event.key == pygame.K_f:
                if ego_vehicle is None:
                    ego_vehicle = find_ego_vehicle(world)
                if ego_vehicle is not None:
                    pos = get_vehicle_position(ego_vehicle)
                    set_camera_position(world, pos.x, pos.y, pos.z + 40)

    # Draw front-camera feed or fallback background
    if ego_vehicle:
        img = get_front_camera_image(world, ego_vehicle)           # numpy 2D array
        surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
        surf = pygame.transform.scale(surf, (WIDTH, HEIGHT))
        screen.blit(surf, (0, 0))
    else:
        screen.fill((30, 30, 30))

    # Top overlay: status messages
    if status_text:
        status_surf = font.render(status_text, True, (255, 255, 0))
        status_rect = status_surf.get_rect(midtop=(WIDTH // 2, 10))
        screen.blit(status_surf, status_rect)

    # Bottom overlay: key instructions
    instructions = [
        "Q: Quit   |   E: Spawn Ego Vehicle   |   T: Load Town",
        "D: Delete Vehicles   |   F: Focus On Ego"
    ]
    for i, text in enumerate(instructions):
        instr_surf = font.render(text, True, (200, 200, 200))
        y = HEIGHT - 10 - (len(instructions) - 1 - i) * 20
        instr_rect = instr_surf.get_rect(midbottom=(WIDTH // 2, y))
        screen.blit(instr_surf, instr_rect)

    # Right-side overlay: vehicle speed & position
    if ego_vehicle:
        speed = get_vehicle_speed(ego_vehicle)
        pos = get_vehicle_position(ego_vehicle)
        info_lines = [
            f"Speed: {speed:.2f} m/s",
            f"Pos X: {pos.x:.2f}",
            f"Pos Y: {pos.y:.2f}",
            f"Pos Z: {pos.z:.2f}",
        ]
    else:
        info_lines = ["Speed: N/A", "Pos: N/A"]

    for i, line in enumerate(info_lines):
        info_surf = font.render(line, True, (255, 255, 255))
        info_rect = info_surf.get_rect(topright=(WIDTH - 10, 10 + i * 20))
        screen.blit(info_surf, info_rect)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()
