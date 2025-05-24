#!/usr/bin/env python

import argparse
import logging
import re
import time

import carla
import pygame

from manual_control import KeyboardControl
from sensors import FakeRadarSensor, CollisionSensor, GnssSensor, CameraSet
from hud import get_actor_display_name, HUD

from manual_control import KeyboardControl
from sensors import FakeRadarSensor, CollisionSensor, GnssSensor, CameraSet
from hud import get_actor_display_name, HUD

from agents.navigation.roaming_agent import RoamingAgent
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.autonomous_agent import AutonomousAgent
import time


class World(object):
    """
    Creates a object which has all the necessary attributes of carla world and ego vehicle. For example
    --> Carla World (a world Object)
    --> Weather Presets
    --> List of sensors for ego vehicle
    --> List of other vwhicle in the world
    
    """
    def __init__(self, carla_world, hud, agent_str, player=None):
        # Carla World
        self.world = carla_world
        self.map = self.world.get_map()
        self.hud = hud
        
        self.world.on_tick(hud.on_world_tick)
        
        # Weather presets
        self._weather_index = 0
        self._weather_presets = None
        self.find_weather_presets()
        
        # Autonomous vehicle and sensors variables
        self.player = None                  # ego vehicle
        self.collision_sensor = None        # sensors
        self.gnss_sensor = None
        self._ego_list = []
        # Other actors
        self._actor_list = []
        # camera manager
        # self.main_rgb_camera = None
        self.depth_camera = None

        # radar set
        self.front_radar = None
        self.back_radar = None
        self.left_front_radar = None
        self.right_front_radar = None

        # Agent
        self.agent_name = agent_str
        self.agent = None
        self.autopilot_mode = True      
        self.player = player
        self.restart()


    def restart(self):
        # Destroy existing ego and actors
        if self._ego_list or self._actor_list:
            self.destroy()

        # get vehicle
        if self.player is None:
            total_connect_attempts = 40
            for i in range(total_connect_attempts):
                try:
                    self.player = next(v for v in self.world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")
                except Exception:
                    # self.get_logger().info(f"attempt {i}/{total_connect_attempts} failed, retrying in 1 scond")
                    print(f"attempt {i}/{total_connect_attempts} failed, retrying in 1 scond")
                    time.sleep(1)
            

        if self.player is None:
            print("Ego vehicle spawn location occupied.\n Spawning failed!")
            return

        # Set up the sensors.
        # self.main_rgb_camera = CameraSet(self.player, self.hud)

        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)

        self.front_radar = FakeRadarSensor(self.player, self.hud, x=0.5, y=0.0, z=1.0, yaw=0.0, fov=5)
        self.left_front_radar = FakeRadarSensor(self.player, self.hud, x=1.0, y=-0.5, z=1.0, yaw=-25.0)
        self.left_back_radar = FakeRadarSensor(self.player, self.hud, x=-1.0, y=-0.5, z=1.0, yaw=-155.0)

        self._ego_list = [
            # self.main_rgb_camera,
            self.collision_sensor.sensor,
            self.gnss_sensor.sensor,
            self.front_radar,
            self.left_front_radar,
            self.left_back_radar,
            self.player]
        
        # Reset agent
        if self.agent_name == "Autonomous":
            self.agent = AutonomousAgent(self)
            # destination Setting
            self.agent.set_destination((100, -4.5, 0))
        elif self.agent_name == "Basic":
            self.agent = BasicAgent(self.player)
            # destination Setting
            self.agent.set_destination((300, 45, 0))
        else:
            self.agent = RoamingAgent(self.player)

        # Display
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def find_weather_presets(self):
        rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
        name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
        presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
        self._weather_presets = [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])    
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        # self.main_rgb_camera.render(display)
        self.hud.render(display)

    def enable_agent(self, enabled):
        self.autopilot_mode = enabled

    def destroy(self):
        for actor in self._ego_list:
            if actor is not None:
                actor.destroy()
        self._ego_list = []
        
        for actor in self._actor_list:
            if actor is not None:
                # disconnect from client
                actor.set_autopilot(False)
                actor.destroy()
        self._actor_list = []



# Main loop
def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        # Connect to client
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        # Display setting
        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        hud = HUD(args.width, args.height)

        # Create ego world and spawn ego vehicle
        world = World(client.get_world(), hud, args.agent)
        if world.player is None:
            return

        # Keyboard controller set up
        controller = KeyboardControl(world, start_in_autopilot=True)

        # Manually start the vehicle to avoid control delay
        world.player.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
        world.player.apply_control(carla.VehicleControl(manual_gear_shift=False))

        clock = pygame.time.Clock()

        while True:
            # Keyboard control
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return

            # as soon as the server is ready continue!
            # world.world.wait_for_tick(5.0)
            world.tick(clock)
            world.render(display)
            pygame.display.flip()
        
            control = world.agent.run_step(debug=True)

            # Agent autopilot
            if world.autopilot_mode:
                # control signal to vehicle        
                control.manual_gear_shift = False
                world.player.apply_control(control)

    finally:
        if world is not None:
            world.destroy()

        pygame.quit()


def main():
    # Arguments
    argparser = argparse.ArgumentParser(description="CARLA Ego Vehicle Client")
    argparser.add_argument("-v", "--verbose", action="store_true", dest="debug",
                           help="print debug information")
    argparser.add_argument("--host", metavar="H", default="127.0.0.1",
                           help="IP of the host server (default: 127.0.0.1)")
    argparser.add_argument("-p", "--port", metavar="P", default=2000, type=int,
                           help="TCP port to listen to (default: 2000)")
    argparser.add_argument("--res", metavar="WIDTH x HEIGHT", default="1280x720",
                           help="window resolution")
    argparser.add_argument("-l", "--location", metavar="Spawn Location", default=[200, 7.5, 0.5], type=list,
                           help="spawn location (default: [170, 7.5, 0.5])")
    argparser.add_argument("--filter", metavar="PATTERN", default="vehicle.tesla.model3",
                           help="actor filter (default: 'vehicle.tesla.model3')")
    argparser.add_argument("-a", "--agent", type=str, choices=["Roaming", "Basic", "Autonomous"], default="Autonomous",
                           help="select which agent to run")
    argparser.add_argument("-s", "--scene", type=str, choices=['0', '1', '2'], default='0',
                           help="select which scene to run")


    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]

    # Logging process
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)
    logging.info("listening to server %s:%s", args.host, args.port)

    # Start the loop
    try:
        game_loop(args)

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")


if __name__ == "__main__":
    main()
