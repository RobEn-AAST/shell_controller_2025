# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


"""This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations."""

import random
import numpy as np
import carla
from ai_src.carla_others.agents.navigation.basic_agent import BasicAgent
from ai_src.carla_others.agents.navigation.local_planner import RoadOption
from ai_src.carla_others.agents.navigation.behavior_types import Cautious, Aggressive, Normal

from ai_src.carla_others.agents.tools.misc import get_speed, positive, is_within_distance, compute_distance


class BehaviorAgent(BasicAgent):
    """
    BehaviorAgent implements an agent that navigates scenes to reach a given
    target destination, by computing the shortest possible path to it.
    This agent can correctly follow traffic signs, speed limitations,
    traffic lights, while also taking into account nearby vehicles. Lane changing
    decisions can be taken by analyzing the surrounding environment such as tailgating avoidance.
    Adding to these are possible behaviors, the agent can also keep safety distance
    from a car in front of it by tracking the instantaneous time to collision
    and keeping it in a certain range. Finally, different sets of behaviors
    are encoded in the agent, from cautious to a more aggressive ones.
    """

    def __init__(self, vehicle, behavior="normal", opt_dict={}, map_inst=None, grp_inst=None):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param behavior: type of agent to apply
        """

        super().__init__(vehicle, opt_dict=opt_dict, map_inst=map_inst, grp_inst=grp_inst)
        self._look_ahead_steps = 0

        # Vehicle information
        self._speed = 0
        self._speed_limit = 0
        self._direction = None
        self._incoming_direction = None
        self._incoming_waypoint = None
        self._min_speed = 5
        self._behavior = None
        self._sampling_resolution = 4.5

        self._is_overtaking = False
        self._n_overtake_wps_to_shift = 19
        self._n_overtake_smooth_wps = 3
        self._n_overtake_done_margin = 0
        self._overtake_wps_count = self._n_overtake_wps_to_shift + self._n_overtake_smooth_wps + self._n_overtake_done_margin  # 5 is safety factor
        self._initial_queue_size = 0
        self._normal_max_speed = 40.0
        self._overtake_max_speed = 25.0

        self.vehicle_ahead_state = False
        self.vehicle_ahead = None
        self.distance_ahead = float("inf")

        self.ignore_traffic_lights(True)
        self.ignore_stop_signs(True)

        # Parameters for agent behavior
        if behavior == "cautious":
            self._behavior = Cautious()

        elif behavior == "normal":
            self._behavior = Normal()

        elif behavior == "aggressive":
            self._behavior = Aggressive()
        else:
            raise NameError("behavior type is not defined")

        self._behavior.max_speed = self._normal_max_speed

    def _update_information(self):
        """
        This method updates the information regarding the ego
        vehicle based on the surrounding world.
        """
        self._speed = get_speed(self._vehicle)
        self._speed_limit = self._vehicle.get_speed_limit()
        self._local_planner.set_speed(self._speed_limit)
        self._direction = self._local_planner.target_road_option
        if self._direction is None:
            self._direction = RoadOption.LANEFOLLOW

        self._look_ahead_steps = int((self._speed_limit) / 10)

        self._incoming_waypoint, self._incoming_direction = self._local_planner.get_incoming_waypoint_and_direction(steps=self._look_ahead_steps)
        if self._incoming_direction is None:
            self._incoming_direction = RoadOption.LANEFOLLOW

    def traffic_light_manager(self):
        """
        This method is in charge of behaviors for red lights.
        """
        actor_list = self._world.get_actors()
        lights_list = actor_list.filter("*traffic_light*")
        affected, _ = self._affected_by_traffic_light(lights_list)

        return affected

    def _tailgating(self, waypoint, vehicle_list):
        """
        This method is in charge of tailgating behaviors.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :param vehicle_list: list of all the nearby vehicles
        """

        left_turn = waypoint.left_lane_marking.lane_change
        right_turn = waypoint.right_lane_marking.lane_change

        left_wpt = waypoint.get_left_lane()
        right_wpt = waypoint.get_right_lane()

        behind_vehicle_state, behind_vehicle, _ = self._vehicle_obstacle_detected(vehicle_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, low_angle_th=160)
        if behind_vehicle_state and self._speed < get_speed(behind_vehicle):
            if (right_turn == carla.LaneChange.Right or right_turn == carla.LaneChange.Both) and waypoint.lane_id * right_wpt.lane_id > 0 and right_wpt.lane_type == carla.LaneType.Driving:
                new_vehicle_state, _, _ = self._vehicle_obstacle_detected(vehicle_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=1)
                if not new_vehicle_state:
                    print("Tailgating, moving to the right!")
                    end_waypoint = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = 200
                    self.set_destination(end_waypoint.transform.location, right_wpt.transform.location)
            elif left_turn == carla.LaneChange.Left and waypoint.lane_id * left_wpt.lane_id > 0 and left_wpt.lane_type == carla.LaneType.Driving:
                new_vehicle_state, _, _ = self._vehicle_obstacle_detected(vehicle_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=-1)
                if not new_vehicle_state:
                    print("Tailgating, moving to the left!")
                    end_waypoint = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = 200
                    self.set_destination(end_waypoint.transform.location, left_wpt.transform.location)

    def _create_traj(self, lane_direction: str):
        assert lane_direction in ["left", "right"], "lane direction can be only 'right' or 'left'"

        current_planned_waypoints = list(self._local_planner._waypoints_queue)
        if not current_planned_waypoints:
            print("No current planned waypoints to shift.")
            return

        modified_waypoints = []
        for i, (wp_carla, road_option) in enumerate(current_planned_waypoints):
            if i >= self._n_overtake_wps_to_shift:
                modified_waypoints.append((wp_carla, road_option))
                continue

            shifted_waypoint = wp_carla.get_left_lane() if lane_direction == "left" else wp_carla.get_right_lane()

            if shifted_waypoint and shifted_waypoint.lane_type == carla.LaneType.Driving:
                modified_waypoints.append((shifted_waypoint, road_option))
            else:
                modified_waypoints.append((wp_carla, road_option))

        # smoothness:
        # Remove first 7 points and 7 points after 40th element for smoothness
        modified_waypoints = modified_waypoints[self._n_overtake_smooth_wps : self._n_overtake_wps_to_shift] + modified_waypoints[self._n_overtake_smooth_wps + self._n_overtake_wps_to_shift :]

        self._local_planner.set_global_plan(modified_waypoints[10:], clean_queue=True)
        self._behavior.tailgate_counter = 200
        self._is_overtaking = True
        self._initial_queue_size = len(self._local_planner._waypoints_queue)

    def _headgating(self, waypoint, vehicle_list, force_lane_switch=False):
        left_wpt = waypoint.get_left_lane()
        right_wpt = waypoint.get_right_lane()

        if left_wpt and left_wpt.lane_type == carla.LaneType.Driving:
            new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                vehicle_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=180,
                lane_offset=-1,
            )
            if not new_vehicle_state:
                self._create_traj("left")

        # Try right lane if left is not possible/clear
        elif right_wpt and right_wpt.lane_type == carla.LaneType.Driving:
            new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                vehicle_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=180,
                lane_offset=1,
            )
            if not new_vehicle_state:  # If target lane is clear
                self._create_traj("right")

        else:
            print("Forced lane change requested, but no valid adjacent driving lane found or clear.")

            # Fallback: try to find any adjacent lanes regardless of type
            current_wpt = waypoint

            map_obj = self._map
            try:
                left_lane_id = current_wpt.lane_id - 1
                left_fallback_wpt = map_obj.get_waypoint_xodr(current_wpt.road_id, left_lane_id, current_wpt.s)
                if left_fallback_wpt:
                    new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                        vehicle_list,
                        max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                        up_angle_th=180,
                        lane_offset=-1,
                    )
                    if not new_vehicle_state:
                        self._create_traj("left")
                        return
            except:
                pass

            try:
                right_lane_id = current_wpt.lane_id + 1
                right_fallback_wpt = map_obj.get_waypoint_xodr(current_wpt.road_id, right_lane_id, current_wpt.s)
                if right_fallback_wpt:
                    new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                        vehicle_list,
                        max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                        up_angle_th=180,
                        lane_offset=1,
                    )
                    if not new_vehicle_state:
                        self._create_traj("right")
                        return
            except:
                pass

    def collision_and_car_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        and managing possible tailgating chances.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a vehicle nearby, False if not
            :return vehicle: nearby vehicle
            :return distance: distance to nearby vehicle
        """

        vehicle_list = self._world.get_actors().filter("*vehicle*")

        def dist(v):
            return v.get_location().distance(waypoint.transform.location)

        vehicle_list = [v for v in vehicle_list if dist(v) < 45 and v.id != self._vehicle.id]

        if self._direction == RoadOption.CHANGELANELEFT:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(vehicle_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=-1)
        elif self._direction == RoadOption.CHANGELANERIGHT:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(vehicle_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=1)
        else:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(vehicle_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 3), up_angle_th=30)

            # Check for tailgating
            if not vehicle_state and self._direction == RoadOption.LANEFOLLOW and not waypoint.is_junction and self._speed > 10 and self._behavior.tailgate_counter == 0:
                self._tailgating(waypoint, vehicle_list)

        return vehicle_state, vehicle, distance

    def pedestrian_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        with any pedestrian.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a walker nearby, False if not
            :return vehicle: nearby walker
            :return distance: distance to nearby walker
        """

        walker_list = self._world.get_actors().filter("*walker.pedestrian*")

        def dist(w):
            return w.get_location().distance(waypoint.transform.location)

        walker_list = [w for w in walker_list if dist(w) < 10]

        if self._direction == RoadOption.CHANGELANELEFT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(walker_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=90, lane_offset=-1)
        elif self._direction == RoadOption.CHANGELANERIGHT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(walker_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=90, lane_offset=1)
        else:
            walker_state, walker, distance = self._vehicle_obstacle_detected(walker_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 3), up_angle_th=60)

        return walker_state, walker, distance

    def car_following_manager(self, vehicle, distance, debug=False):
        """
        Module in charge of car-following behaviors when there's
        someone in front of us.

            :param vehicle: car to follow
            :param distance: distance from vehicle
            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """

        vehicle_speed = get_speed(vehicle)
        delta_v = max(1, (self._speed - vehicle_speed) / 3.6)
        ttc = distance / delta_v if delta_v != 0 else distance / np.nextafter(0.0, 1.0)

        # Under safety time distance, slow down.
        if self._behavior.safety_time > ttc > 0.0:
            target_speed = min([positive(vehicle_speed - self._behavior.speed_decrease), self._behavior.max_speed, self._speed_limit - self._behavior.speed_lim_dist])
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # Actual safety distance area, try to follow the speed of the vehicle in front.
        elif 2 * self._behavior.safety_time > ttc >= self._behavior.safety_time:
            target_speed = min([max(self._min_speed, vehicle_speed), self._behavior.max_speed, self._speed_limit - self._behavior.speed_lim_dist])
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # Normal behavior.
        else:
            target_speed = min([self._behavior.max_speed, self._speed_limit - self._behavior.speed_lim_dist])
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        return control

    def _should_overtake(self, check_distance_meters=50):
        vehicle_list = self._world.get_actors().filter("*vehicle*")
        vehicle_list = [v for v in vehicle_list if v.id != self._vehicle.id]
        self.vehicle_ahead_state, self.vehicle_ahead, self.distance_ahead = self._vehicle_obstacle_detected(
            vehicle_list, max(self._behavior.min_proximity_threshold, self._speed_limit / 3), up_angle_th=30, lane_offset=0
        )

        if self.vehicle_ahead is None:
            return False

        # Check if there's a slow vehicle ahead that warrants overtaking
        if not (self.vehicle_ahead_state and self.vehicle_ahead and get_speed(self.vehicle_ahead) <= 2):
            return False

        # Check if left lane is clear for overtaking
        if not self._is_left_lane_clear(vehicle_list, check_distance_meters):
            return False

        return True

    def _is_left_lane_clear(self, vehicle_list, check_distance_meters):
        """
        Check if the left lane is clear for overtaking, considering vehicle direction and position.

        :param vehicle_list: List of vehicles to check against
        :param check_distance_meters: Distance to check ahead in the left lane
        :return: True if left lane is clear for overtaking, False otherwise
        """
        # Get current vehicle position and waypoint
        ego_transform = self._vehicle.get_transform()
        ego_location = ego_transform.location
        ego_yaw = ego_transform.rotation.yaw
        current_waypoint = self._map.get_waypoint(ego_location, project_to_road=True)

        # Get the left lane waypoint
        left_lane_waypoint = current_waypoint.get_left_lane()
        if left_lane_waypoint is None:
            return False  # No left lane available

        # Check for vehicles in the left lane
        for vehicle in vehicle_list:
            vehicle_transform = vehicle.get_transform()
            vehicle_location = vehicle_transform.location
            vehicle_yaw = vehicle_transform.rotation.yaw
            vehicle_waypoint = self._map.get_waypoint(vehicle_location, project_to_road=True)

            if vehicle_waypoint is None:
                continue

            # Check if vehicle is in the left lane (same road and lane as left_lane_waypoint)
            if vehicle_waypoint.road_id == left_lane_waypoint.road_id and vehicle_waypoint.lane_id == left_lane_waypoint.lane_id:

                # Calculate distance and relative position
                distance_to_vehicle = ego_location.distance(vehicle_location)

                # Skip if vehicle is too far away
                if distance_to_vehicle > check_distance_meters:
                    continue

                # Calculate yaw difference to determine if vehicles are facing each other
                yaw_diff = abs(ego_yaw - vehicle_yaw)
                if yaw_diff > 180:
                    yaw_diff = 360 - yaw_diff

                # Check if vehicles are facing each other (opposite directions)
                if yaw_diff > 150:  # Around 180 degrees difference
                    # Vehicles are facing each other - check if other vehicle is behind us
                    # Use dot product to determine if vehicle is ahead or behind
                    ego_forward = ego_transform.get_forward_vector()
                    to_vehicle = carla.Location(vehicle_location.x - ego_location.x, vehicle_location.y - ego_location.y, 0)

                    # If dot product is positive, vehicle is ahead of us
                    dot_product = ego_forward.x * to_vehicle.x + ego_forward.y * to_vehicle.y

                    if dot_product > 0:  # Vehicle is ahead of us and coming towards us
                        return False  # Not safe to overtake

                else:  # Vehicles are moving in roughly the same direction (yaw difference < 150)
                    vehicle_speed = self._get_vehicle_speed_kmh(vehicle)

                    if vehicle_speed > 15:
                        # Fast vehicle - check if it has already crossed us (is behind us)
                        ego_forward = ego_transform.get_forward_vector()
                        to_vehicle = carla.Location(vehicle_location.x - ego_location.x, vehicle_location.y - ego_location.y, 0)

                        dot_product = ego_forward.x * to_vehicle.x + ego_forward.y * to_vehicle.y

                        if dot_product > 0:  # Vehicle is still ahead of us
                            return False  # Not safe to overtake

                    else:  # Slow vehicle (speed <= 15)
                        # Check if vehicle is behind us by more than 15 meters
                        if distance_to_vehicle <= 15:
                            ego_forward = ego_transform.get_forward_vector()
                            to_vehicle = carla.Location(vehicle_location.x - ego_location.x, vehicle_location.y - ego_location.y, 0)

                            dot_product = ego_forward.x * to_vehicle.x + ego_forward.y * to_vehicle.y

                            # If vehicle is ahead or too close behind, not safe
                            if dot_product >= 0 or distance_to_vehicle <= 15:
                                return False

        return True  # Left lane is clear for overtaking

    def _get_vehicle_speed_kmh(self, vehicle):
        """Helper function to get vehicle speed in km/h"""
        velocity = vehicle.get_velocity()
        speed_ms = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5
        return speed_ms * 3.6  # Convert m/s to km/h  # Left lane is clear

    def run_step(self, debug=False, force_lane_switch=False):
        """
        Execute one step of navigation.

            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """
        self._update_information()

        control = None

        if self._behavior.tailgate_counter > 0:
            self._behavior.tailgate_counter -= 1

        ego_vehicle_loc = self._vehicle.get_location()
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)

        if self._is_overtaking:
            self._behavior.max_speed = self._overtake_max_speed
            current_queue_size = len(self._local_planner._waypoints_queue)

            waypoints_consumed = self._initial_queue_size - current_queue_size
            print(f'wps consumed: {waypoints_consumed}')

            if waypoints_consumed >= self._overtake_wps_count:
                self._behavior.max_speed = self._normal_max_speed
                self._is_overtaking = False
                self._initial_queue_size = 0
                self._behavior.max_speed = self._normal_max_speed
                print("Overtake completed based on waypoint consumption!")

        if (force_lane_switch or self._should_overtake()) and not self._is_overtaking:
            vehicle_list = self._world.get_actors().filter("*vehicle*")
            vehicle_list = [v for v in vehicle_list if v.id != self._vehicle.id]

            self._headgating(ego_vehicle_wp, vehicle_list, force_lane_switch=force_lane_switch)

        # 2.2: Car following behaviors
        vehicle_state, vehicle, distance = self.collision_and_car_avoid_manager(ego_vehicle_wp)

        if self._incoming_waypoint is None:
            return

        if vehicle_state:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            distance = distance - max(vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x) - max(self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)

            # Emergency brake if the car is very close.
            # Emergency stop conditions
            needs_emergency_stop = (distance < self._behavior.min_proximity_threshold and self._is_overtaking and self._speed > self._overtake_max_speed) or (
                distance < self._behavior.braking_distance and not self._is_overtaking
            )
            if needs_emergency_stop:
                return self.emergency_stop()

            # Normal driving behavior
            control = self._local_planner.run_step(debug=debug) if self._is_overtaking else self.car_following_manager(vehicle, distance)

        # 3: Intersection behavior
        elif self._incoming_waypoint.is_junction and (self._incoming_direction in [RoadOption.LEFT, RoadOption.RIGHT]):
            target_speed = min([self._behavior.max_speed, self._speed_limit - 5])
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # 4: Normal behavior
        else:
            target_speed = min([self._behavior.max_speed, self._speed_limit - self._behavior.speed_lim_dist])
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        return control

    def emergency_stop(self):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control
