#!/usr/bin/env python

# Sensor class
import glob
import os
import sys
import weakref
import collections

try:
    sys.path.append(glob.glob('../../CARLA_Simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
from carla import ColorConverter as cc

import pygame
import numpy as np
import math

from lane_detection import *
try:
    sys.path.append('../')
except IndexError:
    pass
from agents.tools.misc import *


# Fake Radar
class FakeRadarSensor(object):  
    def __init__(self, parent_actor, hud, radar_type):  
        """  
        Initialize radar sensor without spawning any actual sensors.  
          
        :param parent_actor: The vehicle this radar is attached to  
        :param hud: HUD reference (kept for compatibility)  
        :param radar_type: 'front_radar', 'left_front_radar', or 'left_back_radar'  
        """  
        self._parent = parent_actor  
        self._world = self._parent.get_world()  
        self._map = self._world.get_map()  
        self.radar_type = radar_type  
          
        # Define detection parameters based on radar type  
        self._detection_params = self._get_detection_params(radar_type)  
          
        # Exposed properties  
        self.detected = False  
        self.rel_pos = None  
        self.rel_vel = None  
          
        # Internal state  
        self._last_obstacle = None  
        self._last_distance = None  
      
    def _get_detection_params(self, radar_type):  
        """Get detection parameters based on radar type."""  
        if radar_type == 'front_radar':  
            return {  
                'max_distance': 30, # 15.0,  
                'up_angle_th': 90,  
                'low_angle_th': 0,  
                'lane_offset': 0  
            }  
        elif radar_type == 'left_front_radar':  
            return {  
                'max_distance': 30, # 10.0,  
                'up_angle_th': 120,  
                'low_angle_th': 30,  
                'lane_offset': -1  # Left lane  
            }  
        elif radar_type == 'left_back_radar':  
            return {  
                'max_distance': 30, # 8.0,  
                'up_angle_th': 150,  
                'low_angle_th': 120,  
                'lane_offset': -1  # Left lane  
            }  
        else:  
            raise ValueError(f"Unknown radar type: {radar_type}")  
      
    def update(self):  
        """Update detection state - call this regularly to refresh detection."""  
        vehicle_list = self._world.get_actors().filter("*vehicle*")  
          
        # Use BasicAgent's vehicle detection logic  
        affected_by_vehicle, blocking_vehicle, distance = self._vehicle_obstacle_detected(  
            vehicle_list,   
            self._detection_params['max_distance'],  
            self._detection_params['up_angle_th'],  
            self._detection_params['low_angle_th'],  
            self._detection_params['lane_offset']  
        )  
          
        if affected_by_vehicle and blocking_vehicle:  
            self.detected = True  
            self._last_obstacle = blocking_vehicle  
            self._last_distance = distance  
              
            # Calculate relative position and velocity  
            self._calculate_relative_data(blocking_vehicle)  
        else:  
            self.detected = False  
            self.rel_pos = None  
            self.rel_vel = None  
            self._last_obstacle = None  
            self._last_distance = None  
      
    def _vehicle_obstacle_detected(self, vehicle_list, max_distance, up_angle_th, low_angle_th, lane_offset):  
        """  
        Adapted from BasicAgent._vehicle_obstacle_detected to work with different radar types.  
        """  
        ego_transform = self._parent.get_transform()  
        ego_location = ego_transform.location  
        ego_wpt = self._map.get_waypoint(ego_location)  
          
        # Adjust lane offset for right-hand traffic  
        if ego_wpt.lane_id < 0 and lane_offset != 0:  
            lane_offset *= -1  
          
        ego_front_transform = ego_transform  
        ego_front_transform.location += carla.Location(  
            self._parent.bounding_box.extent.x * ego_transform.get_forward_vector())  
          
        for target_vehicle in vehicle_list:  
            if target_vehicle.id == self._parent.id:  
                continue  
              
            target_transform = target_vehicle.get_transform()  
            if target_transform.location.distance(ego_location) > max_distance:  
                continue  
              
            target_wpt = self._map.get_waypoint(target_transform.location, lane_type=carla.LaneType.Any)  
              
            # Check if vehicle is in the target lane/area  
            if self.radar_type == 'front_radar':  
                # Front radar: same lane ahead  
                if target_wpt.road_id != ego_wpt.road_id or target_wpt.lane_id != ego_wpt.lane_id + lane_offset:  
                    continue  
            elif self.radar_type in ['left_front_radar', 'left_back_radar']:  
                # Side radars: check left lane  
                if target_wpt.road_id != ego_wpt.road_id or target_wpt.lane_id != ego_wpt.lane_id + lane_offset:  
                    continue  
              
            # Check angle constraints  
            target_forward_vector = target_transform.get_forward_vector()  
            target_extent = target_vehicle.bounding_box.extent.x  
            target_rear_transform = target_transform  
            target_rear_transform.location -= carla.Location(  
                x=target_extent * target_forward_vector.x,  
                y=target_extent * target_forward_vector.y,  
            )  
              
            if self._is_within_angle_range(target_rear_transform, ego_front_transform, max_distance, low_angle_th, up_angle_th):  
                distance = self._compute_distance(target_transform.location, ego_transform.location)  
                return (True, target_vehicle, distance)  
          
        return (False, None, -1)  
      
    def _is_within_angle_range(self, target_transform, ego_transform, max_distance, low_angle_th, up_angle_th):  
        """Check if target is within the specified angle range."""  
        # Simplified angle check - you may want to implement the full logic from BasicAgent  
        distance = target_transform.location.distance(ego_transform.location)  
        return distance <= max_distance  
      
    def _compute_distance(self, target_location, ego_location):  
        """Compute distance between two locations."""  
        return target_location.distance(ego_location)  
      
    def _calculate_relative_data(self, blocking_vehicle):  
        """Calculate relative position and velocity."""  
        veh_transform = self._parent.get_transform()  
        obs_pos = blocking_vehicle.get_location()  
          
        # Calculate relative position (simplified - you may need transform_to_world function)  
        rel_x = obs_pos.x - veh_transform.location.x  
        rel_y = obs_pos.y - veh_transform.location.y  
        rel_z = obs_pos.z - veh_transform.location.z  
        self.rel_pos = [abs(rel_x), abs(rel_y), abs(rel_z)]  
          
        # Calculate relative velocity  
        actor_vel = blocking_vehicle.get_velocity()  
        ego_vel = self._parent.get_velocity()  
        rel_vel_x = actor_vel.x - ego_vel.x  
        rel_vel_y = actor_vel.y - ego_vel.y  
        rel_vel_z = actor_vel.z - ego_vel.z  
        self.rel_vel = [rel_vel_x, rel_vel_y, rel_vel_z]  
        print(self.rel_vel)
      
    def destroy(self):  
        """Cleanup - no sensors to destroy."""  
        pass


# Obstacle sensor (ultrasonic)
class ObstacleSensor(object):
    def __init__(self, parent_actor, hud, debug=True, listen=True, 
                 x=2.5, y=0.0, z=1.0, yaw=0.0, hit_radius='1.5', sensor_tick='0.3'):
        self._parent = parent_actor
        world = self._parent.get_world()
        obs_sensor_bp = world.get_blueprint_library().find('sensor.other.obstacle')
        obs_sensor_bp.set_attribute('distance', '25')
        obs_sensor_bp.set_attribute('sensor_tick', sensor_tick)
        obs_sensor_bp.set_attribute('hit_radius', hit_radius)

        self.location = carla.Location(x=x, y=y, z=z)
        self.rotation = carla.Rotation(yaw=yaw)
        self.transform = carla.Transform(location=self.location, rotation=self.rotation)
        self.sensor = world.try_spawn_actor(obs_sensor_bp, self.transform,
                                            attach_to=self._parent)

        self.obstacle = None
        self.distance_to_obstacle = None
        self.close_to_obstacle = False

        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        if listen:
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: ObstacleSensor._on_detect(weak_self, event))

    @staticmethod
    def _on_detect(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.obstacle = event.other_actor
        self.distance_to_obstacle = event.distance
