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
                'max_distance': 15.0,  
                'up_angle_th': 90,  
                'low_angle_th': 0,  
                'lane_offset': 0  
            }  
        elif radar_type == 'left_front_radar':  
            return {  
                'max_distance': 10.0,  
                'up_angle_th': 120,  
                'low_angle_th': 30,  
                'lane_offset': -1  # Left lane  
            }  
        elif radar_type == 'left_back_radar':  
            return {  
                'max_distance': 8.0,  
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
        self.rel_pos = [rel_x, rel_y, rel_z]  
          
        # Calculate relative velocity  
        actor_vel = blocking_vehicle.get_velocity()  
        ego_vel = self._parent.get_velocity()  
        rel_vel_x = actor_vel.x - ego_vel.x  
        rel_vel_y = actor_vel.y - ego_vel.y  
        rel_vel_z = actor_vel.z - ego_vel.z  
        self.rel_vel = [rel_vel_x, rel_vel_y, rel_vel_z]  
      
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


# Radar
class RadarSensor(object):
    def __init__(self, parent_actor, hud, debug=True,
                 rr='25', hf='30', pps='400', x=2.5, y=0.0, z=1.0, yaw=0.0):
        self._parent = parent_actor
        world = self._parent.get_world()
        radar_sensor_bp = world.get_blueprint_library().find('sensor.other.radar')
        radar_sensor_bp.set_attribute('range', rr)
        radar_sensor_bp.set_attribute('horizontal_fov', hf)
        radar_sensor_bp.set_attribute('points_per_second', pps)
        
        self.location = carla.Location(x=x, y=y, z=z)
        self.rotation = carla.Rotation(yaw=yaw)
        self.transform = carla.Transform(location=self.location, rotation=self.rotation)
        self.sensor = world.try_spawn_actor(radar_sensor_bp, self.transform,
                                            attach_to=self._parent)
        self.detected = False
        self.rel_pos = None
        self.rel_vel = None
        self._velocity_range = 15
        self._debug = debug
        self._draw = world.debug

        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: RadarSensor._on_detect(weak_self, event, debug))

    @staticmethod
    def clamp(min_v, max_v, value):
        return max(min_v, min(value, max_v))
    
    @staticmethod
    def _on_detect(weak_self, event, debug=False):
        self = weak_self()
        if not self:
            return
        
        if debug:
            # Get radar transform
            radar_pos = event.transform.location

        # Get vehicle transform
        vv = self._parent.get_velocity()
        veh_vel = carla.Vector3D(x=vv.x, y=vv.y, z=vv.z)
        v2w_pos_transform = self._parent.get_transform()

        # Ignore points with radar velocity caused by moving vehicles
        # Which are basically static points
        ## TODO This method still cannot avoid all the static points ###
        vel_transform = carla.Transform(carla.Location(), self.rotation)
        moving_vel = transform_to_world(vel_transform, veh_vel, inverse=True)
        self.detected = False

        pos_list = []
        vel_list = []
        # For each point, get its x,y,z in radar coordinate
        for detect in event:
            # Reject points with altitude higher than 20 degree
            if detect.altitude > 0.25 or detect.altitude < 0:
                continue
            # Calculate actual velocity of the point
            act_vel = detect.velocity + moving_vel.x
            # Reject static point
            if abs(act_vel) < 3:
                continue

            # Get point transform
            sphi = math.sin(detect.azimuth)
            cphi = math.cos(detect.azimuth)
            stheta = math.sin(1.5707-detect.altitude)
            ctheta = math.cos(1.5707-detect.altitude)
            x = detect.depth * stheta * cphi
            y = detect.depth * stheta * sphi
            z = detect.depth * ctheta
            pos_list.append([x, y, z])
            vel_list.append(act_vel)
        if not pos_list:
            return

        # Transform to vehicle coordinate
        pos_m = np.transpose(np.array(pos_list))
        rel_pos = transform_to_frame(self.transform, pos_m, inverse=True)
        rel_vel = np.array(vel_list)

        # Draw point
        if self._debug:
            p2w_transform = carla.Transform(radar_pos, self.rotation)
            draw_m = transform_to_frame(p2w_transform, pos_m, inverse=True)
            draw_m = np.transpose(draw_m)
            for i, point_world in enumerate(draw_m):
                draw_vec = carla.Vector3D(x=point_world[0], y=point_world[1], z=point_world[2])
                norm_velocity = vel_list[i] / self._velocity_range # range [-1, 1]
                r = int(self.clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                g = int(self.clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                b = int(abs(self.clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                self._draw.draw_point(draw_vec, size=0.075, 
                    life_time=0.06,persistent_lines=False, color=carla.Color(r, g, b))
        
        # Store detected object
        self.rel_pos = np.mean(rel_pos, axis=1)
        self.rel_vel = np.mean(rel_vel)
        self.detected = True


# CollisionSensor
class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        existing_collision = self._parent.get_world().get_actors().filter('sensor.other.collision')  
        attached_collision = [s for s in existing_collision if s.parent and s.parent.id == self._parent.id]  
        self.sensor = attached_collision[0] if attached_collision else None  
        
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        name = ' '.join(event.other_actor.type_id.replace('_', '.').title().split('.')[1:])
        truncate = 250
        actor_type = (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# GnssSensor
class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        existing_gnss = self._parent.get_world().get_actors().filter('sensor.other.gnss')   
        attached_gnss = [s for s in existing_gnss if s.parent and s.parent.id == self._parent.id]  
        self.sensor = attached_gnss[0] if attached_gnss else None
        
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# Camera Set including Driver's View & Rearview Mirror
class CameraSet(object):
    def __init__(self, parent_actor, hud):
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False

        # Define three rgb cameras
        self._camera_transforms = [
            carla.Transform(carla.Location(x=0.1, y=-0.3, z=1.3)),
            carla.Transform(carla.Location(x=0.6, y=-1.0, z=1), carla.Rotation(yaw=-150)),
            carla.Transform(carla.Location(x=0.6, y=1.0, z=1), carla.Rotation(yaw=150))]
        self._transform_index = 0
        self._transform_list = [
            carla.Transform(carla.Location(x=0.1, y=-0.3, z=1.3)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))]
        self._sensors_param = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Main'],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Left_rearview'],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Right_rearview']]
        self.left_rearview_image = None
        self.right_rearview_image = None

        # Append blueprint in the end of each sensor
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self._sensors_param:
            bp = bp_library.find(item[0])
            if 'Main' in item[2]:
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            if "rearview" in item[2]:
                bp.set_attribute('image_size_x', str(int(hud.dim[0]*1/4)))
                bp.set_attribute('image_size_y', str(int(hud.dim[1]*1/4)))
                bp.set_attribute('fov', str(70))
            item.append(bp)

        # Spawn cameras
        existing_cameras = self._parent.get_world().get_actors().filter('sensor.camera.rgb')  
        attached_cameras = [s for s in existing_cameras if s.parent and s.parent.id == self._parent.id]  
        self._sensor_list = attached_cameras[:3] if len(attached_cameras) >= 3 else []  
        
        self.sensor1 = self._sensor_list[0]
        self.sensor2 = self._sensor_list[1]
        self.sensor3 = self._sensor_list[2]
        
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor1.listen(lambda event: CameraSet._parse_image(weak_self, event))
        self.sensor2.listen(lambda event: CameraSet._store_left_rearview(weak_self, event))
        # self.sensor3.listen(lambda event: CameraSet._store_right_rearview(weak_self, event))

    @staticmethod
    def _store_left_rearview(weak_self, image):
        self = weak_self()
        if not self:
            return

        image.convert(self._sensors_param[1][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        self.left_rearview_image = array

    @staticmethod
    def _store_right_rearview(weak_self, image):
        self = weak_self()
        if not self:
            return

        image.convert(self._sensors_param[2][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        self.right_rearview_image = array

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return

        image.convert(self._sensors_param[0][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        '''
        # Disabled lane_detection for not able to handle vehicle-in-front cases
        # and high computing resources required
        self.ld.lane_detection(array)
        array = self.ld.result_image
        '''

        # Attach left rearview mirror
        display_image = array.copy()
        if self.left_rearview_image is not None:
            display_image[:int(self.hud.dim[1]*1/4), :int(self.hud.dim[0]*1/4), :] = self.left_rearview_image
        '''
        if self.right_rearview_image is not None:
            display_image[self.hud.dim[0]-self.hud.dim[0]*1/4:self.hud.dim[0],\
                self.hud.dim[1]-self.hud.dim[1]*1/4:self.hud.dim[1], :]=\
                self.right_rearview_image
        '''

        self.surface = pygame.surfarray.make_surface(display_image.swapaxes(0, 1))

        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    def toggle_camera(self):
        self._transform_index = (self._transform_index + 1) % len(self._transform_list)
        self.sensor1.set_transform(self._transform_list[self._transform_index])

    def next_sensor(self):
        pass

    def destroy(self):
        for sensor in self._sensor_list:
            if sensor is not None:
                sensor.destroy()


# CameraManager
class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=0.5, z=1.5)),
            carla.Transform(carla.Location(x=0.0, z=2.0)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))]
        self.transform_index = 0
        # Define sensor list
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        # Append blueprint in the end of each sensor
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('channels', '8')
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None
        # For the main rgb camera
        self.process_rate = 10
        self.counter = 10
        self.curvature = None
        self.offset = None
        self.ld = LaneDetection()

    def set_sensor(self, index, notify=True, display_camera=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None \
            else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            if display_camera:
                self.sensor.listen(lambda img: CameraManager._parse_image(weak_self, img))
            else:
                self.sensor.listen(lambda img: CameraManager._process_image(weak_self, img))

        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    # Change camera position
    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self.transform_index])

    # Change camera
    def next_sensor(self):
        self.set_sensor(self.index + 1, display_camera=True)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    # Do nothing
    @staticmethod
    def _process_image(weak_self, image):
        self = weak_self()
        if not self:
            return

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.camera'):
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]

            '''
            # Disabled lane_detection for not able to handle vehicle-in-front cases
            # and high computing resources required
            self.ld.lane_detection(array)
            array = self.ld.result_image
            '''
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        else:
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)
