#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Obstacle avoidance without prior action.
The ego-vehicle encounters an obstacle / unexpected entity on the road and 
must perform an emergency brake or an avoidance maneuver.
"""

import random
import py_trees
import carla
import operator

from agents.navigation.global_route_planner import GlobalRoutePlanner

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      StopVehicle,
                                                                      LaneChange,
                                                                      ActorDestroy,
                                                                      WaypointFollower,
                                                                      AccelerateToCatchUp,
                                                                      ChangeActorTargetSpeed,
                                                                      KeepVelocity
                                                                      )
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle, InTriggerDistanceToNextIntersection, DriveDistance,StandStill
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class custom_6(BasicScenario):

    """
    The ego vehicle is driving on a highway and another car is cutting in just in front.
    This is a single ego vehicle scenario
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):

        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        self._velocity = 35
        self._delta_velocity = 10
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 10
        self._other_actor_stop_in_front_intersection = 10
        point = config.trigger_points[0].location
        self._grp = GlobalRoutePlanner(CarlaDataProvider.get_map(), 2.0)
        super(custom_6, self).__init__("custom_6",
                                       ego_vehicles,
                                       config,
                                       world,
                                       debug_mode,
                                       criteria_enable=criteria_enable)
        self._sensor_list = []
        self._attach_camera_to_ego()

    def _attach_camera_to_ego(self):
        import os
        
        ego_vehicle = CarlaDataProvider.get_hero_actor()
        if ego_vehicle is None:
            raise ValueError("Ego vehicle with role_name 'hero' not found")

        world = CarlaDataProvider.get_world()
        bp_lib = world.get_blueprint_library()

        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '120')

        camera_transforms = {
            'front': carla.Transform(carla.Location(x=1.5, z=2.4), carla.Rotation(pitch=0)),
            'left': carla.Transform(carla.Location(x=0.0, y=-0.8, z=2.2), carla.Rotation(yaw=-90)),
            'right': carla.Transform(carla.Location(x=0.0, y=0.8, z=2.2), carla.Rotation(yaw=90)),
            'back': carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180))
        }

        for view in camera_transforms.keys():
            os.makedirs(f"_out/{view}", exist_ok=True)

        for view, transform in camera_transforms.items():
            camera = world.spawn_actor(camera_bp, transform, attach_to=ego_vehicle)
            camera.listen(lambda image, view=view: image.save_to_disk(f"_out/{view}/ego_{ego_vehicle.id}_%06d.png" % image.frame))
            self._sensor_list.append(camera)

    def _initialize_actors(self, config):
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=True)
   
    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """
        # car_visible
          # let the other actor drive until next intersection
        TriggerDist = py_trees.composites.Parallel(
            "DriveStraight",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        TriggerDist.add_child(InTriggerDistanceToVehicle(self.other_actors[0], self.ego_vehicles[0], distance=25))
        DriveStraight = py_trees.composites.Parallel(
            "DriveStraight",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        DriveStraight.add_child(KeepVelocity(self.other_actors[0], 1))
        DriveStraight.add_child(InTriggerDistanceToVehicle(self.other_actors[0], self.ego_vehicles[0], 40, comparison_operator=operator.gt))

        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(TriggerDist)
        # sequence.add_child(end)
        sequence.add_child(DriveStraight)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []
        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)
        return criteria

    def _create_construction_setup(self, start_transform, lane_width):
        """
        Create Construction Setup
        """
        _initial_offset = {'cones': {'yaw': 180, 'k': lane_width / 2.0},
                           'warning_sign': {'yaw': 180, 'k': 5, 'z': 0},
                           'debris': {'yaw': 0, 'k': 2, 'z': 1}}
        _prop_names = {'warning_sign': 'static.prop.trafficwarning',
                       'debris': 'static.prop.dirtdebris02'}

        _perp_angle = 90
        _setup = {'lengths': [0, 6, 3], 'offsets': [0, 2, 1]}
        _z_increment = 0.1

        # Traffic Warning and Debris
        for key, value in _initial_offset.items():
            if key == 'cones':
                continue
            transform = carla.Transform(
                start_transform.location,
                start_transform.rotation)
            transform.rotation.yaw += value['yaw']
            transform.location += value['k'] * \
                transform.rotation.get_forward_vector()
            transform.location.z += value['z']
            transform.rotation.yaw += _perp_angle
            static = CarlaDataProvider.request_new_actor(
                _prop_names[key], transform)
            static.set_simulate_physics(True)
            self.other_actors.append(static)

        # Cones
        side_transform = carla.Transform(
            start_transform.location,
            start_transform.rotation)
        side_transform.rotation.yaw += _perp_angle
        side_transform.location -= _initial_offset['cones']['k'] * \
            side_transform.rotation.get_forward_vector()
        side_transform.rotation.yaw += _initial_offset['cones']['yaw']

        for i in range(len(_setup['lengths'])):
            self.create_cones_side(
                side_transform,
                forward_vector=side_transform.rotation.get_forward_vector(),
                z_inc=_z_increment,
                cone_length=_setup['lengths'][i],
                cone_offset=_setup['offsets'][i])
            side_transform.location += side_transform.get_forward_vector() * \
                _setup['lengths'][i] * _setup['offsets'][i]
            side_transform.rotation.yaw += _perp_angle

    def create_cones_side(
            self,
            start_transform,
            forward_vector,
            z_inc=0,
            cone_length=0,
            cone_offset=0):
        """
        Creates One Side of the Cones
        """
        _dist = 0
        while _dist < (cone_length * cone_offset):
            # Move forward
            _dist += cone_offset
            forward_dist = carla.Vector3D(0, 0, 0) + forward_vector * _dist

            location = start_transform.location + forward_dist
            location.z += z_inc
            transform = carla.Transform(location, start_transform.rotation)

            cone = CarlaDataProvider.request_new_actor(
                'static.prop.constructioncone', transform)
            cone.set_simulate_physics(True)
            self.other_actors.append(cone)

    def __del__(self):
        """
        Remove all actors after deletion.
        """
        for sensor in self._sensor_list:
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()
        self._sensor_list = []
        self.remove_all_actors()
