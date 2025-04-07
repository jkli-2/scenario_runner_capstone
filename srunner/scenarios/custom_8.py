#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Slow moving hazard at lane edge.
The ego-vehicle encounters a slow moving hazard blocking part of the lane. 
The ego-vehicle must brake or maneuver to avoid it next to a lane of traffic moving in the opposite direction.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      StopVehicle,
                                                                      LaneChange,
                                                                      ActorDestroy,
                                                                      WaypointFollower,
                                                                      AccelerateToCatchUp,
                                                                      ChangeActorTargetSpeed
                                                                       
                                                                      )
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                                InTriggerDistanceToNextIntersection,
                                                                                RelativeVelocityToOtherActor,
                                                                                  DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class custom_8(BasicScenario):

    """
    The ego vehicle is driving on a highway and another car is cutting in just in front.
    This is a single ego vehicle scenario
    """

    timeout = 12000

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        self._velocity = 35
        self._delta_velocity = 10
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 10
        self._other_actor_stop_in_front_intersection = 10
        point = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(point)
        super(custom_8, self).__init__("custom_8",
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
        camera_bp.set_attribute('sensor_tick', '0.1') # 20Hz

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
        waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        
        waypoint_left = waypoint.get_left_lane()
        transform = waypoint_left.transform
        transform.location.z += 0.5
        transform.location.x += 20
        tranform_ped = waypoint.transform
        tranform_ped.location.z += 0.5
        tranform_ped.location.x -= 5
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', transform)

        self.other_actors.append(first_vehicle)
        self.adversary = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', tranform_ped)
        self.other_actors.append(self.adversary)

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
        DriveStraight = py_trees.composites.Parallel(
            "DriveStraight",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        DriveStraight.add_child(WaypointFollower(self.other_actors[0], 15))
        DriveStraight.add_child(WaypointFollower(self.other_actors[1], 2))
        DriveStraight.add_child(InTriggerDistanceToNextIntersection(
            self.other_actors[0], self._other_actor_stop_in_front_intersection))
        DriveStraight.add_child(RelativeVelocityToOtherActor(self.ego_vehicles[0],
            self.adversary, 20))
        #DriveStraight.add_child(ChangeActorTargetSpeed(self.other_actors[0],20))
        
        # stop vehicle
        # stop = StopVehicle(self.other_actors[0], 1.0)

        # end condition
        end = py_trees.composites.Parallel("ego reached intersection",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        end.add_child(InTriggerDistanceToNextIntersection(
            self.ego_vehicles[0], 1))
        # end condition
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(DriveStraight)
        sequence.add_child(end)
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
