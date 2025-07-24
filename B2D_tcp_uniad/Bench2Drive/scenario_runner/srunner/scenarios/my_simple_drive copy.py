#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) [2025] [Jiwon Moon]
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import print_function

import py_trees

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle, StopVehicle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario

def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]['value'])
    else:
        return default

class MySimpleDrive(BasicScenario):
    """
    A scenario where the ego vehicle drives straight at a low speed with no surrounding traffic.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self.timeout = timeout
        self._drive_distance = get_value_parameter(config, 'drive_distance', float, 50) # Distance to drive before ending (meters)

        super().__init__("MySimpleDrive",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _create_behavior(self):
        print(f"Drive distance set to: {self._drive_distance}")  # 디버깅 출력 추가
        sequence = py_trees.composites.Sequence(name="MySimpleDrive")        
        sequence.add_child(DriveDistance(self.ego_vehicles[0], self._drive_distance))        
        sequence.add_child(Idle(10))
        sequence.add_child(StopVehicle(self.ego_vehicles[0], 0.0))  # 차량 정지
        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        for ego_vehicle in self.ego_vehicles:
            collision_criterion = CollisionTest(ego_vehicle)
            criteria.append(collision_criterion)

        timeout_criterion = ScenarioTimeoutTest(self.ego_vehicles[0], self.config, self.timeout)
        criteria.append(timeout_criterion)

        return criteria

    def __del__(self):
        """
        Cleanup actors (none in this case).
        """
        self.remove_all_actors()