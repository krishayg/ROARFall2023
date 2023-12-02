"""
Competition instructions:
Please do not change anything else but fill out the to-do sections.
"""

from typing import List, Tuple, Dict, Optional
import roar_py_interface
import numpy as np
from collections import deque


def normalize_rad(rad : float):
    return (rad + np.pi) % (2 * np.pi) - np.pi

def filter_waypoints(location : np.ndarray, current_idx: int, waypoints : List[roar_py_interface.RoarPyWaypoint]) -> int:
    def dist_to_waypoint(waypoint : roar_py_interface.RoarPyWaypoint):
        return np.linalg.norm(
            location[:2] - waypoint.location[:2]
        )
    for i in range(current_idx, len(waypoints) + current_idx):
        if dist_to_waypoint(waypoints[i%len(waypoints)]) < 3:
            return i % len(waypoints)
    return current_idx

class RoarCompetitionSolution:
    def __init__(
        self,
        maneuverable_waypoints: List[roar_py_interface.RoarPyWaypoint],
        vehicle : roar_py_interface.RoarPyActor,
        camera_sensor : roar_py_interface.RoarPyCameraSensor = None,
        location_sensor : roar_py_interface.RoarPyLocationInWorldSensor = None,
        velocity_sensor : roar_py_interface.RoarPyVelocimeterSensor = None,
        rpy_sensor : roar_py_interface.RoarPyRollPitchYawSensor = None,
        occupancy_map_sensor : roar_py_interface.RoarPyOccupancyMapSensor = None,
        collision_sensor : roar_py_interface.RoarPyCollisionSensor = None,
    ) -> None:
        self.maneuverable_waypoints = maneuverable_waypoints
        self.vehicle = vehicle
        self.camera_sensor = camera_sensor
        self.location_sensor = location_sensor
        self.velocity_sensor = velocity_sensor
        self.rpy_sensor = rpy_sensor
        self.occupancy_map_sensor = occupancy_map_sensor
        self.collision_sensor = collision_sensor
        self._dt=0.03
        self._error_buffer = deque(maxlen=10)

    
    async def initialize(self) -> None:
        # TODO: You can do some initial computation here if you want to.
        # For example, you can compute the path to the first waypoint.

        # Receive location, rotation and velocity data 
        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()
        
        self.current_waypoint_idx = 10
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )


    async def step(
        self
    ) -> None:
        """
        This function is called every world step.
        Note: You should not call receive_observation() on any sensor here, instead use get_last_observation() to get the last received observation.
        You can do whatever you want here, including apply_action() to the vehicle.
        """
        # TODO: Implement your solution here.

        # Receive location, rotation and velocity data 
        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()
        vehicle_velocity_norm = np.linalg.norm(vehicle_velocity)
        print("vehicle"+str(vehicle_rotation))
        # direction_vector = np.array([-np.sin(np.deg2rad(vehicle_rotation[2])),
        #                              0,
        #                              -np.cos(np.deg2rad(vehicle_rotation[2]))])
        direction_vector = np.array([-np.sin(vehicle_rotation[2]),
                                     0,
                                     -np.cos(vehicle_rotation[2])])        
        v_end = vehicle_location + direction_vector
        v_vec = np.array([(v_end[0] - vehicle_location[0]), 0, (v_end[2] - vehicle_location[2])])


        # Find the waypoint closest to the vehicle
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )
         # We use the 3rd waypoint ahead of the current waypoint as the target waypoint
        next_waypoint = self.maneuverable_waypoints[(self.current_waypoint_idx + 3) % len(self.maneuverable_waypoints)]
        #close_waypoint = self.maneuverable_waypoints[(self.current_waypoint_idx + 3) % len(self.maneuverable_waypoints)]
        # Calculate delta vector towards the target waypoint

        # calculate error projection
        w_vec = np.array(
            [
                next_waypoint.location[0] - vehicle_location[0],
                0,
                next_waypoint.location[2] - vehicle_location[2],
            ]
        )
        print(w_vec)
        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #error = np.arccos(v_vec_normed @ w_vec_normed.T)
        error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        print("error:"+str(error))
        _cross = np.cross(v_vec_normed, w_vec_normed)
        if _cross[1]>0:
            error*=-1
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0


        # vector_to_waypoint = (waypoint_to_follow.location - vehicle_location)[:2]
        # heading_to_waypoint = np.arctan2(vector_to_waypoint[1],vector_to_waypoint[0])

        # # Calculate delta angle towards the target waypoint
        # delta_heading = normalize_rad(heading_to_waypoint - vehicle_rotation[2])

        # # Proportional controller to steer the vehicle towards the target waypoint
        #print(error)
        steer_control = 0.3*(0.6*error+_de*0.3+_ie*0.1)
        print(error)
        steer_control = np.clip(steer_control, -1.0, 1.0)

        # Proportional controller to control the vehicle's speed towards 40 m/s
        throttle_control = 0.05 * (20 - vehicle_velocity_norm)
        control = {
            "throttle": np.clip(throttle_control, 0.0, 1.0),
            "steer": steer_control,
            "brake": np.clip(-throttle_control, 0.0, 1.0),
            "hand_brake": 0.0,
            "reverse": 0,
            "target_gear": 0
        }
        await self.vehicle.apply_action(control)
        return control
