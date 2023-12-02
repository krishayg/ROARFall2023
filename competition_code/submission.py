"""
Competition instructions:
Please do not change anything else but fill out the to-do sections.
"""

from typing import List, Tuple, Dict, Optional
import roar_py_interface
import numpy as np
from collections import deque
import json
from pathlib import Path
import keyboard
import math
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
def find_k_values(current_speed,config) -> np.array:
        
        k_p, k_d, k_i = 1, 0, 0
        for speed_upper_bound, kvalues in config.items():
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
        return np.array([k_p, k_d, k_i])
def distance(a,b):
    sum=0
    #print(b,a)
    for i in range(3):
        sum+=(eval(b[i])-float(a[i]))**2
    return math.sqrt(sum)
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
        self.config = json.load(Path("config.json").open(mode='r'))['latitudinal_controller']

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
        self.waypoint_queue_braking = []
        with open("braking_list_new.txt") as f:
            for line in f:
                raw = line.split(" ")
                waypoint = raw[0:4]
                print(waypoint)
                self.waypoint_queue_braking.append(waypoint)
        self.brake_ctr=0
    async def step(
        self
    ) -> None:
        """
        This function is called every world step.
        Note: You should not call receive_observation() on any sensor here, instead use get_last_observation() to get the last received observation.
        You can do whatever you want here, including apply_action() to the vehicle.
        """
        # TODO: Implement your solution here.

        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()
        vehicle_velocity_norm = np.linalg.norm(vehicle_velocity)
        
        # Find the waypoint closest to the vehicle
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )
         # We use the 3rd waypoint ahead of the current waypoint as the target waypoint
        print(self.current_waypoint_idx)
        waypoint_to_follow = self.maneuverable_waypoints[(self.current_waypoint_idx + 3) % len(self.maneuverable_waypoints)]
        dist=distance(waypoint_to_follow.location,self.waypoint_queue_braking[0])
        print(dist)
        print(self.waypoint_queue_braking[0])
        if distance(waypoint_to_follow.location,self.waypoint_queue_braking[0])<=7:
            print("HERE------------------------------------------------------------------------------------------------------------------------------------]")
            self.brake_ctr=int(self.waypoint_queue_braking[0][3])
            self.waypoint_queue_braking.pop(0)

        # if keyboard.is_pressed("space"):
        #     print(waypoint_to_follow)        # Receive location, rotation and velocity data 
        lookahead_waypoint=self.maneuverable_waypoints[(self.current_waypoint_idx + 75) % len(self.maneuverable_waypoints)]
        # Calculate delta vector towards the target waypoint
        vector_to_waypoint = (waypoint_to_follow.location - vehicle_location)[:2]
        heading_to_waypoint = np.arctan2(vector_to_waypoint[1],vector_to_waypoint[0])

        vector_to_lookaheadwaypoint = (lookahead_waypoint.location - vehicle_location)[:2]
        heading_to_lookaheadwaypoint = np.arctan2(vector_to_lookaheadwaypoint[1],vector_to_lookaheadwaypoint[0])
        # Calculate delta angle towards the target waypoint
        delta_heading = normalize_rad(heading_to_waypoint - vehicle_rotation[2])
        delta_headinglookahead=abs(normalize_rad(heading_to_lookaheadwaypoint-vehicle_rotation[2]))
        #lookaheaderror=(
        #     -8.0 * np.sqrt(vehicle_velocity_norm) * delta_headinglookahead / np.pi
        # )
        #print("laerror"+str(delta_headinglookahead))
        # Proportional controller to steer the vehicle towards the target waypoint
        error = (
            -8.0 / np.sqrt(vehicle_velocity_norm) * delta_heading / np.pi
        ) if vehicle_velocity_norm > 1e-2 else -np.sign(delta_heading)
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0


        #k_p=-0.042+(0.4)/(1+(vehicle_velocity_norm/160)**4.9)
        k_p=0.3; k_d=0.4
        k_d=-0.016+(0.64)/(1+(vehicle_velocity_norm/115)**3)
        k_i=0.1
        k_p,k_d,k_i=find_k_values(vehicle_velocity_norm,self.config)
        print(k_p,k_d,k_i,vehicle_velocity_norm)
        steer_control=0.8*(error*k_p+_de*k_d+_ie*k_i)
        steer_control = np.clip(steer_control, -1.0, 1.0)

        # Proportional controller to control the vehicle's speed towards 40 m/s
        wpnumber=self.current_waypoint_idx%2775
        if wpnumber<670 or wpnumber>1900:

            if delta_headinglookahead>0.5 and vehicle_velocity_norm>20:
                throttle_control=-0.25
            elif vehicle_velocity_norm>43:
                throttle_control=0.7
            elif delta_headinglookahead>0.1 and vehicle_velocity_norm>30:
                throttle_control=0.8
            else:
                throttle_control=1
        elif wpnumber>670 and wpnumber<800:
            throttle_control=1
        else:
            if delta_headinglookahead>0.6 and vehicle_velocity_norm>28:
                throttle_control=-0.25
            elif vehicle_velocity_norm>48:
                throttle_control=0.75
            elif delta_headinglookahead>0.1 and vehicle_velocity_norm>30:
                throttle_control=0.85
            else:
                throttle_control=1
        #throttle_control = 0.05 * (30 - vehicle_velocity_norm)
        #throttle_control = 0.05 * (25 - vehicle_velocity_norm)
        if self.brake_ctr>0:
            throttle_control=-1
            self.brake_ctr-=1
           
        print(throttle_control)
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
