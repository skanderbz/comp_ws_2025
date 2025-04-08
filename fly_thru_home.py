#!/usr/bin/env python3

#Dependencies
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition, SensorGps
import math
import time

#QADT functions 
import planeFunctions as pf
from planeFunctions import FlightState



def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    plane = pf.FixedWingControl()
    plane.cruise_altitude = 30

    plane.create_timer(0.1,lambda: task1(plane))
    rclpy.spin(plane)
    plane.destroy_node
    rclpy.shutdown()
    

#FLIGHT STATES = [INIT, ARMING, TAKEOFF, OFFBOARD_IDLE, LANDING]

def task1(self):
    if not self.local_position:
        return
    if not self.armed:
        self.arm_vehicle()
    if self.FlightState == FlightState.ARMING or self.FlightState == FlightState.TAKEOFF:
        self.full_takeoff()
    if self.current_nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        self.publish_offboard_heartbeat()
        print("Switched to offboard")
        self.publish_waypoint_setpoint(0,0,self.cruise_altitude)
        
    elif self.current_nav_state == VehicleStatus.NAVIGATION_STATE_MANUAL:
        print("Gooning to manual")
    # else:
    #     print("Gooning to not manual or offboard")
    print(self.local_position.z)
    print(self.current_nav_state)



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)