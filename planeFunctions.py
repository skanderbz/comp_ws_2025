#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition, SensorGps
import math
import time
# import pyproj

class FixedWingTakeoffControl(Node):
    def __init__(self):
        super().__init__('fixed_wing_takeoff_node')

        # Configure QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile)
        self.gps_position_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.gps_callback, qos_profile)

        # Timer for publishing control commands
        self.create_timer(0.1, self.timer_callback)  # 10Hz

        # Flight parameters
        self.cruise_altitude = 50.0  # meters - final cruise altitude
        self.idle_setpoint_distance = 100.0  # meters - distance of idle circle center from takeoff position
        self.circle_radius = 50.0  # meters - radius of the circle to fly
        
        # State variables
        self.vehicle_status = None
        self.local_position = None
        self.armed = False
        self.current_nav_state = None
        self.flight_phase = 'INIT'  # States: INIT, ARMING, TAKEOFF, OFFBOARD_IDLE
        self.start_position = None
        self.current_position = None
        self.takeoff_time = None
        self.transition_to_offboard = False
        self.offboard_setpoint_sent = False
        self.circle_angle = 0.0  # For generating circular flight pattern
        self.rotations = 0
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        self.current_tgt = 0
        self.targets = [
            [-123.45, 234.56, 50.0],
            [89.01, -456.78, 50.0],
            [210.32, 98.76, 50.0],
            [-50.67, -321.45, 50.0],
            [150.89, 432.10, 50.0]
        ]

    def vehicle_status_callback(self, msg):
        """Monitor vehicle status for arming and flight modes"""
        self.vehicle_status = msg
        self.current_nav_state = msg.nav_state
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        
        # Log current navigation state for debugging
        if self.current_nav_state != getattr(self, 'prev_nav_state', None):
            self.get_logger().info(f'Navigation state changed to: {self.current_nav_state}')
            self.prev_nav_state = self.current_nav_state

    def local_position_callback(self, msg):
        """Store local position information"""
        self.local_position = msg
        if self.start_position is None and msg.xy_valid:
            self.start_position = [msg.x, msg.y, msg.z]
            self.get_logger().info(f"Start position recorded: X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}")
        self.current_position = [msg.x, msg.y, msg.z]

    def gps_callback(self, msg):
        if self.origin_lat is None and self.origin_lon is None and self.origin_alt is None:
            self.origin_lat = msg.latitude_deg
            self.origin_lon = msg.longitude_deg
            self.origin_alt = msg.altitude_msl_m
            self.get_logger().info(f"Set GPS Origin: {self.origin_lat}, {self.origin_lon}, {self.origin_alt}")


    def timer_callback(self):
        """Execute the flight sequence based on current phase"""
        # Ensure we have position data before proceeding
        if not self.local_position:
            return

        # State machine for flight phases
        if self.flight_phase == 'INIT' and self.local_position.xy_valid:
            self.flight_phase = 'ARMING'
            self.get_logger().info('Initializing: Proceeding to arming phase')
        
        elif self.flight_phase == 'ARMING':
            if not self.armed:
                self.arm_vehicle()
            else:
                self.flight_phase = 'TAKEOFF'
                self.takeoff_time = self.get_clock().now()
                self.get_logger().info('Armed: Proceeding to takeoff phase')
                self.initiate_takeoff()
        
        elif self.flight_phase == 'TAKEOFF':
            # Check if we've reached minimum altitude for transition
            current_altitude = -self.local_position.z  # Convert from NED to altitude
            
            # We'll transition to offboard after takeoff mode has had time to get the aircraft in the air
            # and we've reached a reasonable altitude
            if (current_altitude > 30.0 and 
                (self.current_nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF)):
                self.get_logger().info(f"Current altitude: {current_altitude:.2f}m - Ready for offboard transition")
                self.transition_to_offboard = True
                
            # Prepare for offboard mode by sending setpoints before switching
            if self.transition_to_offboard:
                self.publish_offboard_control_mode()
                self.publish_circle_setpoint()
                
                # After sending setpoints for a second, switch to offboard
                current_time = self.get_clock().now()
                if not self.offboard_setpoint_sent:
                    self.offboard_setpoint_sent = True
                    self.setpoint_start_time = current_time
                    self.get_logger().info("Starting to send offboard setpoints")
                
                # Check if we've been sending setpoints for at least 1 second
                if (self.offboard_setpoint_sent and 
                    (current_time - self.setpoint_start_time).nanoseconds / 1e9 >= 1.0):
                    self.flight_phase = 'OFFBOARD_IDLE'
                    self.get_logger().info('Transitioning to OFFBOARD mode with circle pattern')
                    self.set_offboard_mode()
        
        elif self.flight_phase == 'OFFBOARD_IDLE':
            # In offboard mode, continuously publish control mode and position setpoints
            if self.current_nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info(f"Not in OFFBOARD mode (current: {self.current_nav_state}), re-sending command")
                self.set_offboard_mode()
            
            self.publish_offboard_control_mode()

            if self.rotations < 2:
                self.publish_circle_setpoint()
                
                # Update circle angle for next iteration
                self.circle_angle += 0.01
                if self.circle_angle > 2 * math.pi:
                    self.circle_angle -= 2 * math.pi
                    self.rotations += 1
                    self.get_logger().info(f'Rotations: {self.rotations}')
            else:
                at_lat = round(self.targets[self.current_tgt][0], 2) - 2 <= round(self.current_position[0], 2) \
                    and round(self.current_position[0], 2) <= round(self.targets[self.current_tgt][0], 2) + 2
                at_lon = round(self.targets[self.current_tgt][1], 2) - 2 <= round(self.current_position[1], 2) \
                    and round(self.current_position[1], 2) <= round(self.targets[self.current_tgt][1], 2) + 2
                at_alt = round(self.targets[self.current_tgt][2], 2) - 4 <= round(self.current_position[2], 2) \
                    and round(self.current_position[2], 2) <= round(self.targets[self.current_tgt][2], 2) + 4
                
                if not at_lat and not at_lon and not at_alt:
                    self.get_logger().info(f'Current pos {self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}')
                    self.get_logger().info(f'Flying to {self.targets[self.current_tgt]}')
                    self.publish_waypoint_setpoint(*self.targets[self.current_tgt])
                elif self.current_tgt < len(self.targets) - 1:   # Made it, go next
                    self.current_tgt += 1
                else:
                    self.get_logger().info('REACHED ALL TARGETS!!!')

    def arm_vehicle(self):
        """Send command to arm the vehicle"""
        self.get_logger().info("Sending arm command")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def initiate_takeoff(self):
        """Command the vehicle to enter takeoff mode"""
        self.get_logger().info("Commanding takeoff mode")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=0.0,   # Pitch (optional)
            param4=0.0,   # Yaw (optional)
            param5=self.local_position.ref_lat,  # Latitude
            param6=self.local_position.ref_lon,  # Longitude
            param7=self.cruise_altitude  # Altitude
        )

    def set_offboard_mode(self):
        """Command the vehicle to enter offboard mode"""
        self.get_logger().info("Setting OFFBOARD mode")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)


    def publish_offboard_control_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_circle_setpoint(self):
        """Publish position setpoints to make the aircraft fly in a circle"""
        if not self.start_position:
            return
            
        # Calculate position on circle
        x = 5 + self.start_position[0] + self.idle_setpoint_distance + self.circle_radius * math.cos(self.circle_angle)
        y = 5 + self.start_position[1] + self.circle_radius * math.sin(self.circle_angle)
        
        msg = TrajectorySetpoint()
        
        # Position setpoint
        msg.position = [
            x,  # X position on circle
            y,  # Y position on circle
            -self.cruise_altitude  # Z - maintain cruise altitude
        ]
        
        # We can also set velocity to help with the circle pattern
        # These are optional and could be omitted
        tangential_vel = 20.0  # m/s - adjust based on desired speed
        msg.velocity = [
            -tangential_vel * math.sin(self.circle_angle),  # vx
            tangential_vel * math.cos(self.circle_angle),   # vy
            0.0  # vz - maintain altitude
        ]
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_waypoint_setpoint(self, x, y, altitude):
        """Publish a single waypoint setpoint"""
        msg = TrajectorySetpoint()
        
        # Position setpoint
        msg.position = [
            x,  # X coordinate in local frame
            y,  # Y coordinate in local frame
            -altitude  # Z coordinate (negative for altitude in NED frame)
        ]

        # Explicitly set velocity to null but valid
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, 
                               param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """Publish vehicle commands"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)



    