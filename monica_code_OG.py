#!/usr/bin/env python3

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, SensorGps, ActuatorMotors, ActuatorServos, TrajectorySetpoint, VehicleThrustSetpoint, VehicleRatesSetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from geometry_msgs.msg import Twist

EARTH_RADIUS = 6_371_000 # m

# Turning rad  17
# min speed 14 
# cruising speed 22

class Mode(Enum):
    WAIT = auto()
    TAKEOFF = auto()
    FLY = auto()
    LAND = auto()

class Gooncopter(Node):
    """Node for controlling gooncopter in offboard mode."""

    def __init__(self) -> None:
        super().__init__('gooncopter')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.thrust_setpoint_publisher = self.create_publisher(
            VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile)
        self.rates_setpoint_publisher = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.actuator_motors_publisher = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.actuator_servos_publisher = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_gps_position_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_position_callback, qos_profile)

        # ================================================================== #
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.sensor_gps = SensorGps()
        # ================================================================== #
        
        self.start = True

        self.mode = Mode.WAIT

        self.takeoff_height = -30.0

        self.target_x = 10.0
        self.target_y = 600.0

        self.stopwatch = 0
 
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.01, self.timer_callback)
        # For the propeller to spin with ActuatorMotors and VehicleRatesSetpoint,
        # the timer interval needs to be super small, otherwise it doesn't spin fast enough
        # It is probably best not to use ActuatorMotors unless it is needed for something very specific.
        # It causes some very strange behaviour

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def vehicle_gps_position_callback(self, sensor_gps):
        """Callback function for sensor_gps topic subscriber."""
        self.sensor_gps = sensor_gps

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def engage_altitude_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=2.0)
        self.get_logger().info("Switching to altitude mode")

    def engage_mission_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=3.0, param3=2.0)
        self.get_logger().info("Switching to mission mode?")

    def engage_return_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=3.0, param3=4.0)
        self.get_logger().info("Switching to return mode?")

    def takeoff(self):
        """Switch to takeoff mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                                     param5=44.287,
                                     param6=-76.4242,
                                     param7=-self.takeoff_height+10)
        self.get_logger().info("Switching to takeoff mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND,
                                     param4=0.0,
                                     param5=self.home_lat,
                                     param6=self.home_lon)
        self.get_logger().info("Switching to land mode")

    def start_landing(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_LAND_START)
        self.get_logger().info("Starting landing.")

    def return_to_launch(self):
        """Return to launch."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("Returning to launch.")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_thrust_setpoint(self):
        msg = VehicleThrustSetpoint()
        msg.xyz = [1.0, 1.0, 1.0]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.thrust_setpoint_publisher.publish(msg)

    def publish_actuator_servos(self):
        msg = ActuatorServos()
        msg.control[0] = 0.3 # left aileron
        msg.control[1] = 0.3 # right aileron
        msg.control[2] = -1.0 # elevator
        msg.control[3] = 0.0 # rudder
        msg.control[4] = 0.3 # left flap
        msg.control[5] = 0.3 # right flap
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.actuator_servos_publisher.publish(msg)

    def publish_actuator_motors(self):
        msg = ActuatorMotors()
        msg.reversible_flags = 0
        msg.control[0] = 1.0 # Propeller
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.actuator_motors_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_velocity_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.velocity = [x, y, z]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing velocity setpoints {[x, y, z]}")

    def publish_rates_setpoint(self, pitch: float, yaw: float, roll: float, thrust: float):
        """Publish setpoints for pitch, yaw, roll, and thrust"""
        msg = VehicleRatesSetpoint()
        msg.pitch = pitch
        msg.yaw = yaw
        msg.roll = roll
        msg.thrust_body[0] = thrust
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.rates_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def convert_gps_to_xy(self, lat, lon):
        d_lat = (lat - self.home_lat) * math.pi/180
        d_lon = (lon - self.home_lon) * math.pi/180
        x = EARTH_RADIUS * d_lat
        y = EARTH_RADIUS * math.cos(self.home_lat * math.pi/180) * d_lon

        return round(x,6), round(y,6)

    def calc_velocity_vec(self):
        curr_x = self.vehicle_local_position.x
        curr_y = self.vehicle_local_position.y

        diff_x = self.target_x - curr_x
        diff_y = self.target_y - curr_y

        dist = math.sqrt(diff_x**2 + diff_y**2)

        return (self.speed/dist)*(diff_x), (self.speed/dist)*(diff_y)

    def initialize_flight(self):
        """Handle initial flight setup"""
        
        # Check if GPS is working before taking off
        if self.sensor_gps.latitude_deg == 0.0 and self.sensor_gps.longitude_deg == 0.0:
            self.get_logger().warn("=== GPS NOT WORKING ===")
            return  # Prevent takeoff if GPS is not working

        self.get_logger().info(f"Current GPS Position: Latitude: {self.sensor_gps.latitude_deg}, Longitude: {self.sensor_gps.longitude_deg}")

        # Arm
        self.arm()
        self.mode = Mode.TAKEOFF

        # Set home coordinates
        self.home_lat = self.sensor_gps.latitude_deg
        self.home_lon = self.sensor_gps.longitude_deg

        self.takeoff()

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.mode == Mode.WAIT:
            self.initialize_flight()
            return

        if self.mode == Mode.TAKEOFF:
            if self.vehicle_local_position.z < self.takeoff_height + 10:
                self.mode = Mode.FLY
                self.engage_altitude_mode()
                self.stopwatch = 0

        pos = [self.vehicle_local_position.x, self.vehicle_local_position.y]
        if reached_approx(pos, [0, 0], 15) and self.stopwatch >= 1000 and not self.mode == Mode.LAND:
            #self.return_to_launch()
            #self.engage_return_mode()
            #self.land()
            self.start_landing()
            self.mode = Mode.LAND

        self.stopwatch += 1 

        #if self.mode == Mode.FLY:
        #x, y = self.calc_velocity_vec()
        #self.publish_position_setpoint(self.target_x, self.target_y, self.takeoff_height)
        #self.publish_velocity_setpoint(x, y, 0.0)

        current_x = self.vehicle_local_position.x
        current_y = self.vehicle_local_position.y 
        current_z = self.vehicle_local_position.z
        #print(f'x: {current_x} y: {current_y} z: {current_z}')

# Get velocity direction vector
# Get velocity magnitude
            
def reached_approx(pos, target, tolerance):
    dist_x = pos[0] - target[0]
    dist_y = pos[1] - target[1]
    dist = math.sqrt(dist_x ** 2 + dist_y ** 2)

    return tolerance >= dist

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = Gooncopter()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
