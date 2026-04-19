#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so101_hardware_interface.motors.feetech import FeetechMotorsBus
import time
from serial.serialutil import SerialException
import math

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')
        self.subscription = self.create_subscription(
            JointState,
            'real_joint_commands',
            self.joint_state_callback,
            10)
        self.publisher_ = self.create_publisher(JointState, 'real_joint_states', 10)
        self.timer = self.create_timer(0.04, self.publish_real_joint_states)  # 0.04 -> 25 Hz

        self.motors_bus = FeetechMotorsBus(
            port="/dev/ttyACM0",
            motors={},
        )
        self.motors_bus.connect()
        self.motors_bus.set_bus_baudrate(1_000_000)

        import json
        import os
        json_path = os.path.join(os.getcwd(), "src", "arm_hardware_interface", "resource", "so101_calibration.json")
        try:
            with open(json_path, 'r') as f:
                self.calibration = json.load(f)
            self.get_logger().info(f"Successfully loaded calibration from {json_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration JSON from {json_path}: {e}. Stopping.")
            raise e

        self.joint_to_motor_id = {k: v["id"] for k, v in self.calibration.items()}

        # Initialize motors with the correct IDs and model
        self.motor_names = list(self.joint_to_motor_id.keys())
        self.motors_bus.motors = {name: (self.joint_to_motor_id[name], "sts3215") for name in self.motor_names}
        
        # Keep track of previous positions to calculate rough velocity
        self.prev_rad_positions = [0.0] * len(self.motor_names)

        self.configure_motors()
        self.lock_current_position()
        self.set_torque(True)

    def configure_motors(self):
        for name in self.motor_names:
            self.motors_bus.write("Goal_Speed", 100, name)  # Set speed to 100
            self.motors_bus.write("Acceleration", 50, name)  # Set acceleration to 50

    def ticks_to_radians(self, name, raw_ticks):
        calib = self.calibration[name]
        
        if name == "gripper":
            zero_pos = calib["range_min"]
        else:
            zero_pos = (calib["range_min"] + calib["range_max"]) / 2.0
            
        offset_ticks = raw_ticks - zero_pos
        
        # Handle 4096 encoder wrap-around elegantly by forcing the shortest path to center
        offset_ticks = ((offset_ticks + 2048) % 4096) - 2048
        
        if calib.get("drive_mode", 0) == 1:
            offset_ticks = -offset_ticks
            
        return (offset_ticks / 2048.0) * math.pi

    def radians_to_ticks(self, name, radians):
        calib = self.calibration[name]
        
        if name == "gripper":
            zero_pos = calib["range_min"]
        else:
            zero_pos = (calib["range_min"] + calib["range_max"]) / 2.0
            
        offset_ticks = (radians / math.pi) * 2048.0
        
        if calib.get("drive_mode", 0) == 1:
            offset_ticks = -offset_ticks
            
        raw_ticks = int(zero_pos + offset_ticks)
        
        # Clamp to safe recorded continuous range
        raw_ticks = int(max(calib["range_min"], min(calib["range_max"], raw_ticks)))
        
        # Wrap safely back into the 0-4095 range for the Feetech serial bus
        return raw_ticks % 4096

    def lock_current_position(self):
        motor_ids = list(self.joint_to_motor_id.values())
        motor_models = ["sts3215"] * len(motor_ids)
        try:
            positions = self.motors_bus.read_with_motor_ids(motor_models, motor_ids, "Present_Position")
            if positions:
                # Validate the positions are not completely wild before locking
                clamped_positions = []
                for name, pos in zip(self.motor_names, positions):
                    calib = self.calibration[name]
                    clamped_pos = int(max(calib["range_min"], min(calib["range_max"], pos)))
                    clamped_positions.append(clamped_pos)
                
                self.motors_bus.write_with_motor_ids(motor_models, motor_ids, "Goal_Position", clamped_positions)
                self.get_logger().info("Locked motors to current physical positions.")
        except Exception as e:
            self.get_logger().error(f"Could not lock position: {e}")

    def set_torque(self, enable):
        torque_value = 1 if enable else 0
        for name in self.motor_names:
            self.motors_bus.write("Torque_Enable", torque_value, name)
        self.get_logger().info(f"Torque {'enabled' if enable else 'disabled'} for all motors.")

    def joint_state_callback(self, msg):
        motor_ids = []
        motor_values = []
        motor_models = []

        for name, position in zip(msg.name, msg.position):
            if name in self.motor_names:
                calib = self.calibration[name]
                motor_id = calib["id"]
                
                motor_value = self.radians_to_ticks(name, position)
                
                motor_ids.append(motor_id)
                motor_values.append(motor_value)
                motor_models.append("sts3215")

        if motor_ids:
            self.motors_bus.write_with_motor_ids(motor_models, motor_ids, "Goal_Position", motor_values)
            # self.get_logger().info(f"Joint positions: {list(zip(msg.name, motor_values))}")

    def publish_real_joint_states(self):
        motor_ids = list(self.joint_to_motor_id.values())
        motor_models = ["sts3215"] * len(motor_ids)
        try:
            positions = self.motors_bus.read_with_motor_ids(motor_models, motor_ids, "Present_Position")
        except (ConnectionError, SerialException) as e:
            self.get_logger().error(f"Connection error: {e}")
            self.get_logger().info("Attempting to reconnect...")
            while True:
                try:
                    self.motors_bus.reconnect()
                    self.get_logger().info("Reconnected to motors.")
                    return
                except Exception as e:
                    self.get_logger().error(f"Reconnection failed: {e}")
                    time.sleep(1)  # Wait for 1 second before retrying

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.motor_names
        
        rad_positions = []
        rad_velocities = []
        for i, (name, pos) in enumerate(zip(self.motor_names, positions)):
            calib = self.calibration[name]
            
            rad = self.ticks_to_radians(name, pos)
            rad_positions.append(rad)
            
            velocity = (rad - self.prev_rad_positions[i]) / 0.04
            rad_velocities.append(velocity)
            self.prev_rad_positions[i] = rad
            
        joint_state_msg.position = rad_positions
        joint_state_msg.velocity = rad_velocities

        self.publisher_.publish(joint_state_msg)
        # self.get_logger().info(f"Published real joint states: {positions}")
        # self.get_logger().info(f"Published real joint states (radians): {joint_state_msg.position}")

def main(args=None):
    rclpy.init(args=args)
    hardware_interface = HardwareInterface()
    rclpy.spin(hardware_interface)
    hardware_interface.motors_bus.disconnect()
    hardware_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
