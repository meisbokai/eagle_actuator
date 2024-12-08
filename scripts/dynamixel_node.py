#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Bool
from dynamixel_sdk import *
import numpy as np

class DynamixelROSNode:
    def __init__(self):
        # ROS node initialization
        rospy.init_node('dynamixel_controller')

        # Motor Control Table Addresses
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_CURRENT = 102  
        self.ADDR_GOAL_POSITION = 116
        self.PROTOCOL_VERSION = 2.0

        # ROS parameters
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 57600)

        # Motor configuration
        self.motor_config = {
            0: {
                'type': 'current',
                'current_limit': 1000,  # Max current in mA
                'current_range': (-1000, 1000)  # Example current range
            },
            1: {
                'type': 'current', 
                'current_limit': 1000,
                'current_range': (-1000, 1000)
            },
            2: {
                'type': 'position',
                'min_pos': 2850,  # Minimum position
                'max_pos': 3200   # Maximum position
            }
        }

        # Current targets for motors
        self.motor_targets = {
            0: None,  # Current for motor 0
            1: None,  # Current for motor 1
            2: None   # Position for motor 2
        }

        # Setup port and packet handler
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize port
        if not self.port_handler.openPort():
            rospy.logerr(f"Failed to open port {self.port}")
            return
        if not self.port_handler.setBaudRate(self.baudrate):
            rospy.logerr(f"Failed to set baudrate to {self.baudrate}")
            return

        # Group sync write setup
        self.group_sync_write = GroupSyncWrite(
            self.port_handler, 
            self.packet_handler, 
            self.ADDR_GOAL_CURRENT,  # Start with current control address
            2  # 2 bytes for current
        )

        # ROS subscribers
        rospy.Subscriber('motor_0_torque', Float64, lambda msg: self.torque_callback(0, msg))
        rospy.Subscriber('motor_1_torque', Float64, lambda msg: self.torque_callback(1, msg))
        rospy.Subscriber('motor_2_grip', Bool, self.position_callback)

        # Periodic sync write timer
        self.sync_timer = rospy.Timer(rospy.Duration(0.05), self.sync_write_commands)

    def _torque_to_current(self, torque):
        """
        Map torque input to motor current.
        Current = 0.552 tau + 0.145
        
        Args:
            torque (float): Desired torque 
        
        Returns:
            current (float): Required current
        """
        # Precalculated relation
        current = 0.552*torque + 0.145

        # TODO: Add filtering if required
        filtered_current = current
        return filtered_current

    def torque_callback(self, motor_id, msg):
        """Callback for torque-controlled motors."""
        try:
            # Map torque to current
            current = self._map_torque_to_current(msg.data)
            self.motor_targets[motor_id] = current
            rospy.logdebug(f"Motor {motor_id} target current: {current} mA")
        except Exception as e:
            rospy.logerr(f"Motor {motor_id} current control error: {e}")

    def position_callback(self, msg):
        """Callback for position-controlled motor."""
        try:
            config = self.motor_config[2]
            # Toggle between min and max positions
            self.motor_targets[2] = config['max_pos'] if msg.data else config['min_pos']
            rospy.logdebug(f"Motor 2 target position: {self.motor_targets[2]}")
        except Exception as e:
            rospy.logerr(f"Motor 2 position control error: {e}")

    def sync_write_commands(self, event=None):
        """Synchronously write commands for all motors."""
        try:
            # Clear previous parameters
            self.group_sync_write.clearParam()

            # Track which write mode we're using
            write_current = any(
                target is not None and self.motor_config[motor_id]['type'] == 'current' 
                for motor_id, target in self.motor_targets.items()
            )

            # Change sync write address based on current mode or position mode
            if write_current:
                # Current control mode
                self.group_sync_write = GroupSyncWrite(
                    self.port_handler, 
                    self.packet_handler, 
                    self.ADDR_GOAL_CURRENT,
                    2  # 2 bytes for current
                )
            else:
                # Position control mode
                self.group_sync_write = GroupSyncWrite(
                    self.port_handler, 
                    self.packet_handler, 
                    self.ADDR_GOAL_POSITION,
                    4  # 4 bytes for position
                )

            # Add parameters for motors with targets
            for motor_id, target in self.motor_targets.items():
                if target is not None:
                    # Prepare parameter based on motor type
                    if self.motor_config[motor_id]['type'] == 'current':
                        # Convert current to 2-byte array
                        param = [
                            DXL_LOBYTE(abs(int(target))),
                            DXL_HIBYTE(abs(int(target))) | (0x40 if target < 0 else 0)
                        ]
                    else:  # position
                        # Convert position to 4-byte array
                        param = [
                            DXL_LOBYTE(DXL_LOWORD(target)),
                            DXL_HIBYTE(DXL_LOWORD(target)),
                            DXL_LOBYTE(DXL_HIWORD(target)),
                            DXL_HIBYTE(DXL_HIWORD(target)),
                        ]
                    
                    # Add parameter for this motor
                    result = self.group_sync_write.addParam(motor_id, param)
                    if not result:
                        rospy.logerr(f"Failed to add parameter for motor {motor_id}")

            # Execute synchronized write if there are parameters
            if self.group_sync_write.getParamCount() > 0:
                result = self.group_sync_write.txPacket()
                if result != COMM_SUCCESS:
                    rospy.logerr("Failed to execute synchronized movement")

        except Exception as e:
            rospy.logerr(f"Sync write error: {e}")

    def run(self):
        """Keep node running and handle shutdown."""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            # Stop the sync timer
            self.sync_timer.shutdown()
            # Close port
            self.port_handler.closePort()

if __name__ == '__main__':
    try:
        node = DynamixelROSNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"Dynamixel Node Error: {e}")
