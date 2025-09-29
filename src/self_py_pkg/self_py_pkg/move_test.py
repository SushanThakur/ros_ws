#!/usr/bin/env python3
"""
DIRECT JOINT CONTROL - NO MOVETI NEEDED
Fuck complexity, let's make the arm move TODAY
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class DirectJointControl(Node):
    def __init__(self):
        super().__init__('direct_joint_control')
        
        # PS4 controller input
        self.ps4_sub = self.create_subscription(Joy, '/joy', self.ps4_callback, 10)
        
        # Publisher to joint trajectory controller  
        self.traj_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Your joint names from URDF - UPDATE THESE TO MATCH YOUR URDF!
        self.joint_names = [
            'joint_1', 
            'joint_2', 
            'joint_3',
            'joint_4', 
            'joint_5', 
            'joint_6', 
            'joint_7'
        ]
        
        # Pre-defined joint positions (in radians)
        self.presets = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.5, -0.5, 0.8, 0.0, 0.0, 0.0, 0.0],
            'extended': [0.0, -1.0, 1.5, 0.0, 0.0, 0.0, 0.0],
            'wave': [0.0, -0.5, 1.0, 0.0, 1.0, 0.0, 0.0]
        }
        
        self.get_logger().info("🎮 DIRECT JOINT CONTROL READY - PRESS PS4 BUTTONS!")
        self.get_logger().info("X=Home, ○=Ready, △=Extended, □=Wave")
    
    def ps4_callback(self, msg):
        buttons = msg.buttons
        
        if buttons[0]:  # X button - HOME
            self.move_to_joints(self.presets['home'], "HOME")
        elif buttons[1]:  # Circle button - READY  
            self.move_to_joints(self.presets['ready'], "READY")
        elif buttons[2]:  # Triangle button - EXTENDED
            self.move_to_joints(self.presets['extended'], "EXTENDED")
        elif buttons[3]:  # Square button - WAVE
            self.move_to_joints(self.presets['wave'], "WAVE")
        
        # Log that we got a button press
        if any(buttons[:4]):  # If any face button was pressed
            self.get_logger().info("🎮 BUTTON PRESSED - MOVING ARM!")
    
    def move_to_joints(self, positions, pose_name):
        """Send joint positions directly to the controller"""
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2  # Take 2 seconds to get there
        
        traj.points.append(point)
        self.traj_pub.publish(traj)
        
        self.get_logger().info(f"🚀 MOVING TO {pose_name}: {positions}")

def main():
    rclpy.init()
    node = DirectJointControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n💥 SHUTTING DOWN - GOODBYE!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()