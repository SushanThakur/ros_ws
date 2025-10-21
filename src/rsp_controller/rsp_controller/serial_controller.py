import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy

class SerialController(Node):
	def __init__(self):
		super().__init__('serial_controller')

		self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.serial_publisher, 10)
		
		self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_call, 10)

		# self.last_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.last_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.last_gripper_state = [0.0, 0.0]

	def joy_call(self):
		pass

	def serial_publisher(self, joints):
		joint_states = list(re.split(r'[\[\],]',str(joints.position))[2:-1])
		joint_states = [float(f) for f in joint_states]
		joint_states = [f"{f:.3f}" for f in joint_states]
		joint_states = [float(f) for f in joint_states]

		gripper_state = joint_states[:2]
		joint_states = joint_states[2:-1]
		
		if(self.last_joint_state == joint_states):
			pass
		else:
			self.get_logger().info(f"MOVE{joint_states}")

		if self.last_gripper_state == gripper_state:
			pass
		else:
			self.get_logger().info(f"GRIP{gripper_state}")

		self.last_joint_state = joint_states.copy()
		self.last_gripper_state = gripper_state.copy()

def main():
	try:
		rclpy.init()
		serial_controller = SerialController()
		rclpy.spin(serial_controller)
	except KeyboardInterrupt:
		pass
	finally: 
		if rclpy.ok():
			serial_controller.destroy_node()
			rclpy.shutdown()

if __name__ == '__main__':
	main()