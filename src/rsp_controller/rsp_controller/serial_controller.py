import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SerialController(Node):
	def __init__(self):
		super().__init__('serial_controller')

		self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_call, 10)

		# self.last_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.last_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


	def joint_state_call(self, joints):
		joint_states = list(re.split(r'[\[\],]',str(joints.position))[4:-1])
		joint_states = [float(f) for f in joint_states]
		joint_states = [f"{f:.2f}" for f in joint_states]
		joint_states = [float(f) for f in joint_states]
		
		if(self.last_state == joint_states):
			pass
		else:
			self.get_logger().info(f"MOVE>{joint_states}<")

		self.last_state = joint_states.copy()

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