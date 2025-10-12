import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

class GripperParam(Node):
	def __init__(self):
		super().__init__("gripper_param")
		self.declare_parameter("gripper_state_param", "OPENED")

		self.add_on_set_parameters_callback(self.paramChangeCallback)

	def paramChangeCallback(self, params):
		result = SetParametersResult()

		for param in params:
			if param.name == "gripper_state_param" and param.type_ == Parameter.Type.STRING:
				self.get_logger().info(f"gripper_state_param set to: {param.value}")
				result.successful = True

		return result
	
def main():
	rclpy.init()
	gripper_param = GripperParam()
	rclpy.spin(gripper_param)
	gripper_param.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()