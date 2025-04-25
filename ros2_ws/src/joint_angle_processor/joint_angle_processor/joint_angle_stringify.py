import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class JointAngleStringify(Node):
    
    def __init__(self):
        super().__init__('joint_angle_stringify')
        self.get_logger().info("Started JointAngleStringify Node")
        self.publisher_ = self.create_publisher(String, '/string/joint_states', 10)
        self.subscriber_ = self.create_subscription(JointState, '/joint_states', self.call_back, 10)
        
    def call_back(self, msg):
        joint_data = dict(zip(msg.name, msg.position))
        json_str = json.dumps(joint_data)
        string_msg = String()
        string_msg.data = json_str
        self.publisher_.publish(string_msg)
        self.get_logger().info(f"Published JSON: {json_str}")

def main(args=None):
    rclpy.init(args=args)
    node = JointAngleStringify()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
