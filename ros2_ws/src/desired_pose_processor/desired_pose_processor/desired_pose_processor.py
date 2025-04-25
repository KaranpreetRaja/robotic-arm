import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class DesiredPoseProcessor(Node):
    
    def __init__(self):
        super().__init__('desired_pose_processor')
        self.get_logger().info("Started desired_pose_processor Node")
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.subscriber_ = self.create_subscription(String, '/robot/raw/target_pose', self.call_back, 10)
        self.subscriber_
        self.publisher_
        
    def call_back(self, msg):
        self.get_logger().info(f"Message Recieved {msg.data}")
        desired_pose = json.decoder(msg.data)
        

def main(args=None):
    rclpy.init(args=args)
    node = DesiredPoseProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
    