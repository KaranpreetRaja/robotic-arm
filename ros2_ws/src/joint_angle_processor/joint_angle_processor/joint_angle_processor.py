import asyncio
from gc import callbacks
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointAngleProcessor(Node):
    
    def __init__(self):
        super().__init__('joint_angle_processor')
        self.get_logger().info("Started JointAngleProcessor Node")
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.subscriber_ = self.create_subscription(String, '/robot/raw/joint_angles', self.call_back, 10)
        self.subscriber_
        self.publisher_
        
    def call_back(self, msg):
        command = JointTrajectory()
        command.joint_names = ['arm_j6']                
        pt = JointTrajectoryPoint()
        pt.positions = [1.0]
        pt.time_from_start.sec = 2
        pt.time_from_start.nanosec = 0
        command.points = [pt]

        self.get_logger().info(f"Message Recieved {msg.data}")
        self.publisher_.publish(command)

def main(args=None):
    rclpy.init(args=args)
    node = JointAngleProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
    