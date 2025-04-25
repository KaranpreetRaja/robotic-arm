import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointAngleProcessor(Node):
    
    def __init__(self):
        super().__init__('joint_angle_processor')
        self.get_logger().info("Started JointAngleProcessor Node")
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_trajectory_controller/joint_trajectory', 10)
        self.subscriber_ = self.create_subscription(String, '/robot/raw/joint_angles', self.call_back, 10)
        self.subscriber_
        self.publisher_
        
    def call_back(self, msg):
        ## Test
        # command = JointTrajectory()
        # command.joint_names = ['arm_j6']                
        # pt = JointTrajectoryPoint()
        # pt.positions = [1.0]
        # pt.time_from_start.sec = 2
        # pt.time_from_start.nanosec = 0
        # command.points = [pt]

        command = Float64MultiArray()
        data = json.loads(msg.data)
        self.get_logger().info(f"Message Recieved {data}")
                
        orientation = data["orientation"]

        command.data = [
            float(orientation["value1"]),
            float(orientation["value2"]),
            float(orientation["value3"]),
            float(orientation["value4"]),
            float(orientation["value5"]),
            float(orientation["value6"])
        ]
        
        if (bool(data["isCallbackEnabled"])):
            self.karansFunction()

        self.get_logger().info(f"Command {command.data}")
        self.publisher_.publish(command)
        
    def karansFunction(self):
        self.get_logger().info(f"Karan Function Called")

def main(args=None):
    rclpy.init(args=args)
    node = JointAngleProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
    