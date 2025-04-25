import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion

class DesiredPoseProcessor(Node):
    
    def __init__(self):
        super().__init__('desired_pose_processor')
        self.get_logger().info("Started desired_pose_processor Node")
        self.publisher_ = self.create_publisher(Pose, '/desired_pose', 10)
        self.subscriber_ = self.create_subscription(String, '/robot/raw/target_pose', self.call_back, 10)
        
    def call_back(self, msg):
        self.get_logger().info(f"Message Received: {msg.data}")
        try:
            data = json.loads(msg.data)
            position = data["position"]
            rotation = data["rotation"]
            desired_pose = Pose()

            desired_pose.position.x = float(position["x"])
            desired_pose.position.y = float(position["y"])
            desired_pose.position.z = float(position["z"])
                
            desired_pose.orientation.x = float(rotation["x"])
            desired_pose.orientation.y = float(rotation["y"])
            desired_pose.orientation.z = float(rotation["z"])
            
            self.publisher_.publish(desired_pose)
            self.get_logger().info(f"Published Pose message: {desired_pose}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DesiredPoseProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
