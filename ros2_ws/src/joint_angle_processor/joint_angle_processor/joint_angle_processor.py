import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointAngleProcessor(Node):
    
    def __init__(self):
        super().__init__('joint_angle_processor')
        self.get_logger().info("Started JointAngleProcessor Node")

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.subscription_ = self.create_subscription(
            String,
            '/robot/raw/joint_angles',
            self.call_back,
            10
        )
        
        self.joint_names = [
            'arm_j1',
            'arm_j2',
            'arm_j3',
            'arm_j4',
            'arm_j5',
            'arm_j6'
        ]
        
    def call_back(self, msg: String):
        data = json.loads(msg.data)
        self.get_logger().debug(f"Raw JSON: {data}")

        ori = data["orientation"]
        positions = [
            math.radians(float(ori["value1"])),
            math.radians(float(ori["value2"])),
            math.radians(float(ori["value3"])),
            math.radians(float(ori["value4"])),
            math.radians(float(ori["value5"])),
            math.radians(float(ori["value6"]))
        ]

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=1, nanosec=0)

        traj.points = [pt]

        # Optionally call your callback
        if data.get("isCallbackEnabled", False):
            self.karansFunction()

        self.get_logger().info(f"Publishing JointTrajectory: {traj}")
        self.publisher_.publish(traj)
        
    def karansFunction(self):
        self.get_logger().info("Karan Function Called")

def main(args=None):
    rclpy.init(args=args)
    node = JointAngleProcessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
