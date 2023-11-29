
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class PosePublisher(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.callback_group = ReentrantCallbackGroup()

        # Pose publisher
        self.target_pose_pub = self.create_publisher(Pose, "/target_pose_left", 1)
        self.callback_group.add_entity(self.target_pose_pub)
        
        self.timer = self.create_timer(10.0, self.new_pose)
        
        # State variables to switch between poses
        self.state = 10
        
    def new_pose(self):
    	msg = Pose()
    	if self.state == 0:
    		msg.position.z = 0.5
    		msg.position.x = 0.4
    		msg.position.y = 0.2
    		msg.orientation.x = -1.0
    		msg.orientation.w = 0.0007963
    		self.state = 1
    	elif self.state == 1:
    		msg.position.z = 0.7
    		msg.position.x = 0.4
    		msg.position.y = 0.2
    		msg.orientation.x = -1.0
    		msg.orientation.w = 0.0007963
    		self.state = 10
    	elif self.state == 10:
    		msg.position.x = 0.0
    		msg.position.y = 0.3
    		msg.position.z = 0.5
    		msg.orientation.x = 1.0
    		msg.orientation.y = 0.0
    		msg.orientation.z = 1.0
    		msg.orientation.w = 0.0
    	self.target_pose_pub.publish(msg)

    def quaternion_to_euler(self, q):
        w, x, y, z = q
        euler = np.zeros([3])

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        euler[0] = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        euler[1] = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        euler[2] = math.atan2(t3, t4)

        self.get_logger().info(f"Euler angles: {euler}")
        return euler

def main(args: list = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    PosePublisherNode = PosePublisher("pose_publisher")
    executor.add_node(PosePublisherNode)
    try:
        executor.spin()
    finally:
        # Cleanup code, if any
        PosePublisherNode.destroy_node()
        rclpy.shutdown()


    rclpy.init(args=args)
    rclpy.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

