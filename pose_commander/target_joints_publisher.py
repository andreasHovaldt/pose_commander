
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from lbr_fri_msgs.msg import LBRPositionCommand


class JointsPublisher(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.callback_group = ReentrantCallbackGroup()

        # Pose publisher
        self.target_joints_pub = self.create_publisher(LBRPositionCommand, "/target_joints_left", 1)
        self.callback_group.add_entity(self.target_joints_pub)
        
        self.timer = self.create_timer(1.0, self.new_joint_target)
        
        self.run_once_flag = True
        
    def new_joint_target(self):
    	if self.run_once_flag == True:
	    	msg = LBRPositionCommand()
	    	
	    	msg.joint_position = [0.57, 0.28, -0.31, -1.25, -0.10, 1.40, 0.0]
	    	
	    	self.target_joints_pub.publish(msg)

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
    JointsPublisherNode = JointsPublisher("joints_publisher")
    executor.add_node(JointsPublisherNode)
    try:
        executor.spin()
    finally:
        # Cleanup code, if any
        JointsPublisherNode.destroy_node()
        rclpy.shutdown()


    rclpy.init(args=args)
    rclpy.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

