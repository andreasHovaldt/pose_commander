import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# Import for LLM service server
import json
from custom_interfaces.srv import BimanualJson
from custom_interfaces.srv import TargetPose



class LeftPoseCommanderClient(Node):
    def __init__(self):
        super().__init__("left_pose_commander_client")
        self.left_pose_commander_client = self.create_client(TargetPose, '/left_target_pose')
        while not self.left_pose_commander_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pose_commander_left service not available, waiting again...')

    def send_request_left(self, Pose):
        #msg = TargetPose.Request()
        msg = Pose
        self.get_logger().info("Sending request to left arm")
        self.future = self.left_pose_commander_client.call_async(msg)
        self.get_logger().info("Sent request. Waiting for future to complete")
        rclpy.spin_until_future_complete(node=self, future=self.future)
        self.get_logger().info("Future complete")
        return self.future.result()

class RightPoseCommanderClient(Node):
    def __int__(self):
        super().__int__("right_pose_commander_client")
        self.right_pose_commander_client = self.create_client(TargetPose, '/right_target_pose')

        while not self.right_pose_commander_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pose_commander_right service not available, waiting again...')

    def send_request_right(self, Pose):
        #msg = TargetPose.Request()
        msg = Pose
        self.future = self.right_pose_commander_client.call_async(msg)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class LLM_Executor_Server(Node):
    def __init__(self):
        super().__init__("LLM_Executor_Server")



        self.llm_executor_service_ = self.create_timer(5.0, self.llm_service_callback)

        # Make callback groups
        self.callback_group_llm_ = MutuallyExclusiveCallbackGroup()
        self.callback_group_llm_.add_entity(self.llm_executor_service_)

        # Emulator flag to only run once
        self.run_once = True



    def send_request_left(self, pose):
        client_node = LeftPoseCommanderClient()
        client_node.send_request_left(pose)
        client_node.destroy_node()

    def send_request_right(self, pose):
        client_node = RightPoseCommanderClient()
        client_node.send_request_right(pose)
        client_node.destroy_node()


    def llm_service_callback(self):

        if self.run_once == True:
            # Make plan for left arm and execute
            left_pose = TargetPose.Request()
            self.get_logger().info("constructing target pose")
            left_pose.px = float(0)
            left_pose.py = float(0.3)
            left_pose.pz = float(0.4)
            left_pose.qx = float(1)
            left_pose.qy = float(0)
            left_pose.qz = float(1)
            left_pose.qw = float(0)

            self.get_logger().info(f"sending left pose {left_pose}")
            self.send_request_left(left_pose)
    
            # Make plan for right arm and execute
            right_pose = TargetPose.Request()
            right_pose.px = float(0)
            right_pose.py = float(0)
            right_pose.pz = float(0)
            right_pose.qx = float(0)
            right_pose.qy = float(0)
            right_pose.qz = float(0)
            right_pose.qw = float(0)
            self.get_logger().info(f"sending right pose {right_pose}")
            #self.send_request_right(right_pose)

            self.run_once = False

def main(args: list = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(2)
    mission_executor = LLM_Executor_Server()
    executor.add_node(mission_executor)
    try:
        executor.spin()
    finally:
        # Cleanup code, if any
        mission_executor.destroy_node()
        rclpy.shutdown()


    rclpy.init(args=args)
    rclpy.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
