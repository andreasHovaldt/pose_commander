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
    def __init__(self):
        super().__init__("right_pose_commander_client")
        self.right_pose_commander_client = self.create_client(TargetPose, '/right_target_pose')
        while not self.right_pose_commander_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pose_commander_right service not available, waiting again...')

    def send_request_right(self, Pose):
        #msg = TargetPose.Request()
        msg = Pose
        self.get_logger().info("Sending request to right arm")
        self.future = self.right_pose_commander_client.call_async(msg)
        self.get_logger().info("Sent request. Waiting for future to complete")
        rclpy.spin_until_future_complete(node=self, future=self.future)
        self.get_logger().info("Future complete")
        return self.future.result()


class LLM_Executor_Server(Node):
    def __init__(self):
        super().__init__("LLM_Executor_Server")

        self.llm_executor_service_ = self.create_service(BimanualJson, "llm_executor", self.llm_service_callback)



        # Make callback groups
        self.callback_group_llm_ = MutuallyExclusiveCallbackGroup()
        self.callback_group_llm_.add_entity(self.llm_executor_service_)

    def send_request_left(self, pose):
        client_node = LeftPoseCommanderClient()
        client_node.send_request_left(pose)
        client_node.destroy_node()

    def send_request_right(self, pose):
        client_node_right = RightPoseCommanderClient()
        client_node_right.send_request_right(pose)
        client_node_right.destroy_node()

    def llm_service_callback(self, request, response):

        try:
            self.steps = json.loads(request.json_steps)

            for step in self.steps:
                left_coordinates = self.steps[step]["left_ee_coor"]
                right_coordinates = self.steps[step]["right_ee_coor"]
                left_orientation = self.steps[step]["left_ee_orientation"]
                right_orientation = self.steps[step]["right_ee_orientation"]
                left_gripper = self.steps[step]["left_gripper_state"]
                right_gripper = self.steps[step]["right_gripper_state"]

                # Make plan for left arm and execute
                left_pose = TargetPose.Request()
                self.get_logger().info("constructing target pose")
                left_pose.px = float(left_coordinates[0])
                left_pose.py = float(left_coordinates[1])
                left_pose.pz = float(left_coordinates[2])
                left_pose.qx = float(left_orientation[0])
                left_pose.qy = float(left_orientation[1])
                left_pose.qz = float(left_orientation[2])
                left_pose.qw = float(left_orientation[3])

                self.get_logger().info(f"sending left pose {left_pose}")
                self.send_request_left(left_pose)

                # Make plan for right arm and execute
                right_pose = TargetPose.Request()
                right_pose.px = float(right_coordinates[0])
                right_pose.py = float(right_coordinates[1])
                right_pose.pz = float(right_coordinates[2])
                right_pose.qx = float(right_orientation[0])
                right_pose.qy = float(right_orientation[1])
                right_pose.qz = float(right_orientation[2])
                right_pose.qw = float(right_orientation[3])
                self.get_logger().info(f"sending right pose {right_pose}")
                self.send_request_right(right_pose)

            response.success = True
            response.msg = "Executed without error."

        except Exception as e:
            self.get_logger().error(f"Converting string to json failed with error: {e}")
            response.success = False
            response.msg = str(e)

        return response

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