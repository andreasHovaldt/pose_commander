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

class LLM_Executor_Server(Node):
    def __init__(self):
        super().__init__("LLM_Executor_Server")

        self.llm_executor_service_ = self.create_service(BimanualJson, "llm_executor", self.llm_service_callback)

        self.left_pose_commander_client = self.create_client(TargetPose, '/left_target_pose')
        self.right_pose_commander_client = self.create_client(TargetPose, '/right_target_pose')
        while not self.left_pose_commander_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pose_commander_left service not available, waiting again...')
        while not self.right_pose_commander_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pose_commander_right service not available, waiting again...')

        # Make callback groups
        self.callback_group_llm_ = MutuallyExclusiveCallbackGroup()
        self.callback_group_llm_.add_entity(self.llm_executor_service_)

    def send_request_left(self, Pose):
        msg = TargetPose.Request()
        msg = Pose
        self.future = self.left_pose_commander_client.call_async(msg)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_request_right(self, Pose):
        msg = TargetPose.Request()
        msg = Pose
        self.future = self.right_pose_commander_client.call_async(msg)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def llm_service_callback(self, request, response):

        try:
            self.steps = json.loads(request)

            for step in self.steps:
                left_coordinates = self.steps[step]["left_ee_coor"]
                right_coordinates = self.steps[step]["right_ee_coor"]
                left_orientation = self.steps[step]["left_ee_orientation"]
                right_orientation = self.steps[step]["right_ee_orientation"]
                left_gripper = self.steps[step]["left_gripper_state"]
                right_gripper = self.steps[step]["right_gripper_state"]

                # Make plan for left arm and execute
                left_pose = Pose(
                    position=Point(x=left_coordinates[0], y=left_coordinates[1], z=left_coordinates[2]),
                    orientation=Quaternion(x=left_orientation[0], y=left_orientation[1], z=left_coordinates[2],
                                           w=left_orientation[3]),
                )

                self.send_request_left(left_pose)

                # Make plan for right arm and execute
                right_pose = Pose(
                    position=Point(x=right_coordinates[0], y=right_coordinates[1], z=right_coordinates[2]),
                    orientation=Quaternion(x=right_orientation[0], y=right_orientation[1], z=right_coordinates[2],
                                           w=right_orientation[3]),
                )

                self.send_request_right(right_pose)

            response.success = True
            response.message = "Executed without error."

        except Exception as e:
            self.get_logger().error(f"Converting string to json failed with error: {e}")
            response.success = False
            response.message = str(e)

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