import os
import time

import kinpy
import numpy as np
import math
import rclpy
import xacro
from ament_index_python import get_package_share_directory
from lbr_fri_msgs.msg import LBRPositionCommand, LBRState
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# Service
from custom_interfaces.srv import TargetPose

# Used for visualisation in RVIZ
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class PoseCommander(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.callback_group1 = MutuallyExclusiveCallbackGroup()
        self.callback_group2 = MutuallyExclusiveCallbackGroup()
        self.callback_group3 = MutuallyExclusiveCallbackGroup()
        self.callback_group4 = MutuallyExclusiveCallbackGroup()

        # something to compute robot kinematics
        self.robot_description = xacro.process(
            os.path.join(
                get_package_share_directory("lbr_description"),
                "urdf/left_arm/left_arm.urdf.xacro",
            )
        )
        self.kinematic_chain = kinpy.build_serial_chain_from_urdf(
            data=self.robot_description,
            root_link_name="table",
            end_link_name="tool0",
        )

        # publisher / subscriber to command robot
        self.lbr_position_command_ = LBRPositionCommand()
        self.lbr_position_command_pub_ = self.create_publisher(
            LBRPositionCommand, "/left/command/position", 1
        )
        self.lbr_state_sub_ = self.create_subscription(
            LBRState, "/left/state", self.update_joint_angles, 1
        )
        self.callback_group1.add_entity(self.lbr_state_sub_)

        # Service server for receiving new poses to move cartesian to
        self.left_pose_commander_server = self.create_service(TargetPose, '/left_target_pose', self.plan_new_target)

        # Subscriber to new robot position
        #self.target_pose_sub = self.create_subscription(Pose, "/target_pose_left", self.plan_new_target, 1)
        #self.callback_group2.add_entity(self.target_pose_sub)

        # Subscriber to new robot angles
        #self.target_joints_sub = self.create_subscription(LBRPositionCommand, "/target_joints_left", self.plan_new_joint_target, 1)
        #self.callback_group3.add_entity(self.target_joints_sub)

        # Trajectory execution callback function
        self.trajectory_execution_timer = self.create_timer(0.03, self.execute_trajectory)
        self.callback_group4.add_entity(self.trajectory_execution_timer)

        # Pose publisher
        self.state_pose_publisher = self.create_publisher(Pose, "/state_pose_left", 1)

        # FOR SIMULATION
        self.sim_command = JointState()
        self.sim_command_pub = self.create_publisher(JointState, "/joint_states", 1)
        # END FOR SIMULATION

        # Security box defined by two 3-dimensional points, defined in each corner of the box. First corner needs to be all maximum values, and second all minimum values
        self.security_box = np.array([[0.5, 0.5, 1.5], [-0.5, -0.3, 0.03]])

        # Current joint angles
        self.joint_angles = 0


        # Newest trajectory and iterator
        self.trajectory = 0
        self.joint_trajectory = 0
        self.t_iterator = 0

        # New trajectory flag - SHOULD BE FALSE BY DEFAULT. IF TRUE, THAT WAS FOR TESTING. SET TO FALSE!!!!!
        self.new_trajectory_flag = False
        self.new_joint_trajectory_flag = False

    # List of improvements
    # 1. CHECK :) - Offload trajectory execution from the /lbr/state callback, and onto a timer function
    # 2. CHECK :) - Make number of interpolations dynamic, and proportional to length of path
    # 3. CHECK :) - Import new robot model, and make a left specific version of this program
    # 4. CHECK :) - Create security box where cartesian target points are allowed.
    # 5. CHECK :) - Create joint movement function
    # 6. CHECK :) - Create pose publisher
    # 7. Convert target_pose and target_joints from topics into services
    # 8. Make trajectory generation and execution function as third degree polynomials, and time a variable

    # Solves forward kinematics from the newest joint_angles, then converts orientation, creates new trajectory and marks this with a flag
    def plan_new_target(self, request, response):

        self.get_logger().info("Planning new target")

        msg = request
        # Compute forward kinematics
        fk = self.kinematic_chain.forward_kinematics(self.joint_angles)
        euler_rotation = self.quaternion_to_euler(fk.rot)
        start_pose = [fk.pos[0], fk.pos[1], fk.pos[2], euler_rotation[0], euler_rotation[1], euler_rotation[2]] # Use the fk to get pose

        # Convert target_pose to euler angles
        target_pose_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        target_pose_eul = self.quaternion_to_euler(target_pose_quat)
        target_pose = [msg.position.x, msg.position.y, msg.position.z, target_pose_eul[0], target_pose_eul[1], target_pose_eul[2]]

        # Perform security check, to make sure the target is inside the security box
        if self.security_check(target_pose) == True:
            # Generate trajectory
            self.trajectory = self.generate_cartesian_trajectory(start_pose, target_pose)
            self.new_trajectory_flag = True # Mark that a new trajectory is live
        else:
            self.get_logger().error(f"Invalid target. Target pose {target_pose} is not within the security box")

        while self.new_trajectory_flag == True:
            time.sleep(0.1)

        response.success = True
        return response

    def plan_new_joint_target(self, msg):
        self.get_logger().info(f"Joint target looks like this: {msg.joint_position}")
        joint_target = msg.joint_position
        self.joint_trajectory = self.generate_joint_trajectory(joint_target)
        self.new_joint_trajectory_flag = True

    # Function which performs security check. Returns True if desired_pose fits requirements and is safe. Returns false if not
    def security_check(self, desired_pose):
        if np.all(np.less_equal(desired_pose[0:3], self.security_box[0, :])) and np.all(np.greater_equal(desired_pose[0:3], self.security_box[1, :])):
            return True
        else:
            return False

    # Callback function for lbr_state. Receives current joint angles and saves it. Also calculates the pose and publishes it to the state_pose topic.
    def update_joint_angles(self, lbr_state: LBRState) -> None:
        # get joint states
        self.joint_angles = lbr_state.measured_joint_position

        # Calculate current pose and publish it
        fk = self.kinematic_chain.forward_kinematics(self.joint_angles)

        state_pose = Pose()
        state_pose.position.x = fk.pos[0]
        state_pose.position.y = fk.pos[1]
        state_pose.position.z = fk.pos[2]

        state_pose.orientation.x = fk.rot[0]
        state_pose.orientation.y = fk.rot[1]
        state_pose.orientation.z = fk.rot[2]
        state_pose.orientation.w = fk.rot[3]

        self.state_pose_publisher.publish(state_pose)


    # Function which executes trajectories. Cartesian trajectories take priority if multiple trajectories are live.
    def execute_trajectory(self):
        if self.new_trajectory_flag == True:
            # self.get_logger().info(f"Executing iteration nr: {self.t_iterator}")

            # Avoid conflicting trajectories, and deny the new joint trajectory, if there is any
            self.new_joint_trajectory_flag = False

            # Import the newest pose to move to, according to the iterator. Perform inverse kinematics
            new_pose = self.trajectory[self.t_iterator]
            new_pose = kinpy.Transform([new_pose[3], new_pose[4], new_pose[5]], [new_pose[0], new_pose[1], new_pose[2]])
            new_joint_angles = self.kinematic_chain.inverse_kinematics(new_pose, self.joint_angles)

            # Publish new joint angles
            self.lbr_position_command_.joint_position = new_joint_angles.tolist()
            self.lbr_position_command_pub_.publish(self.lbr_position_command_)

            # Publish new joint angles to rviz2
            self.sim_command.position = new_joint_angles.tolist() #[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.sim_command.name = ["A1", "A2", "A3", "A4", "A5", "A6", "A7"]
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            self.sim_command.header = header
            self.sim_command_pub.publish(self.sim_command)

            # Check if movement is complete. If true, then set flag to false and reset the trajectory and iterator
            if self.t_iterator == np.size(self.trajectory, 0)-1:
                self.t_iterator = 0
                self.trajectory = 0
                self.new_trajectory_flag = False
            # If movement is not complete, increase iterator by one
            else:
                # Increase iterator
                self.t_iterator = self.t_iterator + 1

        # If no cartesian trajectory is planned, execute joint trajectory (if any)
        elif self.new_joint_trajectory_flag == True:
            self.get_logger().info(f"Executing run nr. {self.t_iterator}")
            new_joint_angles = self.joint_trajectory[self.t_iterator]
            self.get_logger().info(f"New_joint_angles: {new_joint_angles}")

            # Publish new joint angles
            self.lbr_position_command_.joint_position = new_joint_angles.tolist()
            self.lbr_position_command_pub_.publish(self.lbr_position_command_)

            # Publish new joint angles to rviz2
            self.sim_command.position = new_joint_angles.tolist()  # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.sim_command.name = ["A1", "A2", "A3", "A4", "A5", "A6", "A7"]
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            self.sim_command.header = header
            self.sim_command_pub.publish(self.sim_command)

            # Check if movement is complete. If true, then set flag to false and reset the trajectory and iterator
            if self.t_iterator == np.size(self.joint_trajectory, 0)-1:
                self.t_iterator = 0
                self.joint_trajectory = 0
                self.new_joint_trajectory_flag = False
            # If movement is not complete, increase iterator by one
            else:
                # Increase iterator
                self.t_iterator = self.t_iterator + 1


    # Generates a linear trajectory from start_pose to end_pose. Number of interpolations scales with length,
    # and is dependent on the gain.
    def generate_cartesian_trajectory(self, start_pose, end_pose, gain=200):

        # Find length of path, and define interpolations proportional to this
        length = np.sqrt(np.sum(np.power(np.subtract(end_pose[0:3], start_pose[0:3]), 2)))
        interpolations = int(gain*length)
        self.get_logger().info(f"Interpolations: {interpolations}, Length: {length}")

        # Define the size of the increment
        increment = np.subtract(end_pose, start_pose) * (1 / interpolations)

        # Create an array to store the path.
        trajectory = np.zeros([interpolations, 6])

        # Fill out the path. Start from start pose, and add one increment for each interpolation
        for i in range(interpolations):
            # Add the increment to the start pose. It is i+1 in order to start the loop increment at 1 instead of 0
            trajectory[i] = np.add(start_pose, np.multiply(increment, i + 1))

        return trajectory

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

        # self.get_logger().info(f"Euler angles: {euler}")
        return euler

    def generate_joint_trajectory(self, target_joint_angles, gain=100):
        start_joints = self.joint_angles

        angular_displacement = np.sum(np.abs(np.subtract(target_joint_angles, start_joints)))
        interpolations = int(angular_displacement*gain)

        increment = np.subtract(target_joint_angles, start_joints) * (1/interpolations)

        trajectory = np.zeros([interpolations, 7])

        for i in range(interpolations):
            trajectory[i] = np.add(start_joints, np.multiply(increment, i + 1))

        self.get_logger().info(f"Trajectory shape: {np.shape(trajectory)}")

        return trajectory
def main(args: list = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(5)
    CartesianMotionNode = PoseCommander("pose_commander")
    executor.add_node(CartesianMotionNode)
    try:
        executor.spin()
    finally:
        # Cleanup code, if any
        CartesianMotionNode.destroy_node()
        rclpy.shutdown()


    rclpy.init(args=args)
    rclpy.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
