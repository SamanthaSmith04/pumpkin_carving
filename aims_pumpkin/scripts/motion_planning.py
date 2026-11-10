#!/usr/bin/env python3

import time
import math
import numpy as np
from robodk import robolink, robomath
from robodk.robomath import Mat

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

class RoboDKPlanner():

    def __init__(self, pumpkin_face_frame=None):
        super().__init__()
    
        RDK = robolink.Robolink()
        self.robot = RDK.Item('', robolink.ITEM_TYPE_ROBOT)
        self.robot.setParam("PostProcessor", "Motoman")
        print("Creating motion plan for robot:", self.robot.Name())

        self.pumpkin_face_frame = pumpkin_face_frame

    def plan(self, pose_list):
        test_poses_array = PoseArray()
        test_poses_array.poses = pose_list
        joint_plan = self.create_motion_plan(test_poses_array)
        joint_trajectory_msg = self.convert_to_joint_trajectory_msg(joint_plan)

        for idx, point in enumerate(joint_trajectory_msg.points):
            print(f"Point {idx}:")
            print("  Positions (rad):", point.positions)
            print("  Velocities (rad/s):", point.velocities)
            print("  Accelerations (rad/s^2):", point.accelerations)
            print("  Time from start:", point.time_from_start.sec, "sec", point.time_from_start.nanosec, "nsec")

    def create_motion_plan(self, pose_list):
        """
        Create a motion plan for the robot to follow a list of poses.
        Incoming: meters (positions), quaternions (orientations).
        RoboDK internal: millimeters (translation), degrees (joint angles from SolveIK).
        Outgoing: radians (flat joint list).
        """
        joint_plan = []

        # Get current joints from RoboDK (RoboDK returns degrees)
        current_joints_deg = self.robot.Joints().tolist()
        print("Current robot position (deg):", current_joints_deg)

        for pose in pose_list.poses:
            # 1. Convert meters -> millimeters for RoboDK
            robodk_position = [x * 1000 for x in [pose.position.x, pose.position.y, pose.position.z]]  

            # 2. Convert ROS Quaternion -> Rotation Matrix
            rot_matrix = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()

            # 3. Apply pumpkin_face frame transformation if available (in millimeters)
            if self.pumpkin_face_frame is not None:
                pumpkin_face_position = [self.pumpkin_face_frame.position.x * 1000, 
                                        self.pumpkin_face_frame.position.y * 1000, 
                                        self.pumpkin_face_frame.position.z * 1000]  

                pumpkin_face_rot_matrix = R.from_quat([self.pumpkin_face_frame.orientation.x, 
                                                        self.pumpkin_face_frame.orientation.y, 
                                                        self.pumpkin_face_frame.orientation.z, 
                                                        self.pumpkin_face_frame.orientation.w]).as_matrix()

                robodk_position = [
                    pumpkin_face_rot_matrix[0][0] * robodk_position[0] + pumpkin_face_rot_matrix[0][1] * robodk_position[1] + pumpkin_face_rot_matrix[0][2] * robodk_position[2] + pumpkin_face_position[0],
                    pumpkin_face_rot_matrix[1][0] * robodk_position[0] + pumpkin_face_rot_matrix[1][1] * robodk_position[1] + pumpkin_face_rot_matrix[1][2] * robodk_position[2] + pumpkin_face_position[1],
                    pumpkin_face_rot_matrix[2][0] * robodk_position[0] + pumpkin_face_rot_matrix[2][1] * robodk_position[1] + pumpkin_face_rot_matrix[2][2] * robodk_position[2] + pumpkin_face_position[2]
                ]

            rdk_matrix = Mat([
                [rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2], robodk_position[0]],
                [rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2], robodk_position[1]],
                [rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2], robodk_position[2]],
                [0, 0, 0, 1]
            ]) 

            # SolveIK returns joint angles in DEGREES
            new_robot_joints_deg = self.robot.SolveIK(rdk_matrix)
            
            if hasattr(new_robot_joints_deg, 'tolist'):
                joints_list = new_robot_joints_deg.tolist()
            else:
                joints_list = list(new_robot_joints_deg)
                
            # Flatten and convert RoboDK DEGREES -> ROS RADIANS
            flat_joints_rad = []
            for item in joints_list:
                if isinstance(item, (list, tuple, np.ndarray)):
                    flat_joints_rad.extend([math.radians(float(x)) for x in np.atleast_1d(item).flatten()])
                else:
                    flat_joints_rad.append(math.radians(float(item)))

            print("Moving to pose:", pose)
            print("Calculated joint configuration (rad):", flat_joints_rad)
            joint_plan.append(flat_joints_rad)
            
        return joint_plan
        
    def convert_to_joint_trajectory_msg(self, joint_plan):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = [f'joint_{i+1}' for i in range(len(joint_plan[0]))]

        time_parameterization = ConstantTCPSpeedTimeParameterization(
            robot=self.robot,
            tcp_name="tool0",
            max_translational_vel=0.5,  # m/s
            max_rotational_vel=1.0,     # rad/s
            max_translational_acc=0.5,  # m/s^2
            max_rotational_acc=1.0      # rad/s^2
        )

        time_param_result = time_parameterization.compute(joint_plan)
        if not time_param_result:
            return trajectory_msg

        times = time_param_result["times"]
        velocities = time_param_result["velocities"]
        accelerations = time_param_result["accelerations"]

        for idx, joint_config in enumerate(joint_plan):
            point = JointTrajectoryPoint()
            
            flat_config = [float(j) for j in np.atleast_1d(joint_config).flatten()]
            point.positions = flat_config
            point.velocities = [float(v) for v in np.atleast_1d(velocities[idx]).flatten()]
            point.accelerations = [float(a) for a in np.atleast_1d(accelerations[idx]).flatten()]
            
            t = times[idx]
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t % 1) * 1e9)
            
            trajectory_msg.points.append(point)

        return trajectory_msg


class ConstantTCPSpeedTimeParameterization:
    def __init__(self, robot, tcp_name, max_translational_vel, max_rotational_vel, 
                 max_translational_acc, max_rotational_acc):
        self.robot = robot
        self.tcp_name = tcp_name
        self.max_translational_vel = max_translational_vel
        self.max_rotational_vel = max_rotational_vel
        self.max_translational_acc = max_translational_acc
        self.max_rotational_acc = max_rotational_acc
        
        self.eq_radius = max(
            max_translational_vel / max_rotational_vel if max_rotational_vel > 0 else 1.0,
            max_translational_acc / max_rotational_acc if max_rotational_acc > 0 else 1.0
        )

    def _calc_cartesian_distance(self, pose1, pose2):
        pos1 = pose1[:3, 3]
        pos2 = pose2[:3, 3]
        trans_dist = np.linalg.norm(pos2 - pos1)
        
        R1 = pose1[:3, :3]
        R2 = pose2[:3, :3]
        rot_diff = R.from_matrix(R1.T @ R2)
        rot_dist = np.linalg.norm(rot_diff.as_rotvec())
        
        return trans_dist + self.eq_radius * rot_dist

    def compute(self, trajectory_positions, max_velocity_scaling_factor = 1.0, 
                max_acceleration_scaling_factor = 1.0):
        n_points = len(trajectory_positions)
        if n_points == 0:
            return None

        max_vel = max_velocity_scaling_factor * self.max_translational_vel
        max_acc = max_acceleration_scaling_factor * self.max_translational_acc

        cartesian_poses = []
        segment_lengths = [0.0]
        total_path_length = 0.0

        for joints in trajectory_positions:
            # Time parameterization expects input joint positions in radians, 
            # but RoboDK's SolveFK expects degrees! We must convert radians back to degrees for RoboDK FK.
            joints_deg_list = [math.degrees(float(j)) for j in np.atleast_1d(joints).flatten()]
            
            fk_mat = np.array(self.robot.SolveFK(joints_deg_list))
            cartesian_poses.append(fk_mat)

        for i in range(1, n_points):
            dist = self._calc_cartesian_distance(cartesian_poses[i-1], cartesian_poses[i])
            segment_lengths.append(dist)
            total_path_length += dist

        if total_path_length == 0:
            total_path_length = 1e-6

        acc_dist = (max_vel ** 2) / max_acc if max_acc > 0 else 0
        
        times = [0.0]
        cumulative_dist = 0.0
        
        for i in range(1, n_points):
            cumulative_dist += segment_lengths[i]
            if total_path_length >= acc_dist * 2:
                if cumulative_dist <= acc_dist:
                    t = np.sqrt(2 * cumulative_dist / max_acc)
                elif cumulative_dist >= total_path_length - acc_dist:
                    rem = total_path_length - cumulative_dist
                    t = (total_path_length - acc_dist) / max_vel + (acc_dist - np.sqrt(max(0, 2 * rem * max_acc))) / max_acc
                else:
                    t = acc_dist / max_vel + (cumulative_dist - acc_dist) / max_vel
            else:
                peak_dist = total_path_length / 2.0
                if cumulative_dist <= peak_dist:
                    t = np.sqrt(2 * cumulative_dist / max_acc)
                else:
                    rem = total_path_length - cumulative_dist
                    t = 2 * np.sqrt(total_path_length / max_acc) - np.sqrt(max(0, 2 * rem / max_acc))
            
            times.append(max(t, times[-1] + 0.001))

        joint_velocities = []
        joint_accelerations = []

        for i in range(n_points):
            q_curr = np.array(trajectory_positions[i])
            dof = len(q_curr)
            
            if i == 0:
                dt_next = times[1] - times[0]
                q_next = np.array(trajectory_positions[1])
                vel = (q_next - q_curr) / dt_next
                joint_velocities.append(vel)
                joint_accelerations.append(np.zeros(dof))
            elif i == n_points - 1:
                dt_prev = times[i] - times[i-1]
                q_prev = np.array(trajectory_positions[i-1])
                vel = (q_curr - q_prev) / dt_prev
                joint_velocities.append(vel)
                dt_curr = times[i] - times[i-1]
                acc = (joint_velocities[i] - joint_velocities[i-1]) / dt_curr
                joint_accelerations.append(acc)
            else:
                dt_prev = times[i] - times[i-1]
                dt_next = times[i+1] - times[i]
                q_prev = np.array(trajectory_positions[i-1])
                q_next = np.array(trajectory_positions[i+1])
                
                vel = (q_next - q_prev) / (dt_prev + dt_next)
                joint_velocities.append(vel)
                
                acc = (joint_velocities[i] - joint_velocities[i-1]) / dt_prev
                joint_accelerations.append(acc)

        return {
            "times": times,
            "positions": trajectory_positions,
            "velocities": joint_velocities,
            "accelerations": joint_accelerations
        }

def _plan_motion(pose_list, pumpkin_face_frame=None):
    motion_planner_node = RoboDKPlanner(pumpkin_face_frame=pumpkin_face_frame)
    joint_plan = motion_planner_node.create_motion_plan(pose_list)
    joint_trajectory_msg = motion_planner_node.convert_to_joint_trajectory_msg(joint_plan)
    return joint_trajectory_msg


def main():
    test_poses = [
            Pose(position=Point(x=0.0, y=0.0, z=0.5), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            Pose(position=Point(x=0.6, y=0.1, z=0.3), orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)),
            Pose(position=Point(x=0.7, y=0.2, z=0.4), orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)),
        ]

    pumpkin_face_frame = Pose(
        position=Point(x=1.7, y=0.0, z=1.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    motion_planner_node = RoboDKPlanner(pumpkin_face_frame=pumpkin_face_frame)
    motion_planner_node.plan(test_poses)

if __name__ == "__main__":
    main()