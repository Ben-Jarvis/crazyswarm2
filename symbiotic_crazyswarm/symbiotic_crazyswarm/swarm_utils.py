#!/usr/bin/env python

from pathlib import Path

import os
import yaml
import numpy as np
import functools
from math import isnan

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from rclpy.timer import Timer
from ament_index_python.packages import get_package_share_directory

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import Hover, AssignedPoses, StreamingIds
from nav_msgs.msg import Odometry



# A class to sort the positions of the drones along the x axis and print the poses
class SwarmUtils(Node):
    def __init__(self):
        super().__init__('swarm_utils')
        self.get_logger().info("Initializing SortPos")

        print('====================================== START ===========================================')
        self.declare_parameter('robot_prefixes', ['/cf1', '/cf2'])
        robot_prefixes = self.get_parameter('robot_prefixes').value
        print(robot_prefixes)
        print('====================================== FINISH ===========================================')
        
        for idx, robot_prefix in enumerate(robot_prefixes):
            self.create_subscription(Odometry, robot_prefix + '/odom', functools.partial(self.onboard_odom_callback, idx=idx), 1)

        self.camera_drones = [0,1,2,3]  #TODO: get from yaml file
        
        self.assigned_poses = AssignedPoses()
        self.assigned_poses_pub = self.create_publisher(AssignedPoses, '/assigned_poses', 1)

        self.streaming_ids = [False for _ in self.camera_drones]
        self.streaming_ids_sub = self.create_subscription(StreamingIds, '/streaming_ids', self.streaming_ids_callback, 10)
        
        # Create a timer for the step function
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.step)

        # Initialize the latest pose to None
        self.latest_pose = None
        self.latest_hover = None
        self.onboard_poses = [None] * len(robot_prefixes)

    # Callback for the streaming ids
    def streaming_ids_callback(self, msg):
        self.streaming_ids = msg.streaming_ids
    
    # Extract the pose and velocities from the onboard odometry
    def onboard_odom_callback(self, msg, idx):
        
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        euler = self.quaternion_to_euler(msg.pose.pose.orientation)
        lin_vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        ang_vel = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        
        drone_pose = (position, lin_vel, euler, ang_vel)
        self.onboard_poses[idx] = drone_pose

    # Convert quaternions to euler angles
    def quaternion_to_euler(self, orientation):
        """
        Converts quaternions to euler angles
        """
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.where(t2>+1.0,+1.0,t2)
        t2 = np.where(t2<-1.0,-1.0,t2)
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z
    
        # Get the average yaw of every drone that isn't upside down or too far away from the swarm
    
    # Get the average position of swarm
    def get_average_pos(self):
        # Initialize
        avg_pos = np.array([0.0, 0.0, 0.0])
        num_upright_drones = 0
        # dist_tolerance = 3 * self.target_separation

        # Get the average position of the swarm
        for drone_pose in self.onboard_poses:
            if drone_pose is None:
                continue
            # Check if the drone is off the ground
            if drone_pose[0][2] < 0.1:
                continue

            # # Check if the drone is too far away from the previous average
            # if np.linalg.norm(drone_pose[0] - self.swarm_prev_pos) > dist_tolerance:
            #     continue
            # # Check if the drone is upside down
            # if np.abs(drone_pose[2][0]) > np.pi/4 or np.abs(drone_pose[2][1]) > np.pi/4:
            #     continue
            # # Check if the drone is spinning out of control
            # if not self.check_safe_spin(drone_pose):
            #     continue
            # Add the drone position to the average
            # self.get_logger().info('Drone Pose[0]: type: {} value: {}'.format(type(drone_pose[0]), drone_pose[0]))
            # self.get_logger().info('avg_pos: type: {} value: {}\n'.format(type(avg_pos), avg_pos))
            avg_pos += drone_pose[0]
            num_upright_drones += 1

        # Divide by the number of drones
        if num_upright_drones != 0:
            avg_pos /= num_upright_drones

        return avg_pos

    
    # Get the average yaw of every drone in the swarm
    def get_average_yaw(self):

        # Initialize
        avg_yaw = 0.0
        num_upright_drones = 0
        # dist_tolerance = 3 * self.target_separation


        # Convert the drone yaws to a unit vector
        unit_vecs = np.array([(np.cos(drone_pose[2][2]), np.sin(drone_pose[2][2])) for drone_pose in self.onboard_poses if drone_pose is not None])

        avg_unit_vec = 0

        for idx, drone_pose in enumerate(self.onboard_poses):

            if drone_pose is None:
                continue

            # Check if the drone is off the ground
            # if drone_pose[0][2] < 0.1:
            #     continue
        
            # # Check if the drone is too far away from the previous average
            # if np.linalg.norm(drone_pose[0] - self.swarm_prev_pos) > dist_tolerance:
            #     continue
            
            # # Check if the drone is upside down
            # if np.abs(drone_pose[2][0]) > np.pi/4 or np.abs(drone_pose[2][1]) > np.pi/4:
            #     continue

            # # Check if the drone is spinning out of control
            # if not self.check_safe_spin(drone_pose):
            #     continue

            # Add the drone position to the average
            avg_unit_vec += unit_vecs[idx]
            num_upright_drones += 1

        # Divide by the number of drones
        if num_upright_drones != 0:
            avg_unit_vec /= num_upright_drones

        # Get the average yaw from the unit vector
        if num_upright_drones == 0:
            avg_unit_vec = np.array([0, 0])
        elif len(avg_unit_vec) > 0:    
            avg_yaw = np.arctan2(avg_unit_vec[1], avg_unit_vec[0])
        else:
            avg_yaw = 0.0

        # Make sure the average yaw is between -pi and pi
        if avg_yaw > np.pi:
            avg_yaw -= 2*np.pi
        elif avg_yaw < -np.pi:
            avg_yaw += 2*np.pi
        
        # self.get_logger().info('Average Yaw: type: {} value: {}'.format(type(avg_yaw), avg_yaw))

        self.assigned_poses.avg_yaw = avg_yaw

        return avg_yaw
    
    # Assign the drones to their positions in the swarm
    def assign_pos_simple(self, avg_pos, avg_yaw):

        # Get the swarm line of sight
        swarm_los = [np.cos(avg_yaw), np.sin(avg_yaw), 0]

        # Get the corrs product of the line of sight and the z-axis
        swarm_los_cross_z = np.cross(swarm_los, [0, 0, 1])

        # Get the relative position of each drone with resepect to the swarm
        try:
            drones_pos_rel = [[x - y for x, y in zip(drone_pose[0], avg_pos)] for drone_pose in self.onboard_poses]
        except:
            return
        
        # self.get_logger().info('drone_poses: {}'.format(self.onboard_poses))
        # self.get_logger().info('avg_pos: {}'.format(avg_pos))
        # self.get_logger().info('drones_pos_rel: {}'.format(drones_pos_rel))
        
        # Find the distance along the line of sight
        drone_pos_rel_dots = [np.dot(swarm_los, drone_pos_rel) for drone_pos_rel in drones_pos_rel]

        # Find the distance perpendicular to the line of sight
        drone_pos_rel_cross_z_dots = [np.dot(swarm_los_cross_z, drone_pos_rel) for drone_pos_rel in drones_pos_rel]

        # # log the drone_pos_rel_dots
        # self.get_logger().info('drone_pos_rel_dots: {}'.format(drone_pos_rel_dots))
        # self.get_logger().info('drone_pos_rel_cross_z_dots: {}'.format(drone_pos_rel_cross_z_dots))
        
        # Get list of valid drones
        valid_drone_indices = [idx for idx, _ in enumerate(self.onboard_poses)] #TODO: if self.swarm_state.drone_status[idx]]
        
        # # log the valid_drone_indices
        # self.get_logger().info('valid_drone_indices: {}'.format(valid_drone_indices))
        
        # Get list of valid camera drones
        cam_drone_indices = self.camera_drones        #TODO
        
        # # log the cam_drone_indices
        # self.get_logger().info('cam_drone_indices: {}'.format(cam_drone_indices))
        
        # Remove elements from cam_drone_indices if the value is false in streaming_ids
        cam_drone_indices = [idx for i, idx in enumerate(cam_drone_indices) if self.streaming_ids[i]]
        
        # # log the cam_drone_indices
        # self.get_logger().info('cam_drone_indices: {}'.format(cam_drone_indices))
        # self.get_logger().info('\n')
        
        # Publish the number of valid drones
        self.assigned_poses.num_valid = len(cam_drone_indices)       #len(valid_drone_indices) #TODO

        if len(valid_drone_indices) == 0 or len(cam_drone_indices) == 0:
            return
        
        # Find the backmost drone
        # back_idx = min(valid_drone_indices, key=lambda idx: drone_pos_rel_dots[idx])    # back_idx = drone_pos_rel_dots.index(min(drone_pos_rel_dots))
        # Find the backmost drone that is also a camera drone
        back_idx = min((idx for idx in valid_drone_indices if idx in cam_drone_indices), key=lambda idx: drone_pos_rel_dots[idx])
        drone_pos_rel_dots[back_idx] = 0
        drone_pos_rel_cross_z_dots[back_idx] = 0
        valid_drone_indices.remove(back_idx)
        cam_drone_indices.remove(back_idx)
        self.assigned_poses.back_idx = back_idx

        if len(valid_drone_indices) == 0 or len(cam_drone_indices) == 0:
            return
        
        # Find the frontmost drone
        front_idx = max(valid_drone_indices, key=lambda idx: drone_pos_rel_dots[idx])   # front_idx = drone_pos_rel_dots.index(max(drone_pos_rel_dots))
        front_idx = max((idx for idx in valid_drone_indices if idx in cam_drone_indices), key=lambda idx: drone_pos_rel_dots[idx])
        drone_pos_rel_dots[front_idx] = 0
        drone_pos_rel_cross_z_dots[front_idx] = 0
        valid_drone_indices.remove(front_idx)
        cam_drone_indices.remove(front_idx)
        self.assigned_poses.front_idx = front_idx

        if len(valid_drone_indices) == 0 or len(cam_drone_indices) == 0:
            return

        # Find the leftmost drone
        left_idx = min((idx for idx in valid_drone_indices if idx in cam_drone_indices), key=lambda idx: drone_pos_rel_cross_z_dots[idx])    # left_idx = drone_pos_rel_cross_z_dots.index(min(drone_pos_rel_cross_z_dots))
        drone_pos_rel_dots[left_idx] = 0
        drone_pos_rel_cross_z_dots[left_idx] = 0
        valid_drone_indices.remove(left_idx)
        cam_drone_indices.remove(left_idx)
        self.assigned_poses.left_idx = left_idx

        if len(valid_drone_indices) == 0 or len(cam_drone_indices) == 0:
            return

        # Find the rightmost drone
        right_idx = max((idx for idx in valid_drone_indices if idx in cam_drone_indices), key=lambda idx: drone_pos_rel_cross_z_dots[idx])   # right_idx = drone_pos_rel_cross_z_dots.index(max(drone_pos_rel_cross_z_dots))
        self.assigned_poses.right_idx = right_idx

    # Create a step function to publish the assigned poses
    def step(self):

        # Get the average position of the swarm
        avg_pos = self.get_average_pos()
        
        # Get the average yaw of the swarm
        avg_yaw = self.get_average_yaw()

        # Assign the drones to their positions in the swarm
        self.assign_pos_simple(avg_pos, avg_yaw)

        # Publish the assigned poses
        self.assigned_poses_pub.publish(self.assigned_poses)  

def main(args=None):
    rclpy.init(args=args)
    swarm_utils = SwarmUtils()
    rclpy.spin(swarm_utils)
    swarm_utils.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()