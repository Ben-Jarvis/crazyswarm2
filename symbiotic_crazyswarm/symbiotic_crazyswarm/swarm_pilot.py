import numpy as np
import functools

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# obstacle positions
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from symbiotic_crazyswarm.viewing_metrics import compute_neihgborhood, get_viewing_dir


class SwarmPilot(Node):
    def __init__(self):
        super().__init__('swarm_pilot')

        self.declare_parameter('robot_prefixes', ['/cf1', '/cf2'])
        # self.declare_parameter('obstacle_positions', [])

        robot_prefixes = self.get_parameter('robot_prefixes').value
        # obstacle_positions = self.get_parameter('obstacle_positions').value

        ### TODO: Add as parameter in launch file

           # construct obstacles
        obstacles_yaml = os.path.join(get_package_share_directory('crazyflie'),'config','obstacles.yaml')
        with open(obstacles_yaml, 'r') as ymlfile:
            obstacles = yaml.safe_load(ymlfile)

        self.obstacle_positions = []
        if obstacles['obstacles'] is not None:
            for obstacle in obstacles['obstacles']:
                self.obstacle_positions.append(np.array(obstacles['obstacles'][obstacle]['position']))
                self.cylinder_radius = obstacles['obstacles'][obstacle]['radius']

        ### END TODO
        
        # Reading parameters for neighbors selection and viewing metric
        drone_params_path = os.path.join(get_package_share_directory('crazyflie'),'config','drone_metrics.yaml')
        with open(drone_params_path, 'r') as ymlfile:
            drone_params = yaml.safe_load(ymlfile)
        
        # Save drone params to class variables
        self.neighbors_params = drone_params['neighbors']
        self.viewing_params = drone_params['viewing']

        self.num_drones = len(robot_prefixes)
        self.num_obs = len(self.obstacle_positions)

        self.onboard_poses = [None] * self.num_drones

        # Create a publisher and subscriber for each drone to send the swarm commands and get the onboard odometry
        self.swarm_command_publishers = []
        for idx, robot_prefix in enumerate(robot_prefixes):
            self.swarm_command_publishers.append(self.create_publisher(Twist, robot_prefix + '/cmd_swarm', 1))
            self.create_subscription(Odometry, robot_prefix + '/odom', functools.partial(self.onboard_odom_callback, idx=idx), 1)
            
        # Create a subscriber to get the target separation from the cmd_vel topic
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        
        # Create a boolean array to check if all drones have received their onboard odometry
        self.onboard_odom_received = [False] * self.num_drones

        # Create a timer for the swarm control
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.step)

        self.target_separation = 1.0
        self.cmd_vel = Twist()
        self.count = 0

    # Get the target separationh from the cmd_vel topic
    def cmd_vel_callback(self, msg):

        self.cmd_vel = msg

        self.target_separation = msg.angular.x
 
    # Get the drone poses from the onboard odometry
    def onboard_odom_callback(self, msg, idx):

        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        euler = np.array(self.quaternion_to_euler(msg.pose.pose.orientation))
        lin_vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        ang_vel = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        
        drone_pose = [position, lin_vel, euler, ang_vel]
        self.onboard_poses[idx] = drone_pose
        self.onboard_odom_received[idx] = True

    # Rotate the control commands from the global refernce frame to the drone reference frame
    def rot_global2body(self, control_commands, yaw):
        ctrl_x = control_commands[0] * np.cos(yaw) + control_commands[1] * np.sin(yaw)
        ctrl_y = -control_commands[0] * np.sin(yaw) + control_commands[1] * np.cos(yaw)
        ctrl_z = control_commands[2]
        
        return np.array([ctrl_x, ctrl_y, ctrl_z])
     
    # Rotate the control commands from the drone refernce frame to the global reference frame
    def rot_body2global(self, control_commands, yaw):
        yaw = -yaw
        return self.rot_global2body(control_commands, yaw)
  
    # Convert quaternions to euler angles
    def quaternion_to_euler(self, orientation):
        
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
    
    # Calculate the cohesion intensity for the Olfati-Saber model
    def get_cohesion_intensity(self, r, d_ref, a, b, c):
        
        diff = r - d_ref
        return ((a+b)/2 * (np.sqrt(1+(diff + c)**2) - np.sqrt(1+c**2)) + (a-b)*diff/2)
    
    # Calculate the cohesion intensity derivative for the Olfati-Saber model
    def get_cohesion_intensity_der(self, r, d_ref, a, b, c):
            
        diff = r - d_ref
        return (a+b)/2 * (diff + c) / np.sqrt(1+(diff + c)**2) + (a-b)/2

    # Calcualte the neighbour weight for the Olfati-Saber model
    def get_neighbour_weight(self, r, r0, delta):

        r_ratio = r / r0

        if r_ratio < delta:
            return 1
        elif r_ratio < 1:
            return 0.25 * (1 + np.cos(np.pi * (r_ratio - delta) / (1 - delta)))**2  #with k=2
        else:
            return 0
    
    # Calcualte the derivative of the neighbour weight for the Olfati-Saber model
    def get_neighbour_weight_der(self, r, r0, delta):
        
        r_ratio = r/r0

        if r_ratio < delta:
            return 0
        elif r_ratio < 1:
            arg = np.pi * (r_ratio - delta) / (1 - delta)
            return 1/2*(-np.pi/(1-delta))*(1+np.cos(arg))*np.sin(arg)
        else:
            return 0
    
    # Calculate the attraction/repulsion force for the Olfati-Saber model
    def get_cohesion_force(self, r, d_ref, a, b, c, r0, delta):
        
        return 1/r0 * self.get_neighbour_weight_der(r, r0, delta) * self.get_cohesion_intensity(r, d_ref, a, b, c) + self.get_neighbour_weight(r, r0, delta) * self.get_cohesion_intensity_der(r, d_ref, a, b, c)
     
     
    # Compute the angular velocity command to send to each drone using the desired viewing metric
    def angular_input(self, drone_id, drone_pose, neighbour_poses):

        # Send the drone towards the average yaw of it's neighbours
        yaw_rate_coeff = 0.2
        
        # Initialize the angular velocity command
        angular_command = np.zeros(3)
        
        if len(neighbour_poses) > 0:
            # Apply noise to all other members
            
            # Get noisy neighborhood positions + index in swarm
            neighbors = compute_neihgborhood(drone_id, drone_pose[0], np.array([neighbour_pose[0] for neighbour_pose in neighbour_poses]), **self.neighbors_params)
            # Publish neighbor count
            # msg = NeighborsInfo()
            # msg.count = len(neighbors)
            # self.__drone_neighbors_publishers[drone_id].publish(msg)
            
            # Get estimated viewing direction
            if len(neighbors) == 0:
                viewing_dir = None
            else:
                n_pos = np.array([n[1] for n in neighbors])
                is_2d = np.std(np.concatenate([[drone_pose[0,2]], n_pos[:,2].flatten()])) < 0.01
                #print(f"IS_2D: {is_2d}")
                viewing_dir = get_viewing_dir(drone_pose[0], n_pos, **self.viewing_params, in_2d=is_2d)
            
            # Calculate yaw difference
            yaw_diff = 0
            if viewing_dir is not None:
                yaw_des = np.arctan2(viewing_dir[1], viewing_dir[0])
                drone_yaw = drone_pose[2][2]
                yaw_diff = yaw_des - drone_yaw
            
            # # Compute the yaw_command
            angular_command[2] = np.clip(yaw_diff * yaw_rate_coeff, -0.25, 0.25)
    
        # Return the angular velocity command
        return angular_command
     
    # Compute the olfati-saber swarm commands
    def olfati_saber_input(self, drone_pose, neighbour_poses, cylinder_poses):
        
        self.count += 1
        drone_pos = drone_pose[0]
        drone_vel = drone_pose[1]

        # Get the neighbour positions for neighbours above the ground
        neighbour_positions = [neighbour_pose[0] for neighbour_pose in neighbour_poses if neighbour_pose[0][2] > 0.01]
        num_neighbours = len(neighbour_poses)

        # Define constants
        d_ref = self.target_separation
        d_ref_obs = 1.0

        r0_coh = 10              # Could use this perception radius to limit the number of neighbours
        delta = 0.1

        a = 0.3                 # 0.3
        b = 0.5                 # 0.5
        c = (b - a)/(2*np.sqrt(a*b))

        c_vm = 1.5       #1.0          # Coefficient of velocity matching

        r0_obs = 0.65    #0.6              # Radius of obstacle avoidance
        lambda_obs = 1              # (0,1]
        c_pm_obs = 4.5    #4.3            # Coefficient of obstacle avoidance
        c_vm_obs = 0             # Coefficient of velocity matching

        # Get the refence velocity and direction from the cmd_vel topic and rotate to the global reference frame
        v_ref = [self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z]   #self.__cmd_vel.linear.z] could change max vel here
        # v_ref = self.rot_body2global(v_ref, drone_pose[2][2])
        

        # Normalize the reference velocity
        if np.linalg.norm(v_ref) > 0:
            v_ref_u = v_ref / np.linalg.norm(v_ref)
        else:
            v_ref_u = v_ref


        # Compute the velocity matching force
        acc_vel = c_vm * (v_ref - drone_vel)

        # Initialize the cohesion command
        acc_coh = np.zeros(3)

        # Compute the cohesion force for each neighbour
        if num_neighbours > 0:
            for neighbour_pos in neighbour_positions:
                
                # Get relative position and distance
                pos_rel = neighbour_pos - drone_pos
                dist = np.linalg.norm(pos_rel)

                # Compute the cohesion force
                acc_coh += self.get_cohesion_force(dist, d_ref, a, b, c, r0_coh, delta) * pos_rel / dist
                
            # Rotate the cohesion force to the body reference frame
            acc_coh = self.rot_global2body(acc_coh, drone_pose[2][2])

        # Initialize the obstacle avoidance commands
        acc_obs = np.zeros(3)

        # Compute the obstacle avoidance commands
        for i in range(self.num_obs):
            
            cylinder_poses[i][2] = drone_pos[2]

            pos_rel = drone_pos - cylinder_poses[i]
            dist = np.linalg.norm(pos_rel) - self.cylinder_radius
            
            if dist < r0_obs:

                # s in range (0,1]
                s = self.cylinder_radius / (dist + self.cylinder_radius)
                pos_obs = s*drone_pos + (1-s)*cylinder_poses[i]

                # Derivative of s
                s_der = self.cylinder_radius * (drone_vel * (pos_obs - drone_pos) / dist) / (self.cylinder_radius + dist)**2
                vel_obs = s * drone_vel - self.cylinder_radius * (s_der/s) * (pos_obs-drone_pos)/dist
                pos_gamma = cylinder_poses[i] + lambda_obs * v_ref_u
                d_ag = np.linalg.norm(pos_gamma - pos_obs)

                acc_obs += c_pm_obs * self.get_neighbour_weight(dist/r0_obs, r0_coh, delta) * (self.get_cohesion_force(dist, d_ref_obs, a, b, c, r0_coh, delta)*(pos_obs - drone_pos)/dist +
                                      self.get_cohesion_force(d_ag, d_ref_obs, a, b, c, r0_coh, delta)*(pos_gamma - drone_pos)/(np.linalg.norm(pos_gamma - drone_pos))) + c_vm_obs * (vel_obs - drone_vel)

        # Rotate the obstacle avoidance force to the body reference frame
        acc_obs = self.rot_global2body(acc_obs, drone_pose[2][2])

        # Remove the z component of the cohesion and obstacle command
        acc_coh[2] = 0
        acc_obs[2] = 0        
        acc_command = acc_vel + acc_coh + acc_obs

        velocity_command = acc_command * self.dt

        return velocity_command

    # Create the step function to send the swarm commands
    def step(self):
       
        drone_poses = self.onboard_poses

        cylinder_poses = self.obstacle_positions

        # Check if all drones have received their onboard odometry
        if all(self.onboard_odom_received):

            # compute the swarm commands for each drone
            for i in range(self.num_drones):

                drone_pose = drone_poses[i]
                neighbour_poses = np.delete(drone_poses, i, axis=0)

                # Skip if the altitude is on the ground
                if drone_pose[0][2] < 0.1:
                    continue

                # Remove any neighbours that are on the ground
                neighbour_poses = [neighbour_pose for neighbour_pose in neighbour_poses if neighbour_pose[0][2] > 0.1]
                
                # Compute the swarm commands
                swarm_command = self.olfati_saber_input(drone_pose, neighbour_poses, cylinder_poses)  
                
                # Compute the angular velocity command
                angular_command = self.angular_input(i, np.array(drone_pose), np.array(neighbour_poses))

                # Publish the swarm commands
                msg = Twist()
                msg.linear.x = swarm_command[0]
                msg.linear.y = swarm_command[1]
                msg.linear.z = swarm_command[2]
                msg.angular.z = angular_command[2] #self.cmd_vel.angular.z
                self.swarm_command_publishers[i].publish(msg)          



def main(args=None):
    rclpy.init(args=args)
    swarm_pilot = SwarmPilot()
    rclpy.spin(swarm_pilot)
    swarm_pilot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()