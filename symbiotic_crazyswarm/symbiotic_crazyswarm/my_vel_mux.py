#!/usr/bin/env python3

"""
A Twist message handler that get incoming twist messages from 
    external packages and handles proper takeoff, landing and
    hover commands of connected crazyflie in the crazyflie_server
    node

    2022 - K. N. McGuire (Bitcraze AB)
"""
import rclpy
from rclpy.node import Node
import functools

from geometry_msgs.msg import Twist
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import Hover
import time

class VelMux(Node):
    def __init__(self):
        super().__init__('vel_mux')
        self.declare_parameter('hover_height', 0.5)
        self.declare_parameter('robot_prefixes', ['/cf01','/cf02'])
        self.declare_parameter('incoming_twist_topic', '/cmd_vel')

        self.hover_height  = self.get_parameter('hover_height').value
        self.robot_prefixes  = self.get_parameter('robot_prefixes').value
        incoming_twist_topic  = self.get_parameter('incoming_twist_topic').value

        self.num_drones = len(self.robot_prefixes)
        
        self.subscription = self.create_subscription(Twist, incoming_twist_topic, self.cmd_vel_callback, 10)
        self.msg_cmd_vel = Twist()
        self.received_first_cmd_vel = False
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.takeoff_clients = []
        self.publisher_hovers = []
        self.land_clients = []
        self.notify_clients = []
        self.swarm_command_subs = []
        self.swarm_commands = [Twist() for _ in range(self.num_drones)]
        for idx, robot_prefix in enumerate(self.robot_prefixes):
            self.takeoff_clients.append(self.create_client(Takeoff, robot_prefix + '/takeoff'))
            self.publisher_hovers.append(self.create_publisher(Hover, robot_prefix + '/cmd_hover', 1))
            self.land_clients.append(self.create_client(Land, robot_prefix + '/land'))
            self.notify_clients.append(self.create_client(NotifySetpointsStop, robot_prefix + '/notify_setpoints_stop'))
            self.swarm_command_subs.append(self.create_subscription(Twist, robot_prefix + '/cmd_swarm', functools.partial(self.cmd_swarm_callback, idx=idx), 1))

            self.get_logger().info(f"Velocity Multiplexer set for {robot_prefix}"+
                               f" with height {self.hover_height} m using the {incoming_twist_topic} topic")
            
            self.takeoff_clients[idx].wait_for_service()
            self.land_clients[idx].wait_for_service()

        self.cfs_have_taken_off = False

        self.hover_commands = [Hover() for _ in range(self.num_drones)]

        self.max_speed = 0.5
        

    def cmd_vel_callback(self, msg):
        self.msg_cmd_vel = msg
        # This is to handle the zero twist messages from teleop twist keyboard closing
        # or else the crazyflie would constantly take off again.
        msg_is_zero = msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0 and msg.linear.z == 0.0
        if  msg_is_zero is False and self.received_first_cmd_vel is False and msg.linear.z >= 0.0:
            self.received_first_cmd_vel = True

    def cmd_swarm_callback(self, msg, idx):
        self.swarm_commands[idx] = msg
    
    def timer_callback(self):

        if self.received_first_cmd_vel and self.cfs_have_taken_off is False:
            req = Takeoff.Request()
            req.height = self.hover_height
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            for takeoff_client in self.takeoff_clients:
                takeoff_client.call_async(req)
            self.cfs_have_taken_off = True
            # set the z_distance to the hover height
            for idx, hover_publisher in enumerate(self.publisher_hovers):
                self.hover_commands[idx].z_distance = self.hover_height
            time.sleep(4.0)

        if self.received_first_cmd_vel and self.cfs_have_taken_off:
            if self.msg_cmd_vel.linear.z >= -0.5:
                # msg = Hover()
                # msg.vx = self.msg_cmd_vel.linear.x
                # msg.vy = self.msg_cmd_vel.linear.y
                # msg.yaw_rate = self.msg_cmd_vel.angular.z
                # msg.z_distance = self.hover_height

                for idx, hover_publisher in enumerate(self.publisher_hovers):
                    msg = self.hover_commands[idx]
                    # msg.vx = self.msg_cmd_vel.linear.x
                    # msg.vy = self.msg_cmd_vel.linear.y
                    # msg.yaw_rate = self.msg_cmd_vel.angular.z
                    # msg.z_distance = self.hover_height
                    
                    
                    # Add the swarm commands to the hover commands
                    msg.vx += self.swarm_commands[idx].linear.x
                    msg.vy += self.swarm_commands[idx].linear.y
                    # msg.z_distance += self.swarm_commands[idx].linear.z      # TODO this is mixing velocity and position
                    msg.z_distance = self.hover_height
                    msg.yaw_rate = self.swarm_commands[idx].angular.z

                    # Cap the total speed
                    speed = (msg.vx**2 + msg.vy**2)**0.5
                    if speed > self.max_speed:
                        msg.vx *= self.max_speed/speed
                        msg.vy *= self.max_speed/speed
                   
                    hover_publisher.publish(msg)
            else:
                req = NotifySetpointsStop.Request()
                for notify_client in self.notify_clients:
                    notify_client.call_async(req)
                req = Land.Request()
                req.height = 0.1
                req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()

                for land_client in self.land_clients:
                    land_client.call_async(req)
                time.sleep(2.0)        
                self.cfs_have_taken_off = False
                self.received_first_cmd_vel = False

def main(args=None):
    rclpy.init(args=args)

    vel_mux = VelMux()

    rclpy.spin(vel_mux)

    vel_mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
