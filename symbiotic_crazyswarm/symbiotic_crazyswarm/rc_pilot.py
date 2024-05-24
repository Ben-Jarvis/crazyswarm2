# Use pygame to read joystick axes and buttons and publish them as Twist messages

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from pynput import keyboard

import pygame

# Initialize pygame for joystick support
pygame.display.init()
pygame.joystick.init()
controller = pygame.joystick.Joystick(0)
controller.init()

class RCPilot(Node):
    
        def __init__(self):
            super().__init__('rc_pilot')
            self.__publisher = self.create_publisher(Twist, '/cmd_vel', 1)
            self.timer = self.create_timer(0.01, self.timer_callback)

            # # Create a client to trigger the save image service
            # self.__save_image_client = self.create_client(Trigger, '/save_image')

            # # Wait for the save image service to be ready
            # while not self.__save_image_client.wait_for_service(timeout_sec=1.0):
            #     self.get_logger().info('Save image service not available, waiting again...')
            # self.get_logger().info('Save image service available')

            # Initialize params
            self.button_press = False
            self.separation_range = 0.3
            self.center_ang_x = 1.2
            self.current_ang_x = self.center_ang_x
            self.next_ang_x = self.center_ang_x
            self.alpha = 0.01


            # start the keyboard listener
            listener = keyboard.Listener(on_press=self.on_press)
            listener.start()
            self.key_step = 0.2
    
        # Apply a first order filter to the switch input
        def filter(self, input, prev_output):
             return self.alpha*input + (1-self.alpha)*prev_output
        
        
        def on_press(self, key):
            if key == keyboard.Key.up:
                self.next_ang_x = min(self.next_ang_x + self.key_step, 1.5)
            elif key == keyboard.Key.down:
                self.next_ang_x = max(self.current_ang_x - self.key_step, 0.9)

        
        def timer_callback(self):
            msg = Twist()
    
            # Get next pygame event
            pygame.event.pump()
    
            # Set the linear and angular velocities
            msg.linear.x = controller.get_axis(2)               #[-1, 1]
            msg.linear.y = -1 * controller.get_axis(1)               #[-1, 1]
            msg.linear.z = 0.6 * controller.get_axis(0) * abs(controller.get_axis(0))       #[-0.5, 0.5]
            msg.angular.x = self.filter((self.center_ang_x + controller.get_axis(5)*self.separation_range), self.current_ang_x)          #[0.6, 1.4]
            # msg.angular.x = self.filter(self.next_ang_x, self.current_ang_x)          #[0.9, 1.5]
            msg.angular.y = 0.0
            msg.angular.z = -controller.get_axis(3)          #[-1, 1]
            
            # Update the switch value
            self.current_ang_x = msg.angular.x
    
            # Publish the message
            self.__publisher.publish(msg)

            # # Save an image if the button is pressed
            # if controller.get_button(3) and self.button_press == False:
            #     self.get_logger().info('Saving images...')
            #     self.button_press = True
            #     request = Trigger.Request()
            #     self.__save_image_client.call_async(request)
            #     self.get_logger().info('Images saved!')

            # elif not controller.get_button(3) and self.button_press == True:
            #     self.button_press = False
             

def main(args=None):
    rclpy.init(args=args)
    rc_pilot = RCPilot()
    rclpy.spin(rc_pilot)
    rc_pilot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()