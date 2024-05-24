# Open a socket to multiple crazyflies and display images from their video streams

import argparse
import os
import time
import socket,os,struct, time
import numpy as np
import cv2
import threading
import queue
import yaml
import datetime

import rclpy
from rclpy.node import Node

from crazyflie_interfaces.msg import AssignedPoses, StreamingIds
from ament_index_python.packages import get_package_share_directory

import pyautogui

SCREEN_SIZE = pyautogui.size()

DRONE_IDS = [5, 6, 7, 10]

LAYOUT = 'four_back'

IMG_WIDTH = 324
IMG_HEIGHT = 244

class MultiStream(Node):
    def __init__(self):
        super().__init__('multi_stream')
        
        
        ### Get the drone ip addresses TODO
        camera_yaml = os.path.join(get_package_share_directory('crazyflie'),'config','cameras.yaml')
        with open(camera_yaml, 'r') as ymlfile:
            camera_config = yaml.safe_load(ymlfile)

        self.ip_addresses = []
        for id in DRONE_IDS:
            self.ip_addresses.append(camera_config['drones'][f'cf{id:02}']['ip'])

        # Create a publisher for the cameras that are streaming
        self.streaming_ids_publisher = self.create_publisher(StreamingIds, '/streaming_ids', 10)
        self.streaming_ids = StreamingIds()

        # Subscribe to the assigned poses
        self.subscription = self.create_subscription(AssignedPoses, '/assigned_poses', self.assigned_poses_callback, 10)
        self.assigned_poses = AssignedPoses()

        # Default values
        self.assigned_poses.front_idx = 0
        self.assigned_poses.left_idx = 1
        self.assigned_poses.back_idx = 2
        self.assigned_poses.right_idx = 3

        self.count = 0

        # Make an output directroy that is timestamped
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        os.makedirs('src/symbiotic_crazyswarm/data/saved_images/output_' + self.timestamp)
    
    # Callback for the assigned poses
    def assigned_poses_callback(self, msg):
        self.assigned_poses = msg
    
    # Connect to a socket given the drone ip and port
    def connect_to_socket(self, deck_ip, deck_port, retries=5, timeout=10):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(timeout)

        for _ in range(retries):
            try:
                print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
                client_socket.connect((deck_ip, deck_port))
            except socket.error as e:
                print("Failed to connect to socket on {}:{}. Retrying...".format(deck_ip, deck_port))
                time.sleep(1)
            else:
                print("Connected to socket on {}:{}!".format(deck_ip, deck_port))
                return client_socket

        print("Failed to connect to socket on {}:{}!".format(deck_ip, deck_port))
        return None

    # Receive bytes from a socket
    def rx_bytes(self, client_socket, size):
        data = bytearray()
        while len(data) < size:
            data.extend(client_socket.recv(size-len(data)))
        return data

    # Process the image stream from the socket
    def process_stream(self, deck_ip, deck_port, frame_queue):
        client_socket = self.connect_to_socket(deck_ip, deck_port)

        imgdata = None
        data_buffer = bytearray()
        start = time.time()
        count = 0

        while(1):
            packetInfoRaw = self.rx_bytes(client_socket, 4)
            [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
            imgHeader = self.rx_bytes(client_socket, length - 2)
            [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

            if magic == 0xBC:
                imgStream = bytearray()

                while len(imgStream) < size:
                    packetInfoRaw = self.rx_bytes(client_socket, 4)
                    [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                    chunk = self.rx_bytes(client_socket, length - 2)
                    imgStream.extend(chunk)

                if format == 0:
                    bayer_img = np.frombuffer(imgStream, dtype=np.uint8)    
                    bayer_img.shape = (height, width)
                    color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                    frame_queue.put(color_img)
                else:
                    nparr = np.frombuffer(imgStream, np.uint8)
                    decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                    frame_queue.put(decoded)


    def step(self,):

        # Deck Port
        deck_port = 5000

        # Create queues for each stream
        frame_queues = [queue.Queue() for _ in DRONE_IDS]

        # Initialize empty images for each frame
        screen_height = 1080
        screen_width = 1920
        grid = np.zeros((screen_height, screen_width), dtype=np.uint8)

        # Create the window
        window_name = 'SwarmFPV'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        # Create threads for each stream
        threads = [threading.Thread(target=self.process_stream, args=(ip, deck_port, frame_queue)) 
                for ip, frame_queue in zip(self.ip_addresses, frame_queues)]

        # Start threads
        for thread in threads:
            thread.start()

        # Create a list to see which drones are streaming
        streaming_ids = [False for _ in DRONE_IDS]

        # Create a list to count how many iterations have been missed for each drone
        missed_iterations = [0 for _ in DRONE_IDS]
        
        try:
            while True:
                
                # Get the next frame from each queue if available
                for i, frame_queue in enumerate(frame_queues):
                    if not frame_queue.empty():

                        # Set the streaming id to true and reset the missed iterations
                        streaming_ids[i] = True
                        missed_iterations[i] = 0
                        
                        if LAYOUT == 'back':
                            if self.assigned_poses.back_idx == i:
                                start_x = 474
                                start_y = 52 + int(IMG_HEIGHT/2)
                                width = 3*IMG_WIDTH
                                height = 3*IMG_HEIGHT
                            else:
                                continue
                        
                        elif LAYOUT == 'four_front':
                            if self.assigned_poses.front_idx == i:
                                start_x = 474
                                start_y = 52
                                width = 3*IMG_WIDTH
                                height = 3*IMG_HEIGHT
                            elif self.assigned_poses.left_idx == i:
                                start_x = 474
                                start_y = 784
                                width = IMG_WIDTH
                                height = IMG_HEIGHT
                            elif self.assigned_poses.back_idx == i:
                                start_x = 798
                                start_y = 784
                                width = IMG_WIDTH
                                height = IMG_HEIGHT
                            elif self.assigned_poses.right_idx == i:
                                start_x = 1122
                                start_y = 784
                                width = IMG_WIDTH
                                height = IMG_HEIGHT
                            else:
                                continue

                        elif LAYOUT == 'four_back':
                            if self.assigned_poses.back_idx == i:
                                start_x = 474
                                start_y = 296
                                width = 3*IMG_WIDTH
                                height = 3*IMG_HEIGHT
                            elif self.assigned_poses.left_idx == i:
                                start_x = 474
                                start_y = 52
                                width = IMG_WIDTH
                                height = IMG_HEIGHT
                            elif self.assigned_poses.front_idx == i:
                                start_x = 798
                                start_y = 52
                                width = IMG_WIDTH
                                height = IMG_HEIGHT
                            elif self.assigned_poses.right_idx == i:
                                start_x = 1122
                                start_y = 52
                                width = IMG_WIDTH
                                height = IMG_HEIGHT
                            else:
                                continue


                        # if i == 0 or i == 2:
                        #     imagee = cv2.rotate(frame_queue.get(), cv2.ROTATE_180)
                        #     grid[start_y:start_y+height, start_x:start_x+width] = cv2.resize(imagee, (width, height))
                        # else:
                        grid[start_y:start_y+height, start_x:start_x+width] = cv2.resize(frame_queue.get(), (width, height))

                    else:
                        # If the frame queue is empty, increment the missed iterations
                        missed_iterations[i] += 1

                        # If the missed iterations are greater than 20, set the streaming id to false
                        if missed_iterations[i] > 20:
                            streaming_ids[i] = False


                # Set all stremaing to true
                streaming_ids = [True for _ in DRONE_IDS]
                
                # Publish the streaming ids
                self.streaming_ids.streaming_ids = streaming_ids
                self.streaming_ids_publisher.publish(self.streaming_ids)

                # Write the image to a file using the count variable to 4 digits
                cv2.imwrite('src/symbiotic_crazyswarm/data/saved_images/output_{}/frame_{:04d}.png'.format(self.timestamp, self.count), grid)

                self.count += 1

                # Display the concatenated image
                cv2.imshow(window_name, grid)
                cv2.waitKey(1)

                time.sleep(0.1)

        except KeyboardInterrupt:  
            # Wait for all threads to finish
            print('finishing')
            for thread in threads:
                thread.join()

            print('finished')

def main(args=None):
    rclpy.init(args=args)
    multi_stream = MultiStream()
    multi_stream.step()
    multi_stream.destroy_node()
    multi_stream.out.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()