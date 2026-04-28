#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from ball_finder.msg import BallLocation


class Robot(Node):
    def __init__(self):
        super().__init__('ballfinder')
        self.bridge = CvBridge()
        self.raw_image = []
        self.ranges = []
        self.distance_history = []
        self.buffer_size = 8 #store most recent measurements

        self.loc_publisher = self.create_publisher(BallLocation, '/ball_location', 10) #(type, topic, queue)
        self.im_publisher = self.create_publisher(Image, '/ball_image', 10) # publisher for modified image
        self.timer = self.create_timer(0.5, self.main_loop)

        self.create_subscription( #get image data from robot's camera
            Image,
            '/oakd/rgb/preview/image_raw',
            self.handle_image,
            qos_profile_sensor_data,
        )

        self.create_subscription( #get scan data from robot's lidar
            LaserScan,
            '/scan',
            self.handle_scan,
            qos_profile_sensor_data,
        )

    def handle_image(self, msg): #just stores raw image in self.raw_image by converting to OpenCV.
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            print("Unable to convert ROS image to OpenCV format.")

    def handle_scan(self, msg): 
        self.scan_msg = msg # store msg in scan_msg
        self.ranges = msg.ranges # store ranges in self.ranges for use in main loop

    def main_loop(self):
        if len(self.raw_image) == 0 or len(self.ranges) == 0: #if nothing received, dont run.
            return

        image = self.raw_image.copy() #get image, use a copy to not modify original
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert image to hsv for filtering

        lower_yellow = np.array([20, 100, 100]) #set bounds for yellow color in hsv
        upper_yellow = np.array([30, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow) #mask (binaryImage, lowBound, upBound), returns binary image where pixels in range are 255 and others are 0
        hfov = 80

        height, width = mask.shape
        mask[0:int(0.2*height), :] = 0 #ignore top 20% of image to avoid ceiling

        yellow_cols = np.where(mask == 255)[1] #get column indices of yellow pixels,
        ball_location = BallLocation() #ball_location = custom message of type BallLocation

        if len(yellow_cols) == 0: #if no pixels found, set ball location to invalid values
            ball_location.bearing = -1
            ball_location.distance = -1.0        
        else: #if pixels found...
            avg_x = int(np.mean(yellow_cols)) #calculate average column index of yellow pixels... this is the "center" of the ball in the image
            center_x = width /2
            angle_offset = (center_x - avg_x) * (hfov / width) #calculate angle offset from center of image based on position of ball

            ball_location.bearing = avg_x #convert to bearing

            scan_index = int(218 - avg_x*74/250)
            scan_index = max(220, min(scan_index, 335))

            distance = self.ranges[scan_index] #store distance for validity check

            if np.isnan(distance) or np.isinf(distance): #ensure distance is valid number, if not set to -1.0 to indicate invalid distance
                ball_location.distance = -1.0
            else:
                self.distance_history.append(float(distance)) #add distance to history
                if len(self.distance_history) > self.buffer_size: #if history exceeds buffer size, remove oldest measurement
                    self.distance_history.pop(0)
                ball_location.distance = np.mean(self.distance_history) #set distance to average of history for

            cv2.line(image, (avg_x, 0), (avg_x, height), (255, 0, 0), 2) #draw vertical blue line at avg_x to show ball center

        image[mask>0] = (0, 255, 0) #set yellow pixels in original image to bright yellow for visualization

        try:
            self.im_publisher.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8')) #publish modified image to ROS topic
        except CvBridgeError:
            print('Unable to convert mask to ROS image')
        self.loc_publisher.publish(ball_location)


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()