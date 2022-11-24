
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import time as t
import os

class MinimalSubscriber(Node):


    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            10)
        self.subscription
        self.count = 0 
         # prevent unused variable warning
         # Create a timer that will gate the node actions twice a second
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.image_callback)
        

    # Instantiate CvBridge
   
    def image_callback(self, msg):
        bridge = CvBridge()
        self.path = '/home/iana/Atlas'
    
        #rate = Node.create_rate(self, frequency=2)
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("Error")
        else:
            # Save your OpenCV2 image as a jpeg 
            self.count += 1
            cv2.imwrite(os.path.join(self.path, f'{self.count:07}'+'.jpg'), cv2_img)
            print('Image is saved')
            t.sleep(3)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()