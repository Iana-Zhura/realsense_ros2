
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image as Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
# OpenCV2 for saving an image
import cv2
import time as t
import os
from vicon_receiver.msg import Position
import math

#The vicon wand center from Vicon
# x_trans = -108.7
# y_trans = -3
# z_trans = 35.7

# x = -46.8 mm
# y = -9.82
# z = -21.4

def quaternion2euler(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

class MinimalSubscriber(Node):


    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

        self.sub2 = self.create_subscription(
            Position, '/vicon/realsense/realsense', self.vicon_callback, 10)

        self.pub_euler = self.create_publisher(Pose,'/vicon_pose', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.subscription
        self.vic_pos = np.zeros([7])
        self.path = '/home/iana/Atlas/dataset'
        self.pos_list = []


        self.count = 0
        self.size_of_dataset = 250
        self.isTaskDone = False

        self.t_start = t.time()
        self.capture_interval = 1 #sec
        #  ("/vicon/realsense/realsense", Position, self.vicon_pos)
         # prevent unused variable warning self.call_vicon_pos
         # Create a timer that will gate the node actions twice a second
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.image_callback)

    def timer_callback(self):
        msg = Pose()
        msg.position.x = self.vic_pos[0]
        msg.position.y = self.vic_pos[1]
        msg.position.z = self.vic_pos[2]
        msg.orientation.x = self.vic_pos[3]
        msg.orientation.y = self.vic_pos[4]
        msg.orientation.z = self.vic_pos[5]
        msg.orientation.w = self.vic_pos[6]
        self.pub_euler.publish(msg)


    def vicon_callback(self, msg):
        self.vic_pos[0] = msg.x_trans / 1000
        self.vic_pos[1] = msg.y_trans / 1000
        self.vic_pos[2] = msg.z_trans / 1000
        # eulerAngs = quaternion2euler(msg.x_rot, msg.y_rot, msg.z_rot, msg.w) # zyx euler angs
        # self.vic_pos[3] = eulerAngs[0] #roll
        # self.vic_pos[4] = eulerAngs[1] # pitch
        # self.vic_pos[5] = eulerAngs[2] #yaw 
        

        self.vic_pos[3] = msg.x_rot 
        self.vic_pos[4] = msg.y_rot 
        self.vic_pos[5] =  msg.z_rot 
        self.vic_pos[6] =  msg.w
        # print(self.vic_pos)




    def image_callback(self, msg):
        bridge = CvBridge()
        when = self.get_clock().now().seconds_nanoseconds()
    
        #rate = Node.create_rate(self, frequency=2)
        
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("Error")
        else:
            # Save your OpenCV2 image as a jpeg
            if(self.count < self.size_of_dataset and (t.time() - self.t_start) >= self.capture_interval):  
                self.t_start = t.time()
                print("Received an image!") 
                self.count += 1
                cv2.imwrite(os.path.join(self.path, f'{self.count:08}.jpg'), cv2_img)
                # cv2.imwrite(os.path.join(self.path, f'{self.count}.jpg'), cv2_img)
                print('Image is saved')
                
                # add pose to pose_list
                print(self.count, ":  ", self.vic_pos)
                pos_data = f"{self.vic_pos}"
    
                with open(self.path + f'/poses/img_poses_{self.count}.txt', "w") as f:
                    f.write(str(pos_data))
                print("img poses were saved!")
     
            elif(self.count == self.size_of_dataset): 
                print(f"{self.count} image(s) of dataset was collected! \n")




def main(args=None):
    rclpy.init(args=args)

    sub= MinimalSubscriber()
    rclpy.spin(sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    
    sub.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
