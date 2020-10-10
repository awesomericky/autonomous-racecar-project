#!/usr/bin/env python

import math
import numpy as np

#ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#Debugging
import pdb

discrete_limit = 0.25 #[m]
car_half_width = 0.15 #[m]
half_lidar_data = 540 

steer_angle_limit = 0.34 #[rad]
threshold = 2 #[m]

class GapFollow:
    def __init__(self):
        #Topics & Subs, Pubs
        topic_lidarscan = '/scan'
        topic_drive = '/drive'

        self.sub_lidarscan = rospy.Subscriber(topic_lidarscan, LaserScan, self.scan_callback, queue_size=1, buff_size=1*856)
        self.pub_drive = rospy.Publisher(topic_drive, AckermannDriveStamped, queue_size=1)

    def scan_callback(self, scan_msg):
        ranges_filter = np.array([],dtype=np.float32)
        ranges_filter = scan_msg.ranges
        rangeNAN = np.isnan(ranges_filter)
        rangeinf = np.isinf(ranges_filter)

        for i in range(len(ranges_filter)):
            if rangeNAN[i] == True:
                ## NAN place is 0
                ranges_filter[i] = 0

            elif rangeinf[i] == True:
                ## inf place is 1
                ranges_filter[i] = 1

            else:
                pass
        
        angle_delta = scan_msg.angle_increment
        self.find_discrete(ranges_filter, angle_delta)
    
    def find_discrete(self, ranges_filter, angle_delta):
        index_limit = int(round((math.pi/4)/angle_delta))
        lower_limit = index_limit
        upper_limit = len(ranges_filter)-index_limit
        # range[0:lower_limit]=0
        # range[upper_limit+1:len(range)]=0

        discrete_part = np.array([],dtype=int)
        min_range = 0
        for i in range(lower_limit, upper_limit):
            if ranges_filter[i]-ranges_filter[i+1] <= -discrete_limit:
                if min_range == 0 or ranges_filter[i] < min_range:
                    discrete_part = [0,i]
                    min_range = ranges_filter[i]
            elif ranges_filter[i]-ranges_filter[i+1] >= discrete_limit:
                if min_range == 0 or ranges_filter[i+1] < min_range:
                    discrete_part = [1,i+1]
                    min_range = ranges_filter[i+1]
            else:
                pass
        
        if min_range == 0:
            alpha = 0
        else:
            theta = (car_half_width+0.05)/min_range
            index_bubble = theta/angle_delta
            if discrete_part[0] == 0:
                alpha = -angle_delta*(half_lidar_data-discrete_part[1]-index_bubble)
            else:
                alpha = angle_delta*(discrete_part[1]-index_bubble-half_lidar_data)
        
        if min_range > threshold:
            alpha=0

        self.steer(alpha)

    def steer(self, alpha):
        if abs(alpha) <= steer_angle_limit:
            steer_angle = alpha
        elif alpha < 0:
            steer_angle = -steer_angle_limit
        else:
            steer_angle = steer_angle_limit
        
        print(steer_angle)
        
        if abs(steer_angle) < 0.15:
            velocity = 2
        elif abs(steer_angle) < 0.25:
            velocity = 1.5
        else:
            velocity = 1
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steer_angle
        drive_msg.drive.speed = velocity
        self.pub_drive.publish(drive_msg)

def main():
    rospy.init_node("gapfollow_node", anonymous=True)
    gf = GapFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
