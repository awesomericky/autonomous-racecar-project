#!/usr/bin/env python

import math
import numpy as np

#ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

#Debugging
import pdb

#DISCRETE DETECT PARAMS
discrete_limit = 0.5 #[m]
half_lidar_data = 540 
steer_angle_limit = 0.34 #[rad]


#PID CONTROL PARAMS
kp = 0.45
#kp=0.7
#ki = 1*(10**(-9))
ki = 0
kd = 0.02
kd = 0
p_input = 0
i_input = 0
d_input = 0

#WALL FOLLOW PARAMS
RANGE2_THETA = math.pi/6  #[rad]
RANGE2_TOTAL_THETA = RANGE2_THETA + math.pi/4  #[rad]
future_predict_time_interval = 0.8  #[s]
#future_predict_time_interval = 0.05  #[s]
DESIRED_DISTANCE = 0.4  #[m]
time = np.zeros(2)  # time[0]: past time / time[1]: current time
error = np.zeros(2) # error[0]: past error / error[1]: current error

class GapAndWallFollow:
    def __init__(self):
        #Topics & Subs, Pubs
        topic_lidarscan = '/scan'
        topic_odom = '/odom'
        topic_drive = '/drive'

        self.sub_lidarscan = rospy.Subscriber(topic_lidarscan, LaserScan, self.scan_callback, queue_size=1, buff_size=1*856)
        self.sub_odom = rospy.Subscriber(topic_odom, Odometry, self.odom_callback, queue_size=1)
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

        global time
        secs = scan_msg.header.stamp.secs
        nsecs = scan_msg.header.stamp.nsecs
        time[0] = time[1]
        time[1] = secs + nsecs*(10**(-9))

        self.find_discrete(ranges_filter, angle_delta, time)
    
    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x
    
    def find_discrete(self, rangeData, angle_delta, time):
        # index_limit = int(round((math.pi/4)/angle_delta))
        # lower_limit = index_limit
        # upper_limit = len(rangeData)-index_limit

        ## Priority of data: min_range > discrete_part
        discrete_part = True
        #discrete_part = np.array([],dtype=int)
        min_range = 0
        min_index=0
        # for i in range(len(rangeData)-1):
        #     if rangeData[i]-rangeData[i+1] <= -discrete_limit:
        #         if min_range == 0 or rangeData[i] < min_range:
        #             ## Right wall following
        #             discrete_part = True
        #             min_range = rangeData[i]
        #             a=i
        #     elif rangeData[i]-rangeData[i+1] >= discrete_limit:
        #         if min_range == 0 or rangeData[i+1] < min_range:
        #             ## Left wall following
        #             discrete_part = False
        #             min_range = rangeData[i+1]
        #             a=i
        #     else:
        #         pass

        for i in range(len(rangeData)-1):
            ## Finding discrete scan data
            if rangeData[i]-rangeData[i+1] <= -discrete_limit:
                if min_range == 0 or rangeData[i] < min_range:
                    min_range = rangeData[i]
                    min_index=i
            elif rangeData[i]-rangeData[i+1] >= discrete_limit:
                if min_range == 0 or rangeData[i+1] < min_range:
                    min_range = rangeData[i+1]
                    min_index=i+1
            else:
                pass
        
        if min_index <= 540:
            ## Right wall following
            discrete_part=True
        elif min_index > 540:
            ## Left wall following
            discrete_part=False
        else:
            pass

        #print(min_range)

        if min_range != 0:
            ## Discrete part exist
            self.getDist_and_Error(rangeData, angle_delta, time, discrete_part)
        else:
            ## Discrete part doesn't exist (move straight)
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = 0
            self.pub_drive.publish(drive_msg)
    
    def getDist_and_Error(self, rangeData, angle_delta, time, wallDirection):
        ## Get scan data
        if wallDirection:
            ## Right wall following
            range1_step = int(round((math.pi/4)/angle_delta))
            range2_step = int(round(RANGE2_TOTAL_THETA/angle_delta))
        else:
            ## Left wall following
            range1_step = len(rangeData)-int(round((math.pi/4)/angle_delta))-1
            range2_step = len(rangeData)-int(round(RANGE2_TOTAL_THETA/angle_delta))-1

        range1 = rangeData[range1_step]
        range2 = rangeData[range2_step]

        ## Calculate the current and future distance from the wall
        alpha_tan = (range2*math.cos(RANGE2_THETA) - range1)/(range2*math.sin(RANGE2_THETA))
        alpha = math.atan(alpha_tan)

        current_dis = range1*math.cos(alpha)

        if self.speed == None: 
            default_speed = 2
            future_dis = current_dis + default_speed*future_predict_time_interval*math.sin(alpha)
        else:
            future_dis = current_dis + self.speed*future_predict_time_interval*math.sin(alpha)
        
        global error
        error[0] = error[1]
        error[1] = DESIRED_DISTANCE - future_dis

        self.pid_control(error, time, wallDirection)

    def pid_control(self, error, time, wallDirection):
        global p_input
        global i_input
        global d_input

        p_input = kp*error[1]
        if time[0] != 0:
            i_input += ki*error[1]*(time[1]-time[0])
            d_input = kd*(error[1]-error[0])/(time[1]-time[0])
        total_input = p_input + i_input + d_input

        steering_angle = 0

        if time[0] != 0:
            if wallDirection:
                ## Right wall following
                if abs(total_input) <= steer_angle_limit:
                    steering_angle=total_input
                elif total_input > 0:
                    steering_angle=steer_angle_limit
                else:
                    steering_angle=-steer_angle_limit
            else:
                ## Left wall following
                if abs(total_input) <= steer_angle_limit:
                    steering_angle=-total_input
                elif total_input > 0:
                    steering_angle=-steer_angle_limit
                else:
                    steering_angle=steer_angle_limit

        print(wallDirection)

        if 0 <= abs(steering_angle) and abs(steering_angle) < 13*(math.pi/180):
            velocity = 2
        elif 13*(math.pi/180) <= abs(steering_angle) and abs(steering_angle) < 18*(math.pi/180):
            velocity = 1.8
        else:
            velocity = 1.6
        
        #print(velocity)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        # drive_msg.drive.speed = 0
        self.pub_drive.publish(drive_msg) 

def main():
    rospy.init_node("gapfollow_node", anonymous=True)
    gwf=GapAndWallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
