#!/usr/bin/env python

import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

#Dynamic plotting import
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

#DEBUGGING
import pdb

#PID CONTROL PARAMS
kp = 3
ki = 0.5
kd = 1
p_input = 0
i_input = 0
d_input = 0

#WALL FOLLOW PARAMS
RANGE2_THETA = math.pi/4  #[rad]
RANGE2_TOTAL_THETA = RANGE2_THETA + math.pi/4  #[rad]
future_predict_time_interval = 0.2  #[s]
DESIRED_DISTANCE_RIGHT = 0.4  #[m]
time = np.zeros(2)  # time[0]: past time / time[1]: current time
error = np.zeros(2) # error[0]: past error / error[1]: current error

#DYNAMIC PLOTTING
style.use('ggplot')
fig = plt.figure(figsize=(13,6))
ax1 = fig.add_subplot(1,1,1)
x_time = np.array([])
y_error = np.array([])

class WallFollow:
    """ Implement Wall Following on the car
    """
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
        time[0] = time[1]
        time[1] = scan_msg.header.stamp.nsecs

        self.getDist_and_Error(ranges_filter, angle_delta, time)
    
    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def getDist_and_Error(self, data, angle_delta, time):
        ## Get scan data
        range1_step = int(round((math.pi/4)/angle_delta))
        range2_step = int(round(RANGE2_TOTAL_THETA/angle_delta))

        range1 = data[range1_step]
        range2 = data[range2_step]

        ## Calculate the current and future distance from the wall
        alpha_tan = (range2*math.cos(RANGE2_THETA) - range1)/(range2*math.sin(RANGE2_THETA))
        alpha = math.atan(alpha_tan)

        current_dis = range1*math.cos(alpha)

        if self.speed == None: #######self.speed not defined case occurs if the odometry callback is late
            default_speed = 2
            future_dis = current_dis + default_speed*future_predict_time_interval*math.sin(alpha)
        else:
            future_dis = current_dis + self.speed*future_predict_time_interval*math.sin(alpha)
        
        global error
        error[0] = error[1]
        error[1] = DESIRED_DISTANCE_RIGHT - future_dis

        global x_time
        global y_error

        if len(x_time) == 50:
            x_time = np.roll(x_time, -1)
            x_time[len(x_time)-1] = time[1]
        else:
            x_time = np.append(x_time, time[1])
        
        if len(y_error) == 50:
            y_error = np.roll(y_error, -1)
            y_error[len(y_error)-1] = error[1]
        else:
            y_error = np.append(y_error, error[1])

        self.pid_control(error, time, alpha)    

    def pid_control(self, error, time, alpha):
        global p_input
        global i_input
        global d_input

        p_input = kp*error[1]
        if time[0] != 0:
            i_input += ki*error[1]*(time[1]-time[0])*(10**(-9))
            d_input = kd*(error[1]-error[0])/(time[1]-time[0])*(10**(-9))
        total_input = p_input + i_input + d_input

        steering_angle = 0

        if time[0] != 0:
            if abs(error[1]-error[0])/(time[1]-time[0]) >= 0:
                steering_angle = total_input

        print(steering_angle)

        if 0 <= abs(steering_angle) and abs(steering_angle) < 10*(math.pi/180):
            velocity = 1.5
        elif 10*(math.pi/180) <= abs(steering_angle) and abs(steering_angle) < 20*(math.pi/180):
            velocity = 1
        else:
            velocity = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        self.pub_drive.publish(drive_msg)

    def dynamic_plotting(self, i):
        ax1.clear()
        ax1.set_xlim(min(x_time)-0.5, max(x_time)+0.5)
        ax1.plot(x_time, y_error, '-o', alpha=0.8)
        print(x_time)

def main():
    rospy.init_node("wallfollow_node", anonymous=True)
    wf = WallFollow()
    ani = animation.FuncAnimation(fig, wf.dynamic_plotting)
    plt.show()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
