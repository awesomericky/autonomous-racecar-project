#!/usr/bin/env python

import numpy as np

#ROS imports
import rospy
from nav_msgs.msg import Odometry
from racecar.msg import IT_MPC_reference_data

pub_ITdata = rospy.Publisher("it_mpc_reference_data", IT_MPC_reference_data, queue_size=1)

def odom_callback(odom_msg):
    ITdata_msg = IT_MPC_reference_data()
    ITdata_msg.x_pose = odom_msg.pose.pose.position.x
    ITdata_msg.y_pose = odom_msg.pose.pose.position.y
    #ITdata_msg.theta = 
    ITdata_msg.x_vel = odom_msg.twist.twist.linear.x
    ITdata_msg.y_vel = odom_msg.twist.twist.linear.y
    ITdata_msg.theta_vel = odom_msg.twist.twist.angular.z
    pub_ITdata.publish(ITdata_msg)


def main():
    rospy.init_node("IT_MPC_reference_data_node", anonymous=True)
    sub_odom = rospy.Subscriber("/pf/pose/odom", Odometry, odom_callback, queue_size=1)
    rospy.sleep(0.5)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Operation finished")