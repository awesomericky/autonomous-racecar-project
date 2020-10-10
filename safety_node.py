#!/usr/bin/env python

# TODO: import ROS msg types and libraries
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math


class Safety:
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        """
        The publisher should publish to the /drive topic with a AckermannDriveStamped brake message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.TTC_start = 0.2  ## TTC starts when the racecar is about to collide with the obstacle after 0.2 [s]

        # TODO: create ROS subscribers and publishers.
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1, buff_size=1*856)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.pub_drive = rospy.Publisher('drive', AckermannDriveStamped, queue_size=1)

        # TODO: set the initial velocity of the car
        self.ack_msg = AckermannDriveStamped()
        self.rate = rospy.Rate(500)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: filter LaserScan.ranges
        ranges_filter = np.array([],dtype=np.float32)
        ranges_filter = scan_msg.ranges
        rangeNAN = np.isnan(ranges_filter)
        rangeinf = np.isinf(ranges_filter)
        self.TTC = np.array([],dtype=np.float32)

        for i in range(len(ranges_filter)):
            if rangeNAN[i] == True:
                ## NAN place is 0
                ranges_filter[i] = 0
                self.TTC = np.append(self.TTC, [101])

            elif rangeinf[i] == True:
                ## inf place is 1
                ranges_filter[i] = 1
                self.TTC = np.append(self.TTC, [102])

            else:
                # TODO: calculate TTC
                theta = scan_msg.angle_min + i*scan_msg.angle_increment
                rel_vel = self.speed * math.cos(theta)

                if max([rel_vel, 0]) != 0:
                    ## car is running close to obstacle
                    self.TTC = np.append(self.TTC, [ranges_filter[i]/max([rel_vel, 0])])

                else:
                    ## car is running away from obstacle
                    self.TTC = np.append(self.TTC, [100])
        
        self.TTC_decision(self.TTC)

    def TTC_decision(self, TTC):
        # TODO: publish brake message
        #rospy.loginfo(TTC) ## printing TTC causes time delay
        #rospy.loginfo(np.min(TTC))

        if np.min(TTC) < self.TTC_start:
            while not rospy.is_shutdown():
                self.ack_msg.drive.speed = 0
                self.pub_drive.publish(self.ack_msg)
        else:
            self.ack_msg.drive.speed = 4  ## initial speed of racecar is set to 4 [m/s]
            self.pub_drive.publish(self.ack_msg)
            self.rate.sleep()

def main():
    rospy.init_node('safety_node', anonymous=True)
    Safety()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
