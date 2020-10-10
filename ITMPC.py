#!/usr/bin/env python

import numpy as np
import math as mt
from numba import jit, cuda
from scipy.signal import savgol_filter

#ROS imports
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

#Param for IT_MPC algorithm
K = 1200   #number of samples
T = 80  #number of time steps
cov = np.diag([0.0306, 0.0506])
total_t = 2
delta_t = total_t/T
lamba = 12.5
gamma = 0.1
alpha = 1 - gamma/lamba
control_constraint = [2, 0.34]  # abs(speed) <= 2 / abs(steering_angle) <= 0.34

class IT_MPC:
    def __init__(self):
        #Topics & Subs, Pubs
        topic_pf_odom = "/pf/pose/odom"
        topic_drive = "/drive"

        self.condition = True
        self.control_U = np.zeros((2,T))  # control_U[0]: speed / control_U[1]: steering_angle
        self.sub_pf_odom = rospy.Subscriber(topic_pf_odom, Odometry, self.odom_callback, queue_size=1)
        self.pub_drive = rospy.Publisher(topic_drive, AckermannDriveStamped, queue_size=1)

    def odom_callback(self, odom_msg):
        if self.condition:
            self.condition = False
            ## If condition is true, return odom_msg
            initial_x = np.zeros(6)
            initial_x[0] = odom_msg.pose.pose.position.x
            initial_x[1] = odom_msg.pose.pose.position.y
            initial_x[3] = odom_msg.twist.twist.linear.x
            initial_x[4] = odom_msg.twist.twist.linear.y
            initial_x[5] = odom_msg.twist.twist.angular.z

            ## Computing orientation angle
            q0 = odom_msg.pose.pose.orientation.w
            q3 = odom_msg.pose.pose.orientation.z
            theta1 = mt.asin(2*q0*q3)
            theta2 = mt.acos(1-2*q3**2)
            if theta1 >= 0:
                theta = theta2
            else:
                theta = -theta2
            initial_x[2] = theta

            self.predict_and_decide(initial_x)

    def predict_and_decide(self, initial_x):
        ## Random sampling
        epsil_1 = np.random.normal(0, np.sqrt(cov)[0][0],(K,1,T))
        epsil_2 = np.random.normal(0, np.sqrt(cov)[1][1],(K,1,T))
        epsil = np.concatenate((epsil_1, epsil_2), axis=1)  # epsil[i][0][j]: sample i, time j, speed / epsil[i][1][j]: sample i, time j, steering angle

        sample_cost = self.compute_cost(initial_x, epsil)

        weight = self.compute_weight(sample_cost)

        weight = np.reshape(weight, (K,1,1))
        SGF_input = np.sum(weight*epsil, axis = 0)
        self.control_U = self.control_U + savgol_filter(SGF_input, 15, 5, axis = 0)
        
        self.send_to_actuator(self.control_U[:,0])

        self.control_U[:,0:T-1] = self.control_U[:,1:T]
        self.control_U[:,T-1] = [0,0]
        self.condition = True
    

    ## GPU parallel computing
    @jit(nopython=True, parallel=True, target="cuda")
    def compute_cost(self, initial_x, epsil):
        S = np.zeros(K)  # cost of samples trajectories
        for k in range(K):
            x = initial_x
            for t in range(1,T+1):
                if k < (1-alpha)*K:
                    control_V = np.add(self.control_U[:,t-1], epsil[k][:,t-1])
                else:
                    control_V = epsil[k][:,t-1]

                constrained_control_v = np.arctan(control_V)*control_constraint*(2/mt.pi)

                S[k] += self.state_cost(x) + gamma*np.ndarray.transpose(self.control_U[:,t-1]).dot(np.linalg.inv(cov)).dot(self.control_U[:,t-1] - control_V)
                x = self.transition_model(x, constrained_control_v)
            S[k] += self.terminal_state_cost(x)
        return S
        
    def compute_weight(self, cost):
        lo = np.min(cost)
        cost = np.exp((cost - lo)*(-1/lamba))
        n = np.sum(cost)
        w = np.divide(cost, n)
        return w

    def transition_model(self, x, v):
        x_k_change = np.array([mt.cos(x[2])*x[3] - mt.sin(x[2])*x[4], mt.sin(x[2])*x[3] + mt.cos(x[2])*x[4], x[5]])
        x_k = x[:3] + x_k_change*delta_t
        x_d = x[3:] + x_d_change*delta_t  ###### x_d_cahnge is the value calculated from the NN model
        x_next = np.concatenate([x_k, x_d])
        return x_next

    def state_cost(self, x):
        ##### Calculated state cost based on the waypoint should be in here
        return np.dot(x,x)
    
    def terminal_state_cost(self, x):
        ##### Calculated terminal state cost should be in here
        return np.dot(x,x)

    def send_to_actuator(self, U0):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = U0[0]
        drive_msg.drive.steering_angle = U0[1]
        self.pub_drive.publish(drive_msg)
            

def main():
    rospy.init_node("IT_MPC_node",anonymous=True)
    itmpc = IT_MPC()
    rospy.sleep(0.1)
    rospy.spin()

if __name__  == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("IT_MPC process finished")
