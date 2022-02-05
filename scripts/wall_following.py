#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
# kp = #TODO
# kd = #TODO
# ki = #TODO
# servo_offset = 0.0
# prev_error = 0.0 
# error = 0.0
# integral = 0.0

# #WALL FOLLOW PARAMS
# ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
# DESIRED_DISTANCE_RIGHT = 0.9 # meters
# DESIRED_DISTANCE_LEFT = 0.55
# VELOCITY = 2.00 # meters per second
# CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs

        self.a_deg = 60 #30 degree for calculation
        self.b_deg = 90
        self.theta = (90 - self.a_deg)/ 360 * 2 * np.pi
        self.a = []
        self.b = []
        self.collect_t = -1
        self.sync_time = 1.0
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.get_range_wrapper)
        # self.drive_pub = #TODO: Publish to drive
        self.debug_pub = rospy.Publisher('debug', String, queue_size=10)

    def _debug(self, msg, prefix=""):
        self.debug_pub.publish(prefix + str(msg))

    def reset(self, t):
        self.a = []
        self.b = []
        self.collect_t = t

    def get_time_from_header(self, data):
        t = data.header.stamp
        t = t.secs + t.nsecs*1e-9
        if self.collect_t < 0:
            self.collect_t = t
        return t

    def get_range_wrapper(self, data):
        t = self.get_time_from_header(data)
        if t - self.collect_t < self.sync_time:
            #TODO: static!
            a_idx = int((180+self.a_deg)/360*len(data.ranges))
            b_idx = int((180+self.b_deg)/360*len(data.ranges))

            a, b = data.ranges[a_idx], data.ranges[b_idx]
            self.a.append(a)
            self.b.append(b)
        else:
            if len(self.a)==0 or len(self.b)==0:
                return
            self.get_range(np.mean(self.a), np.mean(self.b))
            self.reset(t)



    def get_range(self, a, b):
        self._debug(a, prefix="a: ")
        self._debug(b, prefix="b: ")
        alpha = np.arctan((a*np.cos(self.theta)-b)/(a*np.sin(self.theta)))
        self._debug(alpha, prefix="alpha:")

        Dt = b * np.cos(alpha)
        self._debug(Dt, prefix="Dt:")


    # def pid_control(self, error, velocity):
    #     global integral
    #     global prev_error
    #     global kp
    #     global ki
    #     global kd
    #     angle = 0.0
    #     #TODO: Use kp, ki & kd to implement a PID controller for 
    #     drive_msg = AckermannDriveStamped()
    #     drive_msg.header.stamp = rospy.Time.now()
    #     drive_msg.header.frame_id = "laser"
    #     drive_msg.drive.steering_angle = angle
    #     drive_msg.drive.speed = velocity
    #     self.drive_pub.publish(drive_msg)

    # def followLeft(self, data, leftDist):
    #     #Follow left wall as per the algorithm 
    #     #TODO:implement
    #     return 0.0 

    # def lidar_callback(self, data):
    #     """ 
    #     """
    #     error = 0.0 #TODO: replace with error returned by followLeft
    #     #send error to pid_control
    #     self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)