#!/usr/bin/env python2

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file:
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    WALL_TOPIC = "/wall"
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    # Parameters
    angle_min = 0
    angle_max = SIDE * 7*np.pi/12
    
    kp_dist = 155 
    #ki_dist = 0.00000001
    kd_dist = 5
    kp_ang = 250 + 150 * np.square(VELOCITY)
    #ki_ang = 0.000000001
    kd_ang = 10

    def __init__(self):
        # Initialize publishers and subscribers
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.get_scan)

        self.dist_err_last = None
        self.ang_err_last = None


    def get_scan(self, data):
        # Convert polar to Cartesian
        distances = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_max, num=len(data.ranges))
        xs = distances * np.cos(angles)
        ys = distances * np.sin(angles)


        # Only look at values from angle_min to angle_max
        if self.SIDE > 0:
            i = [(self.angle_min <= angles) & (angles <= self.angle_max)]
        else:
            i = [(self.angle_min >= angles) & (angles >= self.angle_max)]
        # Weight by distance
        if xs is not None and ys is not None:
            [m, b] = np.polyfit(xs[i], ys[i], 1, w= (self.VELOCITY +np.square(angles[i]))/np.square(distances[i]))
            self.control([m, b])

            y = m * xs[i] +  b

            VisualizationTools.plot_line(xs[i], y, self.line_pub, frame='/laser')
        else:
            self.control([0,self.DESIRED_DISTANCE])

    def control(self, data):
        [m, b] = data
        if self.dist_err_last is None:
            self.dist_err_last = self.DESIRED_DISTANCE - abs(b)
            self.ang_err_last = np.arctan(m)
            self.t_last = rospy.Time.now()
            #self.dist_err_sum = self.dist_err_last * self.t_last.to_sec()
            #self.ang_err_sum = self.ang_err_last * self.t_last.to_sec()
            return

        # PD controller
        t = rospy.Time.now()
        dist_err = self.DESIRED_DISTANCE - abs(b)
        ang_err =  np.arctan(m)
        #dist_err_int = self.dist_err_sum + dist_err * (t - self.t_last).to_sec()
        #ang_err_int = self.ang_err_sum + ang_err * (t - self.t_last).to_sec()
        dist_err_dt = (dist_err - self.dist_err_last) / (t - self.t_last).to_sec()
        ang_err_dt = (ang_err - self.ang_err_last) / (t - self.t_last).to_sec()
        dist_delta = self.kp_dist * dist_err + self.kd_dist * dist_err_dt
        ang_delta = self.kp_ang * ang_err + self.kd_ang * ang_err_dt
        delta = -1 * self.SIDE * dist_delta + ang_delta

        # Update
        self.dist_err_last = dist_err
        self.ang_err_last = ang_err
        #self.dist_err_sum = dist_err_int
        #self.ang_err_sum = ang_err_int

        self.t_last = rospy.Time.now()

        # Send command
        command = AckermannDriveStamped()
        command.header.stamp = rospy.Time.now()
        command.drive.speed = self.VELOCITY
        command.drive.steering_angle =  delta
        self.drive_pub.publish(command)

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
