#!/usr/bin/env python2

 """
        @Author: Akanshu Mahajan
        @Date: 05.07.2022
        """

import rospy
import tf
import math
from geometry_msgs.msg import Twist

#from utils import Utils
from PI_controller import PI

import rospy
from nav_msgs.msg import Odometry


class Turtlebot_polaris():

    """
    This class is the central implementation of the package. It achieves the following:
    - Given the True North heading angle, corresponding to the North Pole, calculate the robot motion to orient it to the North star.
    - The position of the Tru North heading angle is currently found from http://www.geomag.bgs.ac.uk/data_service/models_compass/wmm_calc.html 
        using the Latitute, Longitude & Altitude of the location.
        To- Do : An API can be integrated/ a python wrapper can be written that will find the true north heading angle using Latitude, Longitude & Altitude.
        This can be further done using a GIS sensory system
    - Orient the turtlebot to the True North heading, observe data from the /odom topic and publish angular velocity commands to the /cmd_vel topic.

    Attributes
    ----------
    kp : double
        Propotional Gain
    ki : double
        Integral Gain
    max_angspeed_rad_s : double
        Maximum angular velocity of the robot's actuatator
    mag_declination_inertial : double
        True North Heading angle in the inertial frame
    main_loop_freq : double
        The main loop frequency
    Methods
    -------
    says(sound=None)
        Prints the animals name and what sound it makes    



    """

    def __init__(self, *args):
        #super(Turtlebot_polaris, self).__init__(*args)
        
        # Subscriber
        self.odom = Odometry()                      # Odometery of the Turtlebot
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        #Parameters
        self.main_loop_freq = rospy.get_param('~loop_freq')
        self.max_angspeed_rad_s = rospy.get_param('~max_ang_speed_rad_s')
        self.kp = rospy.get_param('~kp')
        self.ki = rospy.get_param('~ki')
        self.PI_controller = PI(self.kp, self.ki)
        self.mag_declination_inertial = rospy.get_param('~mag_declination_inertial')

        
    def odom_callback(self, data):
        """Odom callback function"""
        self.odom = data

    def get_yaw(self):
        """Finding the yaw component of the robot using pose of the robot
         and further transforming the frame"""
        q = (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)
        angle = euler[2]*(-1)
        return angle

    def rotate(self, yaw_rad, yaw_req_rad):
        """Commanding the robots actuator to implement the orientation using PI controller """
        yaw_e_rad = yaw_req_rad - yaw_rad
        
        # The below logic helps robot take the shorter route
        if yaw_e_rad > math.pi:
            yaw_e_rad -= 2 * math.pi
        if yaw_e_rad < - math.pi:
            yaw_e_rad += 2 * math.pi

        yaw_com = self.PI_controller.update(yaw_e_rad)

        # publish the command
        command = Twist()
        command.angular.z = yaw_com
        self.cmd_vel_pub.publish(command)

    
    def mag_declination_calc(self):
        """The function calculating the Magnetic Declination Angle in the odom/ robot frame"""
        mag_declination_odom = 90 + self.mag_declination_inertial
        return mag_declination_odom
    
    def run(self):
        rate = rospy.Rate(self.main_loop_freq)
        mag_declination_odom = self.mag_declination_calc()
        yaw_req_rad = math.radians(mag_declination_odom)
        while not rospy.is_shutdown():
            yaw_rad = self.get_yaw()
            self.rotate(yaw_rad, yaw_req_rad)
            rate.sleep

def main():
    """Initiate the node to orientate the robot to the polaris"""
    rospy.init_node('Turtlebot_polaris', anonymous=True)
    turtle = Turtlebot_polaris()
    turtle.run()

if __name__ == '__main__':
    main()

