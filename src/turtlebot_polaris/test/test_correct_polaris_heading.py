#!/usr/bin/env python2


 """
        @Author: Akanshu Mahajan
        @Date: 05.07.2022
      """
import unittest
import math
import rospy
import tf
from nav_msgs.msg import Odometry

PKG = 'turtlebot_polaris'

class Test_Polaris_bot(unittest.TestCase):
    """This class is used to carry out the unit test for the orientation
    The true_north_heading is passed as a hardcoded value and it is expected that the 
    robot will orient itself to the passed angle"""

    def setUp(self):

        self.yaw_success_rad = math.radians(rospy.get_param('~yaw_success'))

        # Subscriber
        self.odom = Odometry()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, data):
        self.odom = data
    
    def get_yaw(self):
        """Finding the yaw component of the robot using pose of the robot
         and further transforming the frame"""

        q = (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)
        angle = euler[2]*(-1)
        return angle

    def test_final_heading(self):
        """ The function calculates the continuous difference between the desired angle and the actual angle 
        and continues it's orientation until the error/ differnece is less than the error_limit value"""
        error_limit = 1
        success = False
        while rospy.get_time() == 0.0:
            rospy.sleep(0.1)
        timeout = rospy.get_time() + 10.0

        while (not rospy.is_shutdown() and rospy.get_time() < timeout and(not success)):
            yaw_rad = self.get_yaw()

            yawrate_rad_s = self.odom.twist.twist.angular.z
            
            
            if abs(self.yaw_success_rad - yaw_rad) < error_limit and yawrate_rad_s < error_limit:
                rospy.loginfo("error %s",abs(self.yaw_success_rad - yaw_rad))

                success = True
            rospy.sleep(0.1)

        self.assert_(success)

        
if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rospy.sleep(10)
    rostest.rosrun(PKG, 'test_polaris_bot', Test_Polaris_bot)
    rospy.spin()
