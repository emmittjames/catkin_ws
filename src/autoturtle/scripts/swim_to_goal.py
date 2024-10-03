#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.loginfo(" Press CTRL+c to stop moving the Turtle")
        #rospy.on_shutdown(self.shutdown)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        rospy.loginfo("Set rate 10Hz")

    def pose_callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 3)
        self.pose.y = round(self.pose.y, 3)

    def calculate_distance(self, target):
        return sqrt(pow((target.x - self.pose.x), 2) + pow((target.y - self.pose.y), 2))
        
    def shutdown(self):
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)

    def main(self):
        target_pose = Pose()
        target_pose.x = float(input("Enter the x goal: "))
        target_pose.y = float(input("Enter the y goal: "))
        tolerance = float(input("Enter the tolerance: "))

        vel_msg = Twist()
        v_gain = 1.5
        a_gain = 6
        while self.calculate_distance(target_pose) >= tolerance:
            vel_msg.linear.x = self.calculate_distance(target_pose) * v_gain
            
            vel_msg.angular.z = (atan2(target_pose.y - self.pose.y, target_pose.x - self.pose.x) - self.pose.theta) * a_gain
            
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        #self.shutdown()

        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.main()
    except rospy.ROSInterruptException:
        pass
        
