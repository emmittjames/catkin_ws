#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class ControlTurtlesim():

    def pose_callback(self, data):
        self.pose = data
        if self.starting_pose == None:
            self.starting_pose = self.pose
        distance = math.sqrt((self.pose.x - self.starting_pose.x) ** 2 + (self.pose.y - self.starting_pose.y) ** 2)
        if distance < self.tolerance and rospy.get_time()-self.switch_time > self.switch_time_interval:
            self.move_cmd.angular.z *= -1
            self.velocity_publisher.publish(self.move_cmd)
            self.switch_time = rospy.get_time()

    def __init__(self):
        rospy.init_node('ControlTurtlesim', anonymous=False)
        rospy.loginfo(" Press CTRL+c to stop moving the Turtle")
        rospy.on_shutdown(self.shutdown)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        #self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.starting_pose = None
        self.tolerance = 0.03
        self.switch_time_interval = 0.4
        rate = rospy.Rate(10);
        rospy.loginfo("Set rate 10Hz")
        self.move_cmd = Twist()
        x_velocity = 2
        while True:
            try:
                x_velocity = float(input("Enter x velocity: "))
                if x_velocity>=2 and x_velocity<=6:
                    break
                else:
                    print("x velocity must be between 2 and 6")
            except ValueError:
                print("Invalid input")
        self.move_cmd.linear.x = x_velocity

        z_velocity = 1
        while True:
            try:
                z_velocity = float(input("Enter z velocity: "))
                if z_velocity>=1 and z_velocity<=3:
                    break
                else:
                    print("z velocity must be between 1 and 3")
            except ValueError:
                print("Invalid input")
        self.move_cmd.angular.z = z_velocity
        
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.switch_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.move_cmd)
            rate.sleep()
            

    def shutdown(self):
        rospy.loginfo("Stopping the turtle")
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        ControlTurtlesim()
    except:
        rospy.loginfo("End of the swim for this Turtle.")
        
