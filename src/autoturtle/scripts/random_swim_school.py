#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
import math, random

class ControlTurtlesim():


    def __init__(self):
        rospy.init_node('ControlTurtlesim', anonymous=False)
        rospy.loginfo(" Press CTRL+c to stop moving the Turtle")
        rospy.on_shutdown(self.shutdown)
        self.set_pen(0, 0, 0, 0, 1)
        spawn_x = random.uniform(0, 11)
        spawn_y = random.uniform(0, 11)
        print(f"Spawn point: {spawn_x}, {spawn_y}")
        self.teleport_turtle(spawn_x, spawn_y, 0.0)
        self.set_pen(255, 255, 255, 2, 0)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.starting_pose = None
        self.tolerance = 0.03
        self.switch_time_interval = 0.4
        rate = rospy.Rate(10);
        rospy.loginfo("Set rate 10Hz")
        
        self.move_cmd = Twist()
        self.move_cmd.linear.x = random.uniform(2, 6)
        self.move_cmd.angular.z = random.uniform(1, 3)
        
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.switch_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.move_cmd)
            rate.sleep()
            
    def pose_callback(self, data):
        self.pose = data
        if self.starting_pose == None:
            self.starting_pose = self.pose
        distance = math.sqrt((self.pose.x - self.starting_pose.x) ** 2 + (self.pose.y - self.starting_pose.y) ** 2)
        if distance < self.tolerance and rospy.get_time()-self.switch_time > self.switch_time_interval:
            self.move_cmd.angular.z *= -1
            self.velocity_publisher.publish(self.move_cmd)
            self.switch_time = rospy.get_time()
            
    def teleport_turtle(self, x, y, theta):
        rospy.wait_for_service('/turtle1/teleport_absolute')
        teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        try:
            teleport_turtle(x, y, theta)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_pen(self, r, g, b, width, off):
        rospy.wait_for_service('/turtle1/set_pen')
        set_pen_service = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        try:
            set_pen_service(r, g, b, width, off)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def shutdown(self):
        rospy.loginfo("Stopping the turtle")
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        ControlTurtlesim()
    except:
        rospy.loginfo("End of the swim for this Turtle.")
        
