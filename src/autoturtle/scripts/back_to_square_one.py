#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen, TeleportRelative
from std_srvs.srv import Empty
import math, random

class ControlTurtlesim():


    def __init__(self):
        rospy.init_node('ControlTurtlesim', anonymous=False)
        rospy.loginfo(" Press CTRL+c to stop moving the Turtle")
        rospy.on_shutdown(self.shutdown)
        rate = rospy.Rate(10);
        rospy.loginfo("Set rate 10Hz")
        self.set_pen(0, 0, 0, 0, 1)
        spawn_x = 1
        spawn_y = 1
        rospy.loginfo(f"Spawn point: {spawn_x}, {spawn_y}")
        self.teleport_turtle(spawn_x, spawn_y, 0.0)
        self.set_pen(255, 255, 255, 2, 0)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.turn_status = 0 # 0=right, 1=up, 2=left, 3=down
        self.target_pose = None
        self.tolerance = 0.01
        
        side_length = 1
        while True:
            try:
                side_length = float(input("Enter side length: "))
                if side_length>=1 and side_length<=5:
                    break
                else:
                    print("Side length must be between 1 and 5")
            except ValueError:
                print("Invalid input")
        self.side_length = side_length
        
        print("changing background")
        self.change_background_color(255, 0, 0)
        self.move_cmd = Twist()
        self.default_linear_speed = 3
        self.move_cmd.linear.x = self.default_linear_speed
        self.move_cmd.angular.z = 0
        
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.move_cmd)
            rate.sleep()
            
    def pose_callback(self, data):
        self.pose = data
        if self.target_pose == None:
            self.update_target()
        distance = math.sqrt((self.pose.x - self.target_pose.x) ** 2 + (self.pose.y - self.target_pose.y) ** 2)
        if distance < self.tolerance:
            self.turn_status += 1
            if self.turn_status > 3:
                #self.change_background_color(69, 86, 255) # reset background color back to default blue
                self.move_cmd = None
                self.shutdown()
                return
            self.update_target()
            self.teleport_relative(0, 90)
            self.move_cmd.linear.x = self.default_linear_speed
            self.velocity_publisher.publish(self.move_cmd)
        else:
            self.move_cmd.linear.x = max(0.1, self.default_linear_speed * (distance/self.default_linear_speed))
            
    def update_target(self):
        self.target_pose = Pose()
        self.target_pose.x = self.pose.x
        self.target_pose.y = self.pose.y
        self.target_pose.theta = self.pose.theta
        if self.turn_status == 0:
            self.target_pose.x += self.side_length
        elif self.turn_status == 1:
            self.target_pose.y += self.side_length
        elif self.turn_status == 2:
            self.target_pose.x -= self.side_length
        elif self.turn_status == 3:
            self.target_pose.y -= self.side_length
        else:
            self.shutdown()
        
    def teleport_relative(self, linear, angular):
        rospy.wait_for_service('/turtle1/teleport_relative')
        teleport_relative = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)
        try:
            teleport_relative(linear, math.radians(angular))
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            
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
         
    def change_background_color(self, r, g, b):
        rospy.wait_for_service('/clear')
        clear = rospy.ServiceProxy('/clear', Empty)
        try: 
            print("setting params")
            rospy.set_param('/turtlesim/background_r' , r)
            rospy.set_param('/turtlesim/background_g' , g)
            rospy.set_param('/turtlesim/background_b' , b)
            clear()
        except rospy.ServiceException as e:
            rospy.logerr(f'Service failed with the exception {e}')

        def shutdown(self):
            rospy.loginfo("Stopping the turtle")
            self.velocity_publisher.publish(Twist())
            rospy.sleep(1)

    def shutdown(self):
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        ControlTurtlesim()
    except:
        rospy.loginfo("End of the swim for this Turtle.")
        
