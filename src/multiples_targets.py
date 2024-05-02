#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class TurtleBotNavigator:
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)      
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)    
        self.current_pose = PoseStamped()                                 

        #targets
        self.goals = [
            (1.0, 0.0),
            (1.5, 1.0),
            (2.0, 2.0),
            (3.0, 5.0)]

        self.current_goal_index = 0
        self.next_goal()

    def odom_callback(self, odom):              
        self.current_pose.header = odom.header
        self.current_pose.pose = odom.pose.pose

    def quaternion_to_yaw(self, quaternion):    
        _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw

    def send_velocity_command(self, linear_x, angular_z):   
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        self.velocity_publisher.publish(vel_msg)

    def next_goal(self):                                    
        current_goal = self.goals[self.current_goal_index]
        self.navigate_to_goal(current_goal)

    def navigate_to_goal(self, goal):                        
        while not rospy.is_shutdown():                     
            x_target, y_target = goal
            x_pose = self.current_pose.pose.position.x
            y_pose = self.current_pose.pose.position.y

            x_distance = x_target - x_pose
            y_distance = y_target - y_pose

            theta = math.atan2(y_distance, x_distance)
            point_distance = math.sqrt(x_distance ** 2 + y_distance ** 2)
            angle_distance = theta - self.quaternion_to_yaw(self.current_pose.pose.orientation)

            if point_distance <= 0.1:                       
                print("Chegueei!")
                self.send_velocity_command(0.0, 0.0)        
                self.current_goal_index += 1                
                if self.current_goal_index < len(self.goals):  
                    self.next_goal()
                else:
                    rospy.loginfo("Cheguei em todos os alvos! Tchaau")  
                    rospy.signal_shutdown("All goals reached")          
                break
            else:
                self.send_velocity_command(0.2 * point_distance, 0.4 * angle_distance)  
                                                                                        
if __name__ == "__main__":
    TurtleBotNavigator()
    rospy.spin()
