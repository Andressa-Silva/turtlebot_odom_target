#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion

class MoveTurtle():

    def __init__(self):

        rospy.init_node('move_turtle_foward', anonymous = True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.move2goal, queue_size=10)
        
        self.velocity_msg = Twist()
        self.rate = rospy.Rate(5)

        self.move_foward(0.2, 0.0)

    def adiciona_perturbacao_vel(self, vel):
        """ Adds disturbance
            loc = media, 
            scale = desvio padrao """

        vel_com_perturbacao = Twist()
        vel_com_perturbacao.linear.x  = vel.linear.x  + np.random.normal(loc = 0.0, scale = 0.1)     
        vel_com_perturbacao.angular.z = vel.angular.z + np.random.normal(loc = 0.0, scale = 0.1) 
        
        return vel_com_perturbacao
    
    def move_foward(self, vel_x, vel_z):
        """ Move the robot """

        self.velocity_msg.linear.x  = vel_x
        self.velocity_msg.angular.z = vel_z

        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.velocity_msg)
            self.rate.sleep()
            #self.velocity_publisher.publish(self.adiciona_perturbacao_vel(self.velocity_msg))
            
    def move2goal(self, odom):
        """ Move the robot to the target position """
                    
        position = odom.pose.pose.position
        x_pose = position.x
        y_pose = position.y
            
        print(x_pose, y_pose)
        target_position = (2.0, 0.0)

        distance_x = target_position[0] - x_pose
        distance_y = target_position[1] - y_pose

        print("DISTANCE:", distance_x)

        if distance_x <= 0.0:
            print("CHEGUEEEEEEEEEEEI")
            self.move_foward(0.0 , 0.0)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        MoveTurtle()

    rospy.spin()

    

    

    
