#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import *
from numpy import random

class MoveTurtle:

    def __init__(self):

        rospy.init_node('move_turtle_foward', anonymous = True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.odometry = Odometry()
        self.rate = rospy.Rate(5)

        self.velocity_msg = Twist()

    def adiciona_perturbacao_vel(self, vel):
        ''' loc = media, scale = desvio padrao '''

        vel_com_perturbacao = Twist()
        vel_com_perturbacao.linear.x  = vel.linear.x  + random.normal(loc = 0.0, scale = 0.1)     
        vel_com_perturbacao.angular.z = vel.angular.z + random.normal(loc = 0.0, scale = 0.1) 
        
        return vel_com_perturbacao
    
    def move_foward(self):
        self.velocity_msg.linear.x= 0.1
        self.velocity_publisher.publish(self.adiciona_perturbacao_vel(self.velocity_msg))
        
if __name__ == "__main__":
    while not rospy.is_shutdown():
        MoveTurtle().move_foward()

    rospy.spin()