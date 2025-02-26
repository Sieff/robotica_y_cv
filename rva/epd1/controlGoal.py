#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
import numpy as np


class Turtlebot():
    def __init__(self):
          
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.listener = tf.TransformListener()
     
	
        
    def command(self,gx, gy):
        rospy.loginfo("Command")
        
        goal = PointStamped()
        base_goal = PointStamped()
        
        goal.header.frame_id = 'odom'
        goal.header.stamp = rospy.Time()
        
        goal.point.x = gx
        goal.point.y = gy
        goal.point.z = 0.0
        
        try:
            base_goal = self.listener.transformPoint('base_footprint', goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            return
        
        #------------------------------------------------------------------
        # TODO: Put the control law here.
        # Compute linear and angular vels based on the distance to the goal
        # and the angle between the robot heading and the goal.
        linear = 0.0
        angular = 0.0

        base_goal_a = np.array([base_goal.point.x, base_goal.point.y])
        forward = np.array([1, 0])

        v1, v2 = base_goal_a

        # Error angle towards goal in base coordinate system
        angle = math.atan2(v2, v1)
        rospy.loginfo("Angle")
        rospy.loginfo(angle)

        angular = max(min(angle * 0.1, 0.1), -0.1)
        rospy.loginfo("Angular")
        rospy.loginfo(angular)

        distance = np.linalg.norm(base_goal_a)
        if abs(angle) <= math.pi / 8:
            linear = min(distance * 0.1, 1)

        rospy.loginfo("Distance")
        rospy.loginfo(distance)
        rospy.loginfo("Linear")
        rospy.loginfo(linear)
        #------------------------------------------------------------------
        
        self.publish(linear,angular)


    def publish(self,lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # Copy the forward velocity
        move_cmd.linear.x = lin_vel
        # Copy the angular velocity
        move_cmd.angular.z = ang_vel

        self.cmd_vel.publish(move_cmd)
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('robotcontrol', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        robot=Turtlebot()
        # What function to call when you ctrl + c    
        rospy.on_shutdown(robot.shutdown)

        goalx=float(sys.argv[1])
        goaly=float(sys.argv[2])

        print(goalx)
        print(goaly)

        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            rospy.loginfo("Loop")
            # publish the velocity
            robot.command(goalx,goaly)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    except:
        rospy.loginfo("robotcontrol node terminated.")
