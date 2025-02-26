#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from robot_utils import Utils


class TurtlebotController():
    
    def __init__(self, rate):
        
        robot_vel_topic = rospy.get_param('~robot_vel_topic', 'speed')
        robot_scan_topic = rospy.get_param('~robot_scan_topic', 'laser') 
        
        # read the maximum linear and angular velocities
        # from the parameter server!!!!
        self.max_lin_vel = rospy.get_param('~max_lin_vel', 0.2)  # m/s
        self.max_ang_vel = rospy.get_param('~max_ang_vel', 0.4)  # rad/s
        
        self.rate = rate # Hz  (1/Hz = secs)
        # we store the received path here
        self.path = Path()
        self.path_received = False
        self.laser = LaserScan()
        self.laser_received = False
        
        # Declare the velocity command publisher
        self.cmd_vel = rospy.Publisher(robot_vel_topic, Twist, queue_size=10)
        
        self.listener = tf.TransformListener()

        self.utils = Utils(self.listener, "odom")
        
        # subscription to the scan topic [sensor_msgs/LaserScan]
        rospy.Subscriber(robot_scan_topic, LaserScan, self.laserCallback)
        
        # subscription to the path topic [nav_msgs/Path]
        rospy.Subscriber("path", Path, self.pathCallback)
     
        
    def command(self):

        # TODO: check if the final goal has been reached
        if(self.goalReached()==True):
            rospy.loginfo("GOAL REACHED!!! Stopping!")
            self.publish(0.0, 0.0)
            return True
        
        
        # Determine the local path point to be reached
        # TODO: fill the method getSubGoal
        current_goal = self.getSubGoal() 
            
        # TODO: use current_goal 
        # Put your control law here (copy from EPD1)
        angular = 0.0
        linear = 0.0
        # check the maximum speed values allowed
        if(abs(angular) > self.max_ang_vel):
            if(angular < 0):
                angular = -self.max_ang_vel
            else:
                angular = self.max_ang_vel
        if(linear > self.max_lin_vel):
            linear = self.max_lin_vel
            
            
        # If the computed commands does not provoke a collision,
        # send the commands to the robot
        # TODO: fill the checkCollision function
        if(self.checkCollision(linear, angular)==False):
            self.publish(linear,angular)
            return False

        # if a possible collision is detected,
        # try to find an alternative command to
        # avoid the collision
        # TODO: fill the CollisionAvoidance function
        linear, angular = self.collisionAvoidance() 
        self.publish(linear,angular)
        return False


    def goalReached(self):
        # -------------------------------------------------------------
        # TODO: use the last point of the path to check if the robot
        # has reached the final goal (the robot is in a close position).
        # return True if the FINAL goal was reached, False otherwise
        return False
        # -------------------------------------------------------------


    def getSubGoal(self):
        # -------------------------------------------------------------
        # TODO: use self.path.poses to find the subgoal to be reach
        # You could transform the path points to the robot reference
        # to find the closest point:
        # path_pose = self.path.poses[index]
        # path_pose_in_robot_frame = self.utils.transformPose(path_pose, 'base_footprint')
        subgoal = PoseStamped()
        # -------------------------------------------------------------
        return subgoal


    def checkCollision(self, linear, angular):
        # -------------------------------------------------------------
        # TODO: use self.laser to check possible collisions
        # Optionally, you can also use the velocity commands
        # return True if possible collision, False otherwise
        return False
        # -------------------------------------------------------------


    def collisionAvoidance(self):
        # -------------------------------------------------------------
        # TODO: try to find an alternative command to avoid the collision
        # Here you must try to implement one of the reactive methods
        # seen in T4: bug algorithm, potential fields, velocity obstacles,
        # Dynamic Window Approach, others...
        # Feel free to add the new variables and methods that you may need 
        ang_vel = 0
        lin_vel = 0
        # -------------------------------------------------------------
        return lin_vel, ang_vel

        
    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # Copy the forward velocity
        move_cmd.linear.x = lin_vel
        # Copy the angular velocity
        move_cmd.angular.z = ang_vel
        #rospy.loginfo("Commanding lv: %.2f, av: %.2f", lin_vel, ang_vel)
        self.cmd_vel.publish(move_cmd)


    def laserCallback(self,data):
        self.laser = data
        self.laser_received = True
        #rospy.loginfo("Laser received " + str(len(data.ranges)))
        

    def pathCallback(self,path):
        self.path = path
        self.path_received = True
        

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
 
if __name__ == '__main__':
    #try:
        # initiliaze
        rospy.init_node('TurtlebotController', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        rate = 10 # Frecuency (Hz) for commanding the robot
        robot=TurtlebotController(rate)
        # What function to call when you ctrl + c    
        rospy.on_shutdown(robot.shutdown)
        
        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(rate)
        end = False
        # as long as you haven't ctrl + c keeping doing...
        while not (rospy.is_shutdown() or end==True):
            #rospy.loginfo("Loop")
	        # publish the velocity
            end = robot.command()
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    #except:
    #    rospy.loginfo("TurtlebotController node terminated.")
        
