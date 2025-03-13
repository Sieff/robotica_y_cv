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
import numpy as np


class TurtlebotController():
    
    def __init__(self, rate):
        
        robot_vel_topic = rospy.get_param('~robot_vel_topic', 'speed')
        robot_scan_topic = rospy.get_param('~robot_scan_topic', 'laser') 
        
        # read the maximum linear and angular velocities
        # from the parameter server!!!!
        self.max_lin_vel = rospy.get_param('~max_lin_vel', 0.2)  # m/s
        self.max_ang_vel = rospy.get_param('~max_ang_vel', 0.4)  # rad/s

        self.near_collision = False
        self.r_soi = 1 # Sphere of influence radius
        
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
        if not current_goal:
            self.publish(0.0, 0.0)
            return False

            
        # TODO: use current_goal 
        # Put your control law here (copy from EPD1)
        angular = 0.0
        linear = 0.0

        linear, angular = self.velocity(current_goal.pose.position.x, current_goal.pose.position.y)

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

    
    def velocity(self, x, y):
        linear = 0
        angular = 0

        target = np.array([x, y])

        # Error angle towards goal in base coordinate system
        angle = math.atan2(y, x)
        # Log angle error towards goal
        # rospy.loginfo("Angle")
        # rospy.loginfo(angle)

        allowed_angle_deg = 10
        allowed_angle = allowed_angle_deg * math.pi / 180

        angular = angle * 0.4
        if abs(angle) > 1.5 * allowed_angle:
            if angle > 0:
                angular = max(angular, self.max_ang_vel)
            if angle < 0:
                angular = min(angular, -self.max_ang_vel)    
        # Log angular target velocity
        # rospy.loginfo("Angular")
        # rospy.loginfo(angular)

        distance = np.linalg.norm(target)

        if abs(angle) <= allowed_angle:
            linear = max(min(distance * 0.1, 1), 0.05) # Minimum 0.05 m/s; Maximum 1 m/s

        return linear, angular


    def goalReached(self):
        # -------------------------------------------------------------
        # TODO: use the last point of the path to check if the robot
        # has reached the final goal (the robot is in a close position).
        # return True if the FINAL goal was reached, False otherwise
        if len(self.path.poses) == 0:
            return False

        final_goal = self.path.poses[-1]
        final_goal_in_robot_frame = self.utils.transformPose(final_goal, 'base_footprint')
        final_goal_a = np.array([final_goal_in_robot_frame.pose.position.x, final_goal_in_robot_frame.pose.position.y])
        distance = np.linalg.norm(final_goal_a)
        if distance > 0 and distance < 0.05:
            return True

        return False
        # -------------------------------------------------------------


    def getSubGoal(self):
        # -------------------------------------------------------------
        # TODO: use self.path.poses to find the subgoal to be reach
        # You could transform the path points to the robot reference
        # to find the closest point:
        # path_pose = self.path.poses[index]
        # path_pose_in_robot_frame = self.utils.transformPose(path_pose, 'base_footprint')
        if len(self.path.poses) == 0:
            return False

        lookahead = 1

        distance = math.inf
        index = -1
        for i, point in enumerate(self.path.poses):
            point_in_robot_frame = self.utils.transformPose(point, 'base_footprint')
            point_a = np.array([point_in_robot_frame.pose.position.x, point_in_robot_frame.pose.position.y])
            distance_to_point = np.linalg.norm(point_a)
            if distance_to_point < distance:
                index = i
                distance = distance_to_point

        index = min(index + lookahead, len(self.path.poses) - 1)
        subgoal_pose = self.path.poses[index]
        subgoal = self.utils.transformPose(subgoal_pose, 'base_footprint')
        # -------------------------------------------------------------
        return subgoal


    def checkCollision(self, linear, angular):
        # -------------------------------------------------------------
        # TODO: use self.laser to check possible collisions
        # Optionally, you can also use the velocity commands
        # return True if possible collision, False otherwise
        if not self.laser:
            rospy.loginfo("No laser")
            return False

        if len(self.laser.ranges) == 0:
            rospy.loginfo("No laser ranges")
            return False

        # End collision detection when distance greater than sphere of influence radius + 0.3m
        if np.min(self.laser.ranges) > self.r_soi + 0.3:
            self.near_collision = False
            return False

        # Start collision detection when distance smaller than sphere of influence radius
        if self.near_collision or np.min(self.laser.ranges) < self.r_soi:
            self.near_collision = True
            return True

        
        return False
        # -------------------------------------------------------------


    def collisionAvoidance(self):
        # -------------------------------------------------------------
        # TODO: try to find an alternative command to avoid the collision
        # Here you must try to implement one of the reactive methods
        # seen in T4: bug algorithm, potential fields, velocity obstacles,
        # Dynamic Window Approach, others...
        # Feel free to add the new variables and methods that you may need 

        # Find angle of minimum laser
        index = np.argmin(self.laser.ranges)
        angle = (self.laser.angle_min + index * self.laser.angle_increment)

        # Calculate point from angle and distance
        x = self.laser.ranges[index] * math.cos(angle)
        y = self.laser.ranges[index] * math.sin(angle)

        min_distance = min(np.min(self.laser.ranges), 1)

        # Use inverse point of closest obstacle as repellent force and normalize
        repellent_force = np.array([-x, -y])
        repellent_force_normal = (repellent_force / np.linalg.norm(repellent_force))

        # Get current goal and normalize
        current_goal_pose = self.getSubGoal()
        target_direction = np.array([current_goal_pose.pose.position.x, current_goal_pose.pose.position.y])
        target_direction_normal = (target_direction / np.linalg.norm(target_direction))

        # Potential field method forces
        m_1 = 1
        goal_force = target_direction_normal * m_1

        m_2 = 0
        if min_distance <= self.r_soi:
            m_2 = (self.r_soi - min_distance) / min_distance
        potential_field_force = repellent_force_normal * m_2

        # Add forces with weights to get new target force
        w_goal = 1
        w_potential_field = 1
        target_force = goal_force * w_goal + potential_field_force * w_potential_field

        # rospy.loginfo(f"Cur: {target_direction_normal}, Pot: ${potential_field_force_normal}, new: ${target_force}")

        linear, angular = self.velocity(target_force[0], target_force[1])
        
        ang_vel = angular
        lin_vel = linear
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
        
