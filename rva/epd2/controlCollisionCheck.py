#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np

class Turtlebot():
    
    def __init__(self):
        
        robot_vel_topic = rospy.get_param('~robot_vel_topic', 'speed')
        robot_scan_topic = rospy.get_param('~robot_scan_topic', 'laser') 
        
        # -------------------------------------------------------------
        # TODO: read the maximum linear and angular velocities
        #   from the parameter server!!!!
        rospy.loginfo("Reading parameters...")
        self.max_lin_vel = rospy.get_param('~max_lin_vel') # m/s
        self.max_ang_vel = rospy.get_param('~max_ang_vel') # rad/s
        rospy.loginfo(f"Received parameters: max_lin_vel: {self.max_lin_vel}; max_ang_vel: {self.max_ang_vel}.")
        # -------------------------------------------------------------
        
        self.goal = PointStamped()
        self.goal.header.frame_id = "odom"
        
        self.cmd_vel = rospy.Publisher(robot_vel_topic, Twist, queue_size=10)
        
        self.listener = tf.TransformListener()

        self.laser = None
        
        # subscription to the scan topic [sensor_msgs/LaserScan]
        rospy.Subscriber(robot_scan_topic, LaserScan, self.laserCallback)
        
        # subscription to the goal topic [geometry_msgs/PoseStamped]
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
     
        
    def command(self):
        
        base_goal = PointStamped()
        
        # we update the goal timestamp
        self.goal.header.stamp = rospy.Time()
      
        try:
            base_goal = self.listener.transformPoint('base_footprint', self.goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(f"Problem TF: {e}")
            return
            
        # -------------------------------------------------------------
        # TODO: put the control law here (copy from EPD1)
        angular = 0.0
        linear = 0.0

        base_goal_a = np.array([base_goal.point.x, base_goal.point.y])
        forward = np.array([1, 0])

        v1, v2 = base_goal_a

        # Error angle towards goal in base coordinate system
        angle = math.atan2(v2, v1)
        # Log angle error towards goal
        # rospy.loginfo("Angle")
        # rospy.loginfo(angle)

        allowed_angle_deg = 5
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

        distance = np.linalg.norm(base_goal_a)

        if abs(angle) <= allowed_angle:
            linear = max(min(distance * 0.1, 1), 0.05) # Minimum 0.05 m/s; Maximum 1 m/s

        # Log distance and linear velocity
        # rospy.loginfo(f"Distance to goal: {distance}")
        # rospy.loginfo(f"Linear target velocity: {linear}")

        # TODO: check the maximum speed values allowed
        linear = min(linear, self.max_lin_vel)
        angular = max(min(angular, self.max_ang_vel), -self.max_ang_vel)
        # -------------------------------------------------------------
        
        # -------------------------------------------------------------
        # TODO: fill the checkCollision function
        if(self.checkCollision()):
            rospy.loginfo("possible collision! Stopping!!!!")
            linear = 0.0
            # Allow turning?
            # angular = 0.0
        
        self.publish(linear,angular)
        

    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # Copy the forward velocity
        move_cmd.linear.x = lin_vel
        # Copy the angular velocity
        move_cmd.angular.z = ang_vel
        # rospy.loginfo("Commanding lv: %.2f, av: %.2f", lin_vel, ang_vel)
        self.cmd_vel.publish(move_cmd)
        
        
    def checkCollision(self):
        # -------------------------------------------------------------
        # TODO: use self.laser to check possible collisions
        # return True if possible collision, False otherwise

        if not self.laser:
            rospy.loginfo("No laser")
            return False

        # Simple
        # if np.min(self.laser.ranges) < 0.3:
        #     return True
        # return False

        # Only in front
        fov_deg = 90 # deg
        fov = fov_deg * math.pi / 180
        min_angle = -(fov / 2) % (2 * math.pi)
        max_angle = fov / 2 % (2 * math.pi)

        angles = np.array(range(len(self.laser.ranges))) * self.laser.angle_increment

        # Find the indices where the min_angle and max_angle will be in the ranges array
        start_index = -1
        end_index = -1
        for i, angle in enumerate(angles):
            if i + 1 == len(angles):
                if start_index == -1:
                    start_index = i
                if end_index == -1:
                    end_index = i
                continue
            if min_angle >= angles[i] and min_angle < angles[i + 1]:
                start_index = i
            if max_angle >= angles[i] and max_angle < angles[i + 1]:
                end_index = i

        # Select only ranges between min_angle and max_angle
        relevant_ranges = self.laser.ranges[start_index:end_index]
        if start_index > end_index:
            # Wrap around the edges
            relevant_ranges += self.laser.ranges[0:end_index]
            relevant_ranges += self.laser.ranges[start_index:]

        min_range = np.min(relevant_ranges)

        if min_range < 0.3:
            return True
        return False
        # -------------------------------------------------------------


    def laserCallback(self,data):
        self.laser = data
        #rospy.loginfo("Laser received " + str(len(data.ranges)))
        
        
    def goalCallback(self,goal):
        rospy.loginfo("Goal received! x: %.2f, y:%.2f", goal.pose.position.x, goal.pose.position.y)
        self.goal.header = goal.header
        self.goal.point.x = goal.pose.position.x
        self.goal.point.y = goal.pose.position.y
        
        
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
        rospy.init_node('controlCollisionCheck', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        robot=Turtlebot()
        # What function to call when you ctrl + c    
        rospy.on_shutdown(robot.shutdown)
        
        
        if(len(sys.argv) > 3):
            robot.goal.point.x=float(sys.argv[1])
            robot.goal.point.y=float(sys.argv[2])
            rospy.loginfo("Goal received by command line args! x: %.2f, y:%.2f", robot.goal.point.x, robot.goal.point.y)

        
        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            #rospy.loginfo("Loop")
	        # publish the velocity
            robot.command()
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    except:
        rospy.loginfo("ControlCollisionCheck node terminated.")
        
