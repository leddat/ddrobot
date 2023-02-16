#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time


class RobotControl():

    def __init__(self, robot_name="ddrobot"):
        rospy.init_node('robot_control_node', anonymous=True)
        rospy.loginfo("Robot ddrobot...")      
        cmd_vel_topic='/cmd_vel'
        self._check_laser_ready()

        # We start the publisher
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.cmd = Twist()        

        self.dd_laser_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.dd_laser_callback)
        
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    
    def _check_laser_ready(self):
        self.dd_laser_msg = None
        rospy.loginfo("Checking DDrobot Laser...")
        while self.dd_laser_msg is None and not rospy.is_shutdown():
            try:
                self.dd_laser_msg = rospy.wait_for_message("/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /scan READY=>" + str(self.dd_laser_msg))

            except:
                rospy.logerr("Current /scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking DDrobot Laser...DONE")
        return self.dd_laser_msg


    def publish_once_in_cmd_vel(self):
        
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        
        self.ctrl_c = True


    def dd_laser_callback(self, msg):
        self.dd_laser_msg = msg

    def get_laser(self, pos):
        time.sleep(1)
        return self.dd_laser_msg.ranges[pos]

    def get_front_laser(self):
        time.sleep(1)
        return self.dd_laser_msg.ranges[360]

    def get_laser_full(self):
        time.sleep(1)
        return self.dd_laser_msg.ranges

    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.15
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s


    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
        return s
    def rotate(self, degrees):

        position = Point()

        # Get the current position
        (position, rotation) = self.get_odom()

        # Set the movement command to a rotation
        if degrees > 0:
            self.cmd.angular.z = 0.3
        else:
            self.cmd.angular.z = -0.3

        # Track the last angle measured
        last_angle = rotation
        
        # Track how far we have turned
        turn_angle = 0

        goal_angle = radians(degrees)

        # Begin the rotation
        while abs(turn_angle + self.angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.vel_publisher.publish(self.cmd) 
            self.rate.sleep()
            
            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last lopp
            delta_angle = self.normalize_angle(rotation - last_angle)
            
            turn_angle += delta_angle
            last_angle = rotation

if __name__ == '__main__':
    
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight()

    except rospy.ROSInterruptException:
        pass