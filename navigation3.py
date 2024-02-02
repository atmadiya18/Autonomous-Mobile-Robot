#!/usr/bin/env python3

from pickle import FALSE, TRUE
import re
import sys
import os
import numpy as np

import math
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry

from Controlllers import PidController
from astar_map import trigger
from nav_msgs.msg import OccupancyGrid
from numpy import floor
from math import pow, atan2, radians, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import matplotlib.pyplot as plt


mapData=OccupancyGrid()



class Navigation:
    """! Navigation node class.
    This class should serve as a template to implement the path planning and 
    path follower components to move the turtlebot from position A to B.
    """
    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.local_goal_pose = Pose()
        self.ttbot_pose = PoseStamped()
        self.current_heading = 0
        self.goal_heading = 0
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.distance_tolerance = 0.1
        self.done = 0
        self.cx = 0
        self.cy = 1000


    def init_app(self):
        """! Node intialization.
        @param  None
        @return None.
        """
        # ROS node initilization
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(100)
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=50)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=50)
        rospy.Subscriber('/map', OccupancyGrid, self.mapCallBack)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)



        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=50)
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)


    def image_callback(self,data):

        bridge = CvBridge()
        cv2img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        lower_white = np.array([0,0,240])    # mb_marker_buoy_red
        upper_white = np.array([255,15,255]) # world0 = sunny
        hsv_img = cv2.cvtColor(cv2img, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv_img, lower_white, upper_white)
        res_red = cv2.bitwise_and(cv2img, cv2img, mask= mask_white)
        

        m_red = cv2.moments(mask_white, False)
        try:
            blob_area_red = m_red['m00']
        except ZeroDivisionError:
            blob_area_red = 0
        if m_red["m00"] != 0:
            self.cX = int(m_red["m10"] / m_red["m00"])
            self.cY = int(m_red["m01"] / m_red["m00"])
        else :  
            self.cX = 100000
            self.cY = 100000

        cv2.waitKey(1)



    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        print("Goal pose x: ", self.goal_pose.pose.position.x)
        print("Goal pose y: ", self.goal_pose.pose.position.y)

        q0 = self.goal_pose.pose.orientation.x
        q1 = self.goal_pose.pose.orientation.y
        q2 = self.goal_pose.pose.orientation.z
        q3 = self.goal_pose.pose.orientation.w
 
        orientation_list = [q0,q1,q2,q3]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.goal_heading = yaw
        print("Goal yaw: ", self.goal_heading)

    def rotate(self,p, origin=(0, 0), degrees=0):
        angle = np.deg2rad(degrees)
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
        o = np.atleast_2d(origin)
        p = np.atleast_2d(p)
        return np.squeeze((R @ (p.T-o.T) + o.T).T)


    def mapCallBack(self,data):
       
        global mapData
        mapData=data

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        # TODO: MAKE SURE YOUR POSITION ESTIMATE IS GOOD ENOUGH.
        self.ttbot_pose = data.pose
        cov = data.pose.covariance


        """
        adding new functionality to get the current heading of the ttbot
        converting quaternion to euler
        """
        q0 = self.ttbot_pose.pose.orientation.x
        q1 = self.ttbot_pose.pose.orientation.y
        q2 = self.ttbot_pose.pose.orientation.z
        q3 = self.ttbot_pose.pose.orientation.w
        n = 2.0*(q3*q2+q0*q1)
        d = 1.0 - 2.0*(q1*q1+q2*q2)
        self.current_heading = math.degrees( math.atan2(n,d) )
        orientation_list = [q0,q1,q2,q3]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.current_heading = math.degrees(yaw)

    def callibrate(self):
        """
        callibrate ttbot to localize in rviz
        """
        cmd_vel = Twist()
        if (not(self.current_heading>-30 and self.current_heading<-5)):
            cmd_vel.angular.z = 0.25
            self.velocity_publisher.publish(cmd_vel)
            print("Callibrating, please wait")
            return 1
        else:
            cmd_vel.angular.z = 0
            self.velocity_publisher.publish(cmd_vel)
            print("Callibration done")
            return 0


    def calib(self, target=0):
        cmd_vel = Twist()
        dt = 1/100
        error = (target-self.current_heading)

        if (abs(error)>5.5):

            PID_obj_1 = PidController(0.01, 0.000, 0.0000, dt, -0.2, 0.2)

            cmd_vel.angular.z = PID_obj_1.step(error)
            print("calib Aligning, please wait")
            cmd_vel.linear.x = 0
            self.velocity_publisher.publish(cmd_vel)
            return 1
        else:
            cmd_vel.angular.z = 0
            self.velocity_publisher.publish(cmd_vel)
            print("Done aligning")
            return 0


    
    def euclidean_distance(self, goal_pose):
           """Euclidean distance between current pose and the goal."""
           return sqrt(pow((goal_pose.position.x - self.ttbot_pose.pose.position.x), 2) +
                       pow((goal_pose.position.y - self.ttbot_pose.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=0.6):

            return (constant * self.euclidean_distance(self.local_goal_pose))


    def steering_angle(self, goal_pose):
        return math.degrees(atan2(goal_pose.position.y - self.ttbot_pose.pose.position.y, goal_pose.position.x - self.ttbot_pose.pose.position.x))
  
    def angular_vel(self, goal_pose, constant=0.025):

         return (constant * (self.steering_angle(self.local_goal_pose) - self.current_heading))

    def move2goal(self):
    
        vel_msg = Twist()
        
        while self.euclidean_distance(self.local_goal_pose) >= self.distance_tolerance:
                print("self.cX ",self.cX )
                print("self.cy ",self.cY )

                zero_cx = 950

    
                cx = sqrt(pow((self.cX), 2) +pow((self.cY), 2))
                print("cx ",cx )
                stopx = 0
                if (self.cX<950 and self.cX >200 and self.cY<440 ):
                    stopx = 1
                if (self .cX > 950 and self.cX <2000 and self.cY<440):
                    stopx = 1



                if (self.cY >  430 and stopx == 0):
                    print("movinggggggggggggggggggggggggggggggggggggggggggg")


                    vel_msg.linear.x = self.linear_vel(self.local_goal_pose)
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = self.angular_vel(self.local_goal_pose)
        
                    # vel msg
                    self.velocity_publisher.publish(vel_msg)
                else : 
                    vel_msg.linear.x = 0.0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
        

                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = 0.0

                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

   
    def a_star_path_planner(self):
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        global mapData
        y_rviz_goal, x_rviz_goal = self.goal_pose.pose.position.y, self.goal_pose.pose.position.x
        y_rviz_current, x_rviz_current = self.ttbot_pose.pose.position.y, self.ttbot_pose.pose.position.x

        resolution = mapData.info.resolution
        Xstartx = mapData.info.origin.position.x
        Xstarty = mapData.info.origin.position.y
        width = mapData.info.width
        print("width , " ,width)
        print("Xstartx , " ,Xstartx)
        print("Xstarty , " ,Xstarty)
        print("y_rviz_goal , " ,y_rviz_goal)
        print("x_rviz_goal , " ,x_rviz_goal)

        Data = mapData.data


        index_y_g = int(	(floor((y_rviz_goal-Xstarty)/resolution))*200/480)
        index_X_g = int(	(floor((x_rviz_goal-Xstartx)/resolution))*200/480)

        index_y_c = int(	(floor((y_rviz_current-Xstarty)/resolution))*200/480)
        index_X_c = int(	(floor((x_rviz_current-Xstartx)/resolution))*200/480)

        origin=(100,100)
        
        new_points_g = self.rotate([(0,index_y_g)], origin=origin, degrees=180)

        new_points_c = self.rotate([(0,index_y_c)], origin=origin, degrees=180)

        index_y_c = int(new_points_c[1])

        index_y_g = int(new_points_g[1])

        st_pt = str(index_y_c) + ',' + str(index_X_c)
        end_pt = str(index_y_g) + ',' + str(index_X_g)

        print("Goal pose x: ", self.goal_pose.pose.position.x)
        print("Goal pose y: ", self.goal_pose.pose.position.y)
        print("index_y_G :  ,index_X_g: ",index_y_g,index_X_g)
        print("index_y_c :  ,index_X_c: ",index_y_c,index_X_c)

        path = trigger(st_pt,end_pt) 

        return path

    def converting_waypoints(self, path):

        global mapData

        resolution = mapData.info.resolution
        Xstartx = mapData.info.origin.position.x
        Xstarty = mapData.info.origin.position.y
        width = mapData.info.width
        
        
        transformed_path = []


        for index in range(len(path)-1):
            y1, x1 = path[index]
            y2, x2 = path[index+1]
            origin=(100,100)

            new_points_g = self.rotate([(0,y1)], origin=origin, degrees=180)
            y1 = int(new_points_g[1])
            new_points_g = self.rotate([(0,y2)], origin=origin, degrees=180)
            y2 = int(new_points_g[1])


            # resolution
            y_rviz = (y1*480/200)*resolution+Xstarty
            x_rviz = (x1*480/200)*resolution+Xstartx

            heading =  math.degrees(math.atan2( (y2-y1),(x2-x1) ))
            transformed_path.append((y_rviz, x_rviz, heading))
            

        y_rviz = self.goal_pose.pose.position.y
        x_rviz = self.goal_pose.pose.position.x
        heading = self.goal_heading
        transformed_path.append((y_rviz, x_rviz, heading))


        

        return transformed_path
        

    def path_follower(self,transformed_path):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        for y,x,heading in transformed_path:
   
            self.local_goal_pose.position.x = x
            self.local_goal_pose.position.y = y
            self.move2goal()
            print("new point")

        print("Done")
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.done = 1


    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """
        path_complete = False
        timeout = False
        path = self.a_star_path_planner() #call only once
        transformed_path = self.convert_waypoints(path)
   
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        for X in transformed_path:
            pose = PoseStamped()
            pose.pose.position.x = X[1]
            pose.pose.position.y = X[0]
            msg.poses.append(pose)

        self.path_pub.publish(msg)



        while not rospy.is_shutdown():
            if self.done == 0:
                
                self.path_follower(transformed_path)
            # print(ts2)

            self.rate.sleep() 
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))


if __name__ == "__main__":
    nav = Navigation(node_name='Navigation')
    nav.init_app()
    try:
        while (nav.callibrate()):
            pass
        while(nav.calib(0)):
            pass
    except rospy.ROSInterruptException:
        velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        print("program interrupted before completion", file=sys.stderr)



    try:
        nav.run()
    except rospy.ROSInterruptException:
        velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        print("program interrupted before completion", file=sys.stderr)