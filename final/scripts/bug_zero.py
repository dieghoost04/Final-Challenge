#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion 
import numpy as np
import conditions as cnd
import gtg as gtg
import wf as wf

#This class will make the puzzlebot move to a given goal 
class AutonomousNav():  
    def __init__(self, x, y):  
        rospy.on_shutdown(self.cleanup)

        ############ ROBOT CONSTANTS ################  

        self.r=0.05 #wheel radius [m] 
        self.L = 0.19 #wheel separation [m] 

        ############ Variables ############### 

        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = np.pi/2 #angle of the robot [rad] 

        ############ Variables ############### 

        self.x_target = x
        self.y_target = y

        self.goal_received = 1  #flag to indicate if the goal has been received 
        self.lidar_received = 0 #flag to indicate if the laser scan has been received 
        self.target_position_tolerance=0.12 #acceptable distance to the goal to declare the robot has arrived to it [m] 
        wf_distance = 0.2
        self.integral = 0.0
        self.prev_error = 0.0
        stop_distance = 0.08 # distance from closest obstacle to stop the robot [m] 
        v_msg = Twist() #Robot's desired speed  
        self.wr = 0 #right wheel speed [rad/s] 
        self.wl = 0 #left wheel speed [rad/s] 
        self.current_state = 'GoToGoal' #Robot's current state 

        ############################## INIT PUBLISHERS ##################################  
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("/wl", Float32, self.wl_cb)  
        rospy.Subscriber("/wr", Float32, self.wr_cb)  
        rospy.Subscriber("/scan", LaserScan, self.laser_cb)
        #rospy.Subscriber("puzzlebot_goal", Float32MultiArray, self.goal_cb) 
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        ############################### INIT NODE ######################################  
        freq = 20 
        rate = rospy.Rate(freq) #freq Hz  
        dt = 1.0/float(freq) #Dt is the time between one calculation and the next one 
        fwf = True
        d_t1 = 0.0
        d_t = 0.0
        theta_ao = 0.0
        theta_gtg = 0.0

        ############################### MAIN LOOP #######################################  
        while not rospy.is_shutdown():
            print("X_robot: ",self.x)
            print("Y_robot: ",self.y)
            print("Theta: ",self.theta)
            print()
            print("x_ trobot: ",self.x_target)
            print("Y_ trobot: ",self.y_target)
            print("Current State: ", self.current_state)
            if self.lidar_received:
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 

                if self.current_state == 'Stop': 
                    if self.goal_received and closest_range > wf_distance and not self.at_goal():
                        print("Change to Go to goal from stop") 
                        self.current_state = "GoToGoal" 
                        self.goal_received = 0 
                    elif self.goal_received and closest_range < wf_distance and not self.at_goal():
                        print("Change to wall following from Stop") 
                        self.current_state = "WallFollower"
                        self.wf_time = rospy.get_time()
                    elif self.at_goal(): 
                        v_msg.linear.x = 0.0 
                        v_msg.angular.z = 0.0
                        break
                    else: 
                        v_msg.linear.x = 0.0 
                        v_msg.angular.z = 0.0 

                elif self.current_state == 'GoToGoal':  
                    if self.at_goal() or  closest_range <  stop_distance:  
                        print("Change to Stop from Go to goal") 
                        print(self.x_target)
                        print(self.y_target)
                        self.current_state = "Stop" 
                    elif closest_range < wf_distance and not self.at_goal(): 
                        print("Change to wall following from Go to goal") 
                        self.current_state= "WallFollower" 
                        self.wf_time = rospy.get_time()

                    else:        
                        v_gtg, w_gtg = gtg.compute_gtg_control(self.x_target, self.y_target, self.x, self.y, self.theta)   
                        v_msg.linear.x = v_gtg 
                        v_msg.angular.z = w_gtg      

                elif self.current_state == 'WallFollower': 
                    theta_gtg, theta_ao = cnd.compute_angles(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle)
                    d_t = np.sqrt((self.x_target-self.x)**2+(self.y_target-self.y)**2)
                    if self.at_goal() or closest_range < stop_distance: 
                        print("Change to Stop") 
                        self.current_state = "Stop" 
                        fwf = True
                    elif cnd.quit_wf_bug_zero(theta_gtg, theta_ao, d_t, d_t1) and rospy.get_time() - self.wf_time > 2.0 and not self.at_goal():
                        print("Change to Go to goal from wall follower") 
                        self.current_state = "GoToGoal" 
                        fwf = True
                    else:
                        d_t1 = np.sqrt((self.x_target-self.x)**2+(self.y_target-self.y)**2)  if fwf else d_t1
                        clk_cnt = cnd.clockwise_counter(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle) if fwf else clk_cnt
                        fwf = False
                        v_wf, w_wf = wf.compute_wf_controller(closest_angle, closest_range, 0.18, clk_cnt, dt) 
                        v_msg.linear.x = v_wf
                        v_msg.angular.z = w_wf 

            self.pub_cmd_vel.publish(v_msg)  
            rate.sleep()  

    def at_goal(self): 
        return np.sqrt((self.x_target-self.x)**2+(self.y_target-self.y)**2)<self.target_position_tolerance 

    def get_closest_object(self, lidar_msg): 
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 

        return closest_range, closest_angle  

    def laser_cb(self, msg):   
        ## This function receives a message of type LaserScan   
        self.lidar_msg = msg  
        self.lidar_received = 1  

    def wl_cb(self, wl):  
        ## This function receives a the left wheel speed [rad/s] 
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data  

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion(orientation_list)

        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta)) 
    
    def goal_cb(self, goal):  
       # self.x_target = goal.pose.position.x 
       # self.y_target = goal.pose.position.y 

        self.goal_received=1 

    def cleanup(self):  
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

############################### MAIN PROGRAM ####################################  


if __name__ == "__main__":
    
    rospy.init_node("go_to_goal_with_obstacles1", anonymous=True)  
    points = [[0.5 ,1.55], [1.38, 1.03], [2.05, 0.35]]
    for i in range(len(points)):
        AutonomousNav(points[i][0], points[i][1])
        print()
        print("¡¡Llego al punto!!")
        print()
        pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        vel_msg = Twist() 
        pub_cmd_vel.publish(vel_msg)
        rospy.sleep(2)

