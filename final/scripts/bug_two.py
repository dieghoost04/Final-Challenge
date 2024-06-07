#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 
from sensor_msgs.msg import LaserScan   
import numpy as np 
import conditions as cnd
import gtg as gtg
import wf as wf


class AutonomousNav():  
    def __init__(self, x_target, y_target, index):  
        rospy.on_shutdown(self.cleanup)
        self.r = 0.05 # Radio de la rueda [m] 
        self.L = 0.19 # Separación entre ruedas [m] 

        self.x_init = 0.0
        self.y_init = 0.0
        self.flag_init = True

        self.x = 0.0 # Posición x del robot [m] 
        self.y = 0.0 # Posición y del robot [m] 
        self.theta = 0.0 # Ángulo del robot [rad] 

        self.x_target = x_target
        self.y_target = y_target
        
        # self.x_target = 0.5
        # self.y_target = 1.55
        self.goal_received = 1  # Bandera para indicar si se ha recibido la meta 
        self.lidar_received = 0 # Bandera para indicar si se ha recibido el escaneo láser 
        self.target_position_tolerance = 0.1 if index != 1 else 0.15# Tolerancia aceptable para la meta [m] 
        wf_distance = 0.20
        stop_distance = 0.08 # Distancia del obstáculo más cercano para detener el robot [m] 
        v_msg = Twist() # Velocidad deseada del robot  
        self.wr = 0 # Velocidad de la rueda derecha [rad/s] 
        self.wl = 0 # Velocidad de la rueda izquierda [rad/s] 
        self.current_state = 'GoToGoal' # Estado actual del robot 

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("scan", LaserScan, self.laser_cb)
        rospy.Subscriber("odom", Odometry, self.odom_cb)

        self.dt = 0.1
        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t 
        fwf = True
        d_t1 = 0.0
        d_t = 0.0
        theta_ao = 0.0
        theta_gtg = 0.0

        while not rospy.is_shutdown():
            print("X_robot: ",self.x)
            print("Y_robot: ",self.y)
            print("Y_robot: ",self.y)
            print("Theta: ",self.theta)
            print()
            print("x_ trobot: ",self.x_target)
            print("Y_ trobot: ",self.y_target)
            print("x_init: ",self.x_init)
            print("y_init: ",self.y_init)

            if self.lidar_received:
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) # Obtener el rango y ángulo del objeto más cercano 
                print("Estado actual",self.current_state )

                if self.current_state == 'Stop':     
                    if self.goal_received and closest_range > wf_distance and not self.at_goal():
                        print("Change to Go to goal from stop") 
                        self.current_state = "GoToGoal" 
                        self.goal_received = 0 
                    elif self.goal_received and closest_range < wf_distance and not self.at_goal():
                        print("Change to wall following from Stop") 
                        self.current_state = "WallFollower"
                    elif self.at_goal():
                        v_msg.linear.x = 0.0 
                        v_msg.angular.z = 0.0
                        break
                    else: 
                        v_msg.linear.x = 0.0 
                        v_msg.angular.z = 0.0 

                elif self.current_state == 'GoToGoal':  
                    if self.at_goal() or closest_range < stop_distance:  
                        print("Change to Stop from Go to goal")
                        print("At goal: ",self.at_goal())
                        self.current_state = "Stop"
                        
                    elif closest_range < wf_distance: 
                        self.current_state= "WallFollower"
                        self.wf_time = rospy.get_time()
                    else:        
                        v_gtg, w_gtg = gtg.compute_gtg_control(self.x_target, self.y_target, self.x, self.y, self.theta)   
                        v_msg.linear.x = v_gtg 
                        v_msg.angular.z = w_gtg      
                
                elif self.current_state == 'WallFollower': 
                    theta_gtg, theta_ao = cnd.compute_angles(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle)
                    d_t = np.sqrt((self.x_target - self.x)**2 + (self.y_target - self.y)**2)
                    print("Quit?: ",cnd.quit_wf_bug_two(theta_gtg, theta_ao, self.x_target, self.y_target, self.x, self.y, self.x_init, self.y_init))

                    if self.at_goal() or closest_range < stop_distance:
                        print("Change to Stop") 
                        self.current_state = "Stop" 
                        fwf = True

                    elif cnd.quit_wf_bug_two(theta_gtg, theta_ao, self.x_target, self.y_target, self.x, self.y, self.x_init, self.y_init) and (rospy.get_time() - self.wf_time > 5.0) or self.at_goal():
                        self.current_state= "WallFollower" 
                        print("Change to Go to goal from wall follower") 
                        self.current_state = "GoToGoal" 
                        fwf = True

                    else:
                        d_t1 = np.sqrt((self.x_target - self.x)**2 + (self.y_target - self.y)**2) if fwf else d_t1
                        clk_cnt = cnd.clockwise_counter(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle) if fwf else clk_cnt
                        fwf = False
                        v_wf, w_wf = wf.compute_wf_controller(closest_angle, closest_range, 0.20, clk_cnt, self.dt) 
                        v_msg.linear.x = v_wf
                        v_msg.angular.z = w_wf
            
            self.pub_cmd_vel.publish(v_msg) 
            rate.sleep()  

    def at_goal(self): 
        return np.sqrt((self.x_target - self.x)**2 + (self.y_target - self.y)**2) < self.target_position_tolerance 

    def get_closest_object(self, lidar_msg): 
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 

        return closest_range, closest_angle

    def laser_cb(self, msg):   
        self.lidar_msg = msg  
        self.lidar_received = 1  

    def wl_cb(self, wl):  
        self.wl = wl.data 

    def wr_cb(self, wr):  
        self.wr = wr.data  

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion(orientation_list)

        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion(orientation_list)

        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.x_init = self.x if self.flag_init else self.x_init
        self.y_init = self.y if self.flag_init else self.y_init

        self.flag_init = False


    def goal_cb(self, goal):  
        self.x_target = goal.data[0] 
        self.y_target = goal.data[1]
        self.goal_received = 1 

    def cleanup(self):  
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg)
    

if __name__ == "__main__":
    rospy.init_node("go_to_goal_with_obstacles1", anonymous=True)  
    #points = [[0.5 ,1.55], [1.38, 1.03], [2.05, 0.35], [2.97, 0.4], [0.5, 1.55]]
    points = [[0.5 ,1.55], [1.38, 1.03], [2.05, 0.35]]
    for i in range(len(points)):
        AutonomousNav(points[i][0], points[i][1], i)
        print()
        print("¡¡Llego al punto!!")
        print()
        pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        vel_msg = Twist() 
        pub_cmd_vel.publish(vel_msg)
        rospy.sleep(2)

