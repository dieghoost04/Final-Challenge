#conditions.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np

def is_point_near_segment(x1, y1, x2, y2, x3, y3, epsilon):
        a, b, c = calculate_line_equation(x2, y2, x3, y3)
        numerator = np.abs(a*x1 + b*y1 + c)
        denominator =  np.sqrt(a*a + b*b)
        distance = numerator / denominator
        
        if distance <= epsilon:
            return True , distance 
        return False, distance

def calculate_line_equation(x1, y1, x2, y2):
        a = y2 - y1
        b = x1 - x2
        c = x2 * y1 - x1 * y2

        return a, b, c

def quit_wf_bug_two(theta_gtg, theta_ao, x_target, y_target, x_robot, y_robot, x_init, y_init):
        n_segment, distance = is_point_near_segment(x_robot, y_robot, x_init, y_init, x_target, y_target, 0.10)
        print("Distancia al punto: ",distance)
        print("Theta a Goal: ", np.rad2deg(theta_gtg))
        print("Theta a Obstacle: ", np.rad2deg(theta_ao))
        print("Clear? : ", np.abs(theta_ao - theta_gtg) < np.pi/2)
        print()

        if (n_segment):# and (np.abs(theta_ao - theta_gtg) < np.pi/2): #nd (np.abs(theta_gtg) < np.pi/2):
            return True
        else:
            return False

def quit_wf_bug_zero(theta_gtg, theta_ao, d_t, d_t1):
        print("D(h1): ",d_t1)
        print("D: ",d_t)
        print("Clear? : ", np.abs(theta_ao - theta_gtg))
        print("Theta a Goal: ", np.rad2deg(theta_gtg))
        print("Theta a Obstacle: ", np.rad2deg(theta_ao))
        print()
        if (d_t < d_t1) and (np.abs(theta_ao - theta_gtg) < np.pi/2):
            return True
        else:
            return False
    
def clockwise_counter(x_target, y_target, x_robot, y_robot, theta_robot, closest_angle):
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta=theta_target-theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi/2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
    
        if np.abs(theta_fw - e_theta) <= np.pi/2:
            return 1
        else:
            return 0

def compute_angles(x_target, y_target, x_robot, y_robot, theta_robot, closest_angle):
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta=theta_target-theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        return e_theta, theta_ao