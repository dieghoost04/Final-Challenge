#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

def compute_gtg_control(x_target, y_target, x_robot, y_robot, theta_robot): 
    #kvmax = 0.1
    #kwmax = 0.4

    #av = 0.5  
    #aw = 1.0

    kvmax = 0.5
    kwmax = 0.5

    av = 2
    aw = 2

    ed = np.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2) 
    
    theta_target = np.arctan2(y_target - y_robot, x_target - x_robot) 
    e_theta = theta_target - theta_robot 

    e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 

    if abs(e_theta) > 0.01: 
        kw = kwmax * (1 - np.exp(-aw * e_theta**2)) / abs(e_theta)
        w = kw * e_theta 
        #w = min(w, 0.20)

    else:
        w = 0

    if abs(e_theta) > np.pi / 7: 
        v = 0  
    else: 
        kv = kvmax * (1 - np.exp(-av * ed**2)) / abs(ed) 
        v = kv * ed
        v = min(v, 0.1)


    return v, w
