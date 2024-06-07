#!/usr/bin/env python

import rospy  
from geometry_msgs.msg import Twist  
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Float32 
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import tf2_ros #ROS package to work with transformations 
from geometry_msgs.msg import TransformStamped 
from tf.transformations import quaternion_from_euler 
import numpy as np 
from kf import KalmanFilter


np.set_printoptions(suppress=True) 

np.set_printoptions(formatter={'float': '{: 0.4f}'.format}) 


#This class will do the following: 

#   subscribe to the /wr and /wl topic  
#   publish to /odom   

class Localisation():  
    def __init__(self):  

        # first thing, init a node! 

        rospy.init_node('localisation') 
        

        ###******* INIT PUBLISHERS *******###  

        # Create the subscriber to wr and wl topics

        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 
     

        # Create ROS publishers 
        rospy.Subscriber("aruco_topic", Float32MultiArray, self.aruco_cb)

        self.odom_pub = rospy.Publisher('odom', Odometry ,queue_size=1) #Publisher to pose_sim topic 

        ############ ROBOT CONSTANTS ################  

        self.r=0.05 #puzzlebot wheel radius [m] 
        self.L = 0.19 #puzzlebot wheel separation [m] 
        self.dt = 0.1 # Desired time to update the robot's pose [s] 

        ############ Variables ############### 

        self.w = 0.0 # robot's angular speed [rad/s] 
        self.v = 0.0 #robot's linear speed [m/s] 
        self.x = 0.36 # 
        self.y = 0.9
        self.theta = 0.0  
        self.wr = 0.0
        self.wl = 0.0
        self.get_aruco = False
        

        
        ########################################### DEAD RECKONING ############################
    
        ######### CONTROL VARIABLES FOR Q ##########
        #self.wr_k = 0.15

        #self.wl_k = 0.2

        self.wr_k = 20
        self.wl_k = 30


        ###################

        self.z_covariance = np.zeros((3, 3))
        self.miu = np.array([[self.x], [self.y], [self.theta]])

        self.w_sigma_const = 0.5 * self.dt * self.r
        
        self.odom = Odometry() 

        KF = KalmanFilter(self.dt, self.miu)



        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t 
       # rate = rospy.Rate(20)

        while not rospy.is_shutdown(): 

         
            [self.v, self.w] = self.get_robot_vel(self.wr,self.wl)

            ####### DEAD RECKONING ########################

            rospy.loginfo("Prediccion")
            self.prediction(self.miu, self.z_covariance, self.wl, self.wr)


            ####################################################

            if self.get_aruco == True:
                rospy.loginfo("Correccion")
                self.correction_step(self.miu_pred, self.z_covariance_pred, 
                                     self.x_aruco, self.y_aruco,
                                     self.d_aruco,self.theta_aruco)
                #_, _ =  self.correction_step(self.miu, self.z_covariance, 
                #                             self.x_aruco, self.y_aruco,
                #                             self.d_aruco,self.theta_aruco)
                self.get_aruco = False
            else:
                self.step()
           
            self.update_robot_pose(self.v, self.w) 
            odom_msg = self.get_odom_stamped(self.miu[0][0], self.miu[1][0], self.miu[2][0], self.z_covariance, self.v, self.w) 
           # odom_msg.twist.twist.linear.x = self.v
           # odom_msg.twist.twist.angular.z = self.w

            print("x: ", self.miu[0][0])
            print("y: ", self.miu[1][0])
            print("theta: ", self.miu[2][0])        

            ######################################### 
            ######## Publish the data ################# 
            self.send_transform(odom_msg)
            self.odom_pub.publish(odom_msg) 

            rate.sleep() 

    def prediction(self, miu, z_covariance, wl, wr):
        sigma_k = np.array([[self.wr_k* wr, 0],
                                    [0,  self.wl_k * wl]])
    


        w_sigma = self.w_sigma_const * (np.array([[np.cos(miu[2][0]), np.cos(miu[2][0])],
                                                            [np.sin(miu[2][0]), np.sin(miu[2][0])],
                                                            [2 / self.L, 2 / self.L]]))
    
    
        Q_k =  w_sigma.dot(sigma_k).dot(w_sigma.T)

        
        H = np.array([[1, 0, -(((wl + wr) * 0.5 * self.r * self.dt )* np.sin(miu[2][0]))],
                                [0, 1,(((wl + wr) * 0.5 * self.r * self.dt )* np.cos(miu[2][0]))],
                                [0, 0, 1]])
        

        self.miu_pred = np.array([[miu[0][0] + (((wl + wr) *0.5 * self.r * self.dt)* np.cos(miu[2][0]))],
                                    [miu[1][0] + (((wl + wr) * 0.5 * self.r * self.dt )* np.sin(miu[2][0]))],
                                    [miu[2][0] + (((wr - wl)/self.L) * self.r * self.dt)]])

        self.z_covariance_pred = (H.dot(z_covariance).dot(H.T)) + Q_k

        #return miu, z_covariance
    
    def step(self):
        self.miu = self.miu_pred
        self.z_covariance = self.z_covariance_pred
    
    def correction_step(self, miu, z_covariance, x_aruco, y_aruco, d_aruco, theta_aruco):
        delta_x = x_aruco - miu[0][0] 
        delta_y =  y_aruco - miu[1][0] 
        
        print()
        print("Miu inicial: ", miu)

        p = delta_x ** 2 + delta_y ** 2
        I = np.identity(3)

        z = np.array([[d_aruco], 
                    [theta_aruco]])

        z_i = np.arctan2(delta_y, delta_x) - miu[2][0]
        z_i = np.arctan2(np.sin(z_i), np.cos(z_i))
        z_hat = np.array([[np.sqrt(p)], 
                        [z_i]])
        

        G = np.array([[-(delta_x / np.sqrt(p)), -delta_y / np.sqrt(p), 0],
                           [(delta_y / p), -(delta_x / p), -1]])
        
        R_k = np.array([[0.02, 0], 
                        [0, 0.02]])
	
        Z = G.dot(z_covariance).dot(G.T) + R_k

        K = np.array(z_covariance.dot(G.T).dot(np.linalg.inv(Z)))

        miu_final = miu + K.dot((z - z_hat))
        #miu_final[2][0]= theta_aruco-self

        covariance_final = (I - K.dot(G)).dot(z_covariance)

        
        print("Delta x: ",delta_x)
        print("Delta y: ",delta_y)
        print("P: ",p)
        print("z: ",z)
        print("G: ",G)
        print("R_k: ",R_k)
        print("Z: ",Z)
        print("K: ",K)
        print("miu final ",miu_final)
        print("Covariance Final ",covariance_final)


        print()

        return miu_final, covariance_final

    def wl_cb(self, msg): 
        self.wl = msg.data

    def wr_cb(self, msg): 
        self.wr = msg.data

    def get_robot_vel(self,wr,wl): 
        v = ((wl + wr)/2)*self.r 
        w = ((wr - wl) / self.L)*self.r

        return [v, w] 

    def aruco_cb(self, msg):
        self.id_aruco = msg.data[0]
        self.d_aruco = msg.data[1]
        self.theta_aruco = msg.data[2]
        self.meas = [self.d_aruco, self.theta_aruco]
        
        self.id_arucos(self.id_aruco)
        self.get_aruco = True

    def id_arucos(self, id):
        x_y = {702: [0, 0.80], 701: [0, 1.60], 703: [1.73, 0.80],
               704: [2.63, 0.39], 705: [2.85, 0], 706: [2.865,2.0],
               707: [1.735, 1.22], 705: [1.5, 0.5]}
        
        self.x_aruco = x_y[id][0]
        self.y_aruco = x_y[id][1]

    def get_odom_stamped(self, x, y, yaw,Sigma,Mu_v,Mu_w): 
        odom_stamped = Odometry() 

        odom_stamped.header.frame_id = "odom" 
        odom_stamped.child_frame_id = "base_link"
        odom_stamped.header.stamp = rospy.Time.now() 
        odom_stamped.pose.pose.position.x = x
        odom_stamped.pose.pose.position.y = y

        quat = quaternion_from_euler(0,0,yaw) 
        odom_stamped.pose.pose.orientation.x = quat[0]
        odom_stamped.pose.pose.orientation.y = quat[1]
        odom_stamped.pose.pose.orientation.z = quat[2]
        odom_stamped.pose.pose.orientation.w = quat[3]

        odom_array = np.array([[Sigma[0][0],Sigma[0][1],0,0,0,Sigma[0][2]],
                       [Sigma[1][0],Sigma[1][1],0,0,0,Sigma[1][2]],
                       [0,0,0,0,0,0],
                       [0,0,0,0,0,0],
                       [0,0,0,0,0,0],
                       [Sigma[2][0],Sigma[2][1],0,0,0,Sigma[2][2]]])
        
        odom_stamped.pose.covariance = odom_array.flatten().tolist()

        odom_stamped.twist.twist.linear.x = Mu_v
        odom_stamped.twist.twist.angular.z = Mu_w
        
        return odom_stamped 

    def send_transform(self, odom):
        self.tf_send = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        self.tf_send = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.frame_id = "map" 
        t.child_frame_id = "odom"
        t.header.stamp = rospy.Time.now() 
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0

        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        self.tf_send.sendTransform(t)
                    
        t.header.frame_id = "odom" 
        t.child_frame_id = "base_link"
        t.header.stamp = rospy.Time.now() 
        t.transform.translation.x = self.miu[0][0]
        t.transform.translation.y = self.miu[1][0]
        t.transform.translation.z = 0

        t.transform.rotation.x = odom.pose.pose.orientation.x
        t.transform.rotation.y = odom.pose.pose.orientation.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w

        self.tf_send.sendTransform(t)

    def update_robot_pose(self, v, w): 
        self.x = self.x + v * np.cos(self.theta)*(self.dt)
        self.y = self.y + v * np.sin(self.theta)*(self.dt)
        self.theta = self.theta + w*self.dt

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    Localisation()  