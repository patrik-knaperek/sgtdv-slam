#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: MartinLucan
"""

import numpy as np
from robot import Robot
from plotmap import plotMap, plotEstimate, plotMeasurement
from ekf import predict, update
import rospy
import time
from sgtdv_msgs.msg import *


class Slam():

    def __init__(self):

        # init parameters

        self.mapsize = 50
        self.fov = 150
        self.Rt = 5*np.array([[0.1,0,0],
                            [0,0.1,0],
                            [0,0,0.01]])
        self.Qt = 5*np.array([[0.01,0],
                        [0,0.01]])

        self.dt = 1
        self.gamma = 0.0 # orientation error term
        self.pose_init = [0,0,0]
        self.steps = 25
        self.obs = []
        #self.camera_pose_arr = list()
        self.camera_pose = []
        self.n = 10

        self.inf = 1e6   
        self.mu = np.append(np.array([self.pose_init]).T,np.zeros((2*self.n,1)),axis=0)  #init only. Value is overwritten later
        self.mu_update = self.mu 
       
        
        self.cov = self.inf*np.eye(2*self.n+3)  #init only. Value is overwritten later
        self.cov[:3,:3] = np.zeros((3,3))
        self.cov_update = self.cov
        #self.c_prob = 0.5*np.ones((self.n,1)) 

        #ROS Subscribers callback init (topic name, msg type, callback)
        self.subscribe_robot_pose = rospy.Subscriber('camera_pose', CarState, self.subs_pose) #switch to PoseWithCovariance
        self.subscribe_cones = rospy.Subscriber('camera_cones', ConeArr, self.subs_cones) 
        
        #ROS Publishers init (topic name, msg type)
        self.pub_carstate = rospy.Publisher('slam_pose', CarState, queue_size=10)
        self.pub_map = rospy.Publisher('slam_map', ConeArr, queue_size=10)
    ''' 
      
    #plotMap(cone_coords_arr,pose_true,r1,mapsize)
    #plotEstimate(mu, cov, r1, mapsize, pose_true)
    '''
    #///////////////////////////////////////////////////////////////////////////////////////
    def subs_pose(self, msg):
    
        #global camera_pose
        #Subscription to camera poses
        self.camera_pose_x = msg.position.x
        self.camera_pose_y = msg.position.y
        self.camera_pose_theta = msg.yaw
        self.camera_pose_cov = msg.covariance
        #self.camera_pose_cov = self.camera_pose_cov_full[:6]
        self.camera_pose = [self.camera_pose_x, self.camera_pose_y, self.camera_pose_theta]
        self.camera_pose.extend(self.camera_pose_cov)
        #self.camera_pose_arr.append(self.camera_pose)

        #print "func1",camera_pose,"\n"
        #print self.camera_pose[2]
        return self.camera_pose
    #///////////////////////////////////////////////////////////////////////////////////////

    def subs_cones(self,msg):
        #Subscription to relative cone x,y positions, color, add index    
        rel_cone_coords_arr = []
        n = len(msg.cones)
        
        print n
        for a in range(n):
        
            self.rel_cone_coord_x = msg.cones[a].coords.x
            self.rel_cone_coord_y = msg.cones[a].coords.y
            self.cone_color = msg.cones[a].color 
            self.cone_index = a
            self.rel_cone = [self.rel_cone_coord_x, self.rel_cone_coord_y, self.cone_index]
            rel_cone_coords_arr.append(self.rel_cone)
            #a = a+1
               
      
                
    # print "camera_pose",camera_pose[2],"\n"
        obs = [] #np.empty((0,3))
        
        for landmark in rel_cone_coords_arr:
            #for every cone in rel_cone_coords_arr compute relative r,theta, append index   
            #print "camera pose:",self.camera_pose,"\n"                
            rel_angle = np.arctan2(landmark[1],landmark[0])                             #angle of cone position with reference in x axis
            meas_range = np.sqrt(np.power(landmark[1],2)+np.power(landmark[0],2))   #pytagor
            meas_bearing = (rel_angle - self.camera_pose[2] + np.pi)%(2*np.pi)-np.pi   #angle between car heading and r
            observ = [meas_range, meas_bearing, landmark[2]]    #relative range, bearing, index
            #observation = np.append(observation,[meas_range[0], meas_bearing[0], landmark[2]],axis=0)    #range, bearing, index
            obs.append(observ)
        #print obs
       # Call EKF predict and update functions       
        #print "update",self.mu_update,"\n"    
        
        self.mu_update = self.mu_update[0:2*n+3,:]       
        self.cov_update = self.cov_update[0:2*n+3,0:2*n+3]
        mu_predict, cov_predict = predict(self.mu_update, self.cov_update, self.camera_pose) 
        #print "cov_predict",cov_predict,"\n"

        #mu = np.append(self.mu,mu_predict,axis=1)
        #print "predict",mu_predict,"\n"
        #print "mu",mu,"\n"
                            
        self.mu_update, self.cov_update = update(mu_predict, cov_predict, obs)
        #mu = np.append(mu,self.mu_update,axis=1)
        
                     
        #return mu, self.mu_update, self.cov_update

    

       # i += 1 
        cone_count = ((len(self.mu_update))-3)/2  #actual number of cones
        print cone_count
        #extract cones x,y coordinates
        mu_new_cone_x = self.mu_update[3::2]
        mu_new_cone_y = self.mu_update[4::2]
      
        #pair cones x,y coord inates
        cone_pair = []
        for j in range(cone_count):
            cone_pair_xy = [mu_new_cone_x[j],mu_new_cone_y[j]]
            cone_pair.append(cone_pair_xy)
        #print "cone_pair",cone_pair,"\n"
                             
        a = CarState()
        a.position.x = self.mu_update[0]
        a.position.y = self.mu_update[1]
        a.yaw = self.mu_update[2]
        rospy.loginfo(a)
        self.pub_carstate.publish(a)

        arr = list()
        msgConeArr = ConeArr()
        for b in range(cone_count):
            msgPoint = Point2D()
            msgCone = Cone()
            msgPoint.x = cone_pair[b][0]
            msgPoint.y = cone_pair[b][1]
            msgCone.coords = msgPoint
            msgCone.color = 1
            arr.append(msgCone)
           
          # b.cones.coords.x = mu_new[3]
          #b.cones.coords.y = mu_new[4]
          # b.cones.color = 1
                      
        msgConeArr.cones = arr
        rospy.loginfo(msgConeArr)
        self.pub_map.publish(msgConeArr)
''' 
        #time.sleep(0.5)
        

'''
 
              
if __name__ == '__main__':
    rospy.init_node('slam_node', anonymous=True)
    SlamObject = Slam()
    mu_store = SlamObject.subs_cones
    #print mu_store
    #subscribe_robot_pose = rospy.Subscriber('camera_pose', CarState, SlamObject.subs_pose)
    #subscribe_cones = rospy.Subscriber('camera_cones', ConeArr, SlamObject.subs_cones)
    #rate = rospy.Rate(10) # 10hz
    rospy.spin()
    
       

  







