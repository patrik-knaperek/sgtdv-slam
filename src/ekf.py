# -*- coding: utf-8 -*-
"""
Created on Wed Dec  4 15:25:35 2019

@author: MartinLucan
"""

import numpy as np

def predict(mu_update, cov_update, pose):

   


    #Predict step of EKF filter
    #Input from ROS robot_localization package: topic /odom/filtered contains pose and covariance
    '''
    = len(mu)
    
    [v, w] = u
        
    if w == 0:
        motion = np.array([[v*np.cos(mu[2][0])],
                       [v*np.sin(mu[2][0])],
                       [0]])[]
    
        J = np.array([[0,0,-v*np.cos(mu[2][0])+v*np.cos(mu[2][0])],
                 [0,0,v*np.sin(mu[2][0])-v*np.sin(mu[2][0])],
                 [0,0,0]])  # Gxt
    
    else:
        motion = np.array([[-v/w*np.sin(mu[2][0])+v/w*np.sin(mu[2][0]+w*dt)],
                       [v/w*np.cos(mu[2][0])-v/w*np.cos(mu[2][0]+w*dt)],
                       [w*dt]])
    
        J = np.array([[0,0,-v/w*np.cos(mu[2][0])+v/w*np.cos(mu[2][0]+w*dt)],
                 [0,0,v/w*np.sin(mu[2][0])-v/w*np.sin(mu[2][0]+w*dt)],
                 [0,0,0]])  # Gxt
    
    
    F = np.append(np.eye(3),np.zeros((3,n-3)),axis=1)
    
    # Predict new state
    mu_bar = mu + (F.T).dot(motion)
   
    G = np.eye(n) + (F.T).dot(J).dot(F)
    
    # Predict new covariance
    cov_bar = G.dot(cov).dot(G.T) + (F.T).dot(Rt).dot(F)
    '''
    mu_predict = mu_update
    mu_predict[:3] = np.array([pose[:3]]).T      #replace x,y,theta with predict values

    cov_predict = cov_update 
    cov_predict[0][0] = pose[3]        #xx
    cov_predict[0][1] = pose[4]        #xy
    cov_predict[0][2] = pose[8]        #xtheta
    cov_predict[1][0] = pose[9]        #yx
    cov_predict[1][1] = pose[10]        #yy
    cov_predict[1][2] = pose[14]        #ytheta
    cov_predict[2][0] = pose[32]        #thetax
    cov_predict[2][1] = pose[33]        #thetay
    cov_predict[2][2] = pose[37]        #thetatheta

       
    #print('Predicted location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(mu_bar[0][0],mu_bar[1][0],mu_bar[2][0]))
    return mu_predict, cov_predict

def update(mu_predict,cov_predict,obs):
    #global mu_update
    N = len(mu_predict)
        
    #print np.shape(obs)
    
    for [r,theta,j] in obs:     #relative cone x,y,index
        j = int(j)
        #print r,theta,j,"\n"  
        
        # if landmark has not been observed before
        if cov_predict[2*j+3][2*j+3] >= 1e6 and cov_predict[2*j+4][2*j+4] >= 1e6:
            # define landmark estimate as current measurement
            # range, bearing to absolute x,y coordinates
            #print mu[0][0] + r*np.cos(theta+mu[2][0])
            mu_predict[2*j+3][0] = mu_predict[0][0] + r*np.cos(theta+mu_predict[2][0])
            mu_predict[2*j+4][0] = mu_predict[1][0] + r*np.sin(theta+mu_predict[2][0])
        
                 
                    
        # if landmark is static
        #if c_prob[j] >= 0.5:
        # compute expected observation
        delta = np.array([mu_predict[2*j+3][0] - mu_predict[0][0], mu_predict[2*j+4][0] - mu_predict[1][0]])
        q = delta.T.dot(delta)
        sq = np.sqrt(q)
        z_theta = np.arctan2(delta[1],delta[0])
        z_hat = np.array([[sq], [z_theta-mu_predict[2][0]]])
        
        # calculate Jacobian
        F = np.zeros((5,N))
        F[:3,:3] = np.eye(3)
        F[3,2*j+3] = 1
        F[4,2*j+4] = 1
        H_z = np.array([[-sq*delta[0], -sq*delta[1], 0, sq*delta[0], sq*delta[1]],
                        [delta[1], -delta[0], -q, -delta[1], delta[0]]], dtype='float')
        H = 1/q*H_z.dot(F)
        
        # calculate Kalman gain        
        K = cov_predict.dot(H.T).dot(np.linalg.inv(H.dot(cov_predict).dot(H.T)))  #+Qt
        
        # calculate difference between expected and real observation
        z_dif = np.array([[r],[theta]])-z_hat
        z_dif = (z_dif + np.pi) % (2*np.pi) - np.pi
        # update state vector and covariance matrix        
        mu_update = mu_predict + K.dot(z_dif)
        cov_update = (np.eye(N)-K.dot(H)).dot(cov_predict)
        
    #print('Updated location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(mu[0][0],mu[1][0],mu[2][0]))
    return mu_update, cov_update
    