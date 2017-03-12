# -*- coding: utf-8 -*-
"""
Created on Sun Nov 15 13:32:48 2015

@author: Brian
"""

# filepath = '/Users/Leo/Documents/Localization/Data Files/CAN/CPG_Oval/t1.mat'
# filepath = '../../../../Data Files/CAN/CPG_Oval/t1.mat'

#import matplotlib.pyplot as plt

import scipy.io as sio
import numpy as np
import math as m 

STEER_CONSTANT = 14.5
Earth_Rad   = 6371060 #[m] 
GPS_Pos0    = [-118.0266,35.0536] # This value changes based on the test performed. This value is specifically for winding track 

def LatLon_to_XY(PosLat, PosLon, GPS_Pos0):
    
    n           = len(PosLon)
   
    GPS_Pos     = np.transpose(np.array([PosLon,PosLat]))

    GPS_Shifted         = GPS_Pos - np.kron(np.ones((n,1)),GPS_Pos0)
    
    GPS_Shifted[:,0]    = GPS_Shifted[:,0]*m.cos(m.radians(GPS_Pos0[1]))
    GPS_Local_Pos       = GPS_Shifted*Earth_Rad*m.pi/180

    X   = GPS_Local_Pos[:,0]
    Y   = GPS_Local_Pos[:,1]

    
    return (X,Y)
    
def HeadingTransformation(AngleHeading):
    heading = -(AngleHeading) * m.pi/180 + m.pi/2
    for i in range(len(heading)):
        if heading[i] < 0:
            heading[i] = heading[i] + m.pi*2
    return heading
    
def load(filepath):    
    
    data = sio.loadmat(filepath)
    
    time = data['t'].flatten();
    
    #numSamples = len(time)
    
    latitude = data['CAN2_LatitudeLongitude_PosLat'].flatten()
    longitude = data['CAN2_LatitudeLongitude_PosLon'].flatten()
    Psi = data['CAN2_HeadingPitchRoll_AngleHeading'].flatten()    
    
    whl_spd_rr = data['CAN1_L_WHL_SPD_WHL_SPD_RR'].flatten()
    whl_spd_rl = data['CAN1_L_WHL_SPD_WHL_SPD_RL'].flatten()
    
    steering_angle = data['CAN1_L_SAS1_SAS_Angle'].flatten()
    
    velocity = (whl_spd_rr + whl_spd_rl)/(2*3.6)
    
    del_f = steering_angle * (m.pi/(180*STEER_CONSTANT))
    
    (X,Y) = LatLon_to_XY(latitude,longitude,GPS_Pos0)
    Psi = HeadingTransformation(Psi)
    
    return (time, X,Y,Psi,velocity,del_f)


# do the lane measurements as well    
def load2(filepath):    
    
    data = sio.loadmat(filepath)
    
    time = data['t'].flatten();
    
    #numSamples = len(time)
    
    latitude = data['CAN2_LatitudeLongitude_PosLat'].flatten()
    longitude = data['CAN2_LatitudeLongitude_PosLon'].flatten()
    Psi = data['CAN2_HeadingPitchRoll_AngleHeading'].flatten()    
    
    whl_spd_rr = data['CAN1_L_WHL_SPD_WHL_SPD_RR'].flatten()
    whl_spd_rl = data['CAN1_L_WHL_SPD_WHL_SPD_RL'].flatten()
    
    steering_angle = data['CAN1_L_SAS1_SAS_Angle'].flatten()    
    
    velocity = (whl_spd_rr + whl_spd_rl)/2
    
    del_f = steering_angle * (m.pi/(180*STEER_CONSTANT))
    
    (X,Y) = LatLon_to_XY(latitude,longitude,GPS_Pos0)
    Psi = HeadingTransformation(Psi)
    
    a_l0 = data['CAN2_ME_Left_Lane_A_LaneMarkPosition_Lh_ME'].flatten()
    a_l1 = data['CAN2_ME_Left_Lane_B_LaneMarkHeadAngle_Lh_ME'].flatten()
    a_l2 = data['CAN2_ME_Left_Lane_A_LaneMarkModelA_Lh_ME'].flatten()
    a_l3 = data['CAN2_ME_Left_Lane_B_LaneMarkModelDerivA_Lh_ME'].flatten()    
    
    a_l_quality = data['CAN2_ME_Left_Lane_A_LaneMarkQuality_Lh_ME'].flatten()    
    
    a_r0 = data['CAN2_ME_Right_Lane_A_LaneMarkPosition_Rh_ME'].flatten()
    a_r1 = data['CAN2_ME_Right_Lane_B_LaneMarkHeadAngle_Rh_ME'].flatten()
    a_r2 = data['CAN2_ME_Right_Lane_A_LaneMarkModelA_Rh_ME'].flatten()
    a_r3 = data['CAN2_ME_Right_Lane_B_LaneMarkModelDerivA_Rh_ME'].flatten()
    
    a_r_quality = data['CAN2_ME_Right_Lane_A_LaneMarkQuality_Rh_ME'].flatten()
    a_l_quality = data['CAN2_ME_Left_Lane_A_LaneMarkQuality_Lh_ME'].flatten()
    
    return (time,X,Y,Psi,velocity,del_f,a_l0,a_l1,a_l2,a_l3,a_r0,a_r1,a_r2,a_r3,
            a_l_quality,a_r_quality)
    
    
