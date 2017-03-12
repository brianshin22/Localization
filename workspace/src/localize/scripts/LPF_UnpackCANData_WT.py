#-----------------------------------------------------------------------------
# University of California Berkeley
# Department of Mechanical Engineering
# Model Predictive Control Laboratory
#
# Capstone Project 2015-2016:
# Fault-Tolerant Control in Autonomous Driving - Team A (Localization)
#
# Team-Members : Leo Li
#                Joe Wu
#                Byunghyun Shin
#                Kilian Schindler
#
# Supervisors  : Francesco Borrelli (Prof. Dr.)
#                Ashwin Carvalho
#-----------------------------------------------------------------------------
#
# ABOUT THIS FILE
# This file contains the functions required to read in recorded CAN-data from
# the Full Winding Track.
#
# REVISION HISTORY
# [Mar 18, 2016] created
# [Mar 30, 2016] changed 'GPS_Pos0' to be a parameter of the 'LatLon_to_XY' 
#                function instead of a global constant
#
#-----------------------------------------------------------------------------


import scipy.io as sio
import numpy as np
import math as m 


STEER_CONSTANT = 14.5 #[-]
Earth_Rad   = 6371060 #[m]


def LatLon_to_XY(PosLat, PosLon, GPS_Pos0):
   
    GPS_Pos     = np.transpose(np.array([PosLon,PosLat]))

    GPS_Shifted         = GPS_Pos - np.kron(np.ones((len(PosLon),1)),GPS_Pos0)
    
    GPS_Shifted[:,0]    = GPS_Shifted[:,0]*m.cos(m.radians(GPS_Pos0[1]))
    GPS_Local_Pos       = GPS_Shifted*Earth_Rad*m.pi/180

    X   = GPS_Local_Pos[:,0]
    Y   = GPS_Local_Pos[:,1]

    return (X,Y)
	
def LatLon_to_XY_one(PosLat,PosLon,GPS_Pos0):
    
    GPS_Pos = np.array([PosLon,PosLat])
    GPS_Shifted = GPS_Pos - GPS_Pos0
    GPS_Shifted[0] = GPS_Shifted[0] * m.cos(m.radians(GPS_Pos0[1]))
    GPS_Local_Pos = GPS_Shifted * Earth_Rad * m.pi/180
    
    X = GPS_Local_Pos[0]
    Y = GPS_Local_Pos[1]
    return (X,Y)
    
    
def HeadingTransformation(AngleHeading):
    
    heading = -(AngleHeading) * m.pi/180 + m.pi/2
    
    for i in range(len(heading)):
        
        if heading[i] < 0:
            
            heading[i] = heading[i] + m.pi*2
    
    return heading
    
 
def loadCANdata(filepath, filter_frequency, GPS_Pos0):
    
    ###############################################################
    ## LOAD DESIGNATED CAN-DATA AND EXTRACT RELEVANT RAW SIGNALS ##
    ###############################################################
    
    data = sio.loadmat(filepath)
    
    raw_time = data['t'].flatten();
    
    raw_latitude = data['CAN2_LatitudeLongitude_PosLat'].flatten()
    raw_longitude = data['CAN2_LatitudeLongitude_PosLon'].flatten()
    (raw_X,raw_Y) = LatLon_to_XY(raw_latitude, raw_longitude, GPS_Pos0)
    
    raw_Psi = data['CAN2_HeadingPitchRoll_AngleHeading'].flatten() 
    raw_Psi = HeadingTransformation(raw_Psi)
    
    raw_whl_spd_rr = data['CAN1_WHL_SPD_WHL_SPD_RR'].flatten()
    raw_whl_spd_rl = data['CAN1_WHL_SPD_WHL_SPD_RL'].flatten()
    raw_V_straight = 0.5*(1/3.6)*(raw_whl_spd_rr + raw_whl_spd_rl)
    
    raw_steering_angle = data['CAN1_SAS1_SAS_Angle'].flatten()    
    raw_del_f = raw_steering_angle * (m.pi/(180*STEER_CONSTANT))
    
    raw_V_lateral = (-1.0) * data['CAN2_VelocityLevel_VelLateral'].flatten() 
    
    raw_dot_Psi = (m.pi/180) * data['CAN1_ESP2_YAW_RATE'].flatten()
    
    raw_a_r0 = data['CAN2_ME_Right_Lane_A_LaneMarkPosition_Rh_ME'].flatten()
    raw_a_r1 = data['CAN2_ME_Right_Lane_B_LaneMarkHeadAngle_Rh_ME'].flatten()
    raw_a_r2 = data['CAN2_ME_Right_Lane_A_LaneMarkModelA_Rh_ME'].flatten()
    raw_a_r3 = data['CAN2_ME_Right_Lane_B_LaneMarkModelDerivA_Rh_ME'].flatten()
    raw_a_r_quality = data['CAN2_ME_Right_Lane_A_LaneMarkQuality_Rh_ME'].flatten()
    
    raw_a_l0 = data['CAN2_ME_Left_Lane_A_LaneMarkPosition_Lh_ME'].flatten()
    raw_a_l1 = data['CAN2_ME_Left_Lane_B_LaneMarkHeadAngle_Lh_ME'].flatten()
    raw_a_l2 = data['CAN2_ME_Left_Lane_A_LaneMarkModelA_Lh_ME'].flatten()
    raw_a_l3 = data['CAN2_ME_Left_Lane_B_LaneMarkModelDerivA_Lh_ME'].flatten()    
    raw_a_l_quality = data['CAN2_ME_Left_Lane_A_LaneMarkQuality_Lh_ME'].flatten()
    
    
    #################################################################
    ## PARSE EXTRACTED SIGNALS TO MATCH FILTER ITERATION FREQUENCY ##
    #################################################################
    
    measurement_frequency = 1.0 / (raw_time[1] - raw_time[0])
    
    length = int(len(raw_time) * filter_frequency/measurement_frequency)

    time = np.zeros(length)
    X = np.zeros(length)
    Y = np.zeros(length)
    Psi = np.zeros(length)
    V_straight = np.zeros(length)
    del_f = np.zeros(length)
    V_lateral = np.zeros(length)
    dot_Psi = np.zeros(length)
    a_r0 = np.zeros(length)
    a_r1 = np.zeros(length)
    a_r2 = np.zeros(length)
    a_r3 = np.zeros(length)
    a_r_quality = np.zeros(length)
    a_l0 = np.zeros(length)
    a_l1 = np.zeros(length)
    a_l2 = np.zeros(length)
    a_l3 = np.zeros(length)
    a_l_quality = np.zeros(length)

    for i in range(length):
        
        sam = int(i*(measurement_frequency/filter_frequency))
        
        time[i] = raw_time[sam]
        X[i] = raw_X[sam]
        Y[i] = raw_Y[sam]
        Psi[i] = raw_Psi[sam]
        V_straight[i] = raw_V_straight[sam]
        del_f[i] = raw_del_f[sam]
        V_lateral[i] = raw_V_lateral[sam]
        dot_Psi[i] = raw_dot_Psi[sam]
        a_r0[i] = raw_a_r0[sam]
        a_r1[i] = raw_a_r1[sam]
        a_r2[i] = raw_a_r2[sam]
        a_r3[i] = raw_a_r3[sam]
        a_r_quality[i] = raw_a_r_quality[sam]
        a_l0[i] = raw_a_l0[sam]
        a_l1[i] = raw_a_l1[sam]
        a_l2[i] = raw_a_l2[sam]
        a_l3[i] = raw_a_l3[sam]
        a_l_quality[i] = raw_a_l_quality[sam]
    
    
    # processed measurement signals
    return (time, X, Y, Psi, V_straight, del_f, V_lateral, dot_Psi, \
            a_r0, a_r1, a_r2, a_r3, a_r_quality, a_l0, a_l1, a_l2, a_l3, a_l_quality) 
 
 