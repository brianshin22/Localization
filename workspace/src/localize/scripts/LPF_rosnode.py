#!/usr/bin/env python

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
# This file is going to be the ROS node executing the particle filter. However,
# it still needs to be implemented.
#
# REVISION HISTORY
# [Mar 18, 2016] created
# [Mar 30, 2016] work in progress - left to do:
#                   1.) read in V_lateral_meas and dot_Psi_mes
#                   2.) think about what to do if other meaurements than GPS
#                       are not available
#                   3.) what if computation takes more than allotted time
#
#-----------------------------------------------------------------------------



import rospy
import numpy as np
from std_msgs.msg import String, Float64, Int32
from localize.msg import LaneMeasure
from geometry_msgs.msg import Pose2D
import LPF_functions_rosnode as pf
import logging



logging.basicConfig(filename='/home/mpc/Localization/workspace/src/localize/log/test1.log',level=logging.DEBUG)



X_meas = 0.
Y_meas = 0.
Psi_meas = 0.
dot_Psi_meas = 0.
V_straight_meas = 0.
V_lateral_meas = 0.
del_f_meas = 0.

a_r0 = 0.
a_r1 = 0.
a_r2 = 0.
a_r3 = 0.
a_r_quality = 0.

a_l0 = 0.
a_l1 = 0.
a_l2 = 0.
a_l3 = 0.
a_l_quality = 0.

GPS_available = 0
V_straight_available = 0
del_f_available = 0
V_lateral_available = 0
dot_Psi_available = 0
RightLane_available = 0
LeftLane_available = 0



##############################################################################
## ----------------- ADJUST PARTICLE FILTER SETTINGS HERE ----------------- ##
## vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv ##


## 1.) Iteration Frequency [Hz]
################################
frequency = 50


## 2.) Track Data (un-comment one option below)
################################################
#filepath = '../../../../Data Files/CAN/CPG_Oval/t1.mat'
#filepath = '../../../../Data Files/CAN/CPG_WindingTrack/WT_LSRL_clean.mat'


## 3.) Origin of GPS coordinate frame
#######################################
GPS_Pos0 = [-118.0266,35.0536]


## 4.) Model and Sensor Setting
################################
activate_DynamicModel = 1
activate_LaneMeas = 1


## 5.) GPS Acceptance Settings
###############################
GPS_accept_interval = 10
GPS_deny_interval = 100
activate_GPS_reset = 1


## 6.) Filter Parameters
##########################

# number of particles
num_particles = 100

# bias of V_lateral signal (guessed by Ashwin, needs to be measured)
gps_V_lateral_bias = 0.02

# standard deviation of process noise for dt = 0.02s
straight_position_noise = 0.01 # tuned ('run_LPF_OverallParamTuning.m')
turn_position_noise = 0.0005 # tuned ('run_LPF_OverallParamTuning.m')
lateral_velocity_noise = 0.1 # tuned ('run_LPF_OverallParamTuning.m')
turn_velocity_noise = 0.0001 # tuned ('run_LPF_OverallParamTuning.m')

# standard deviation of measurement noise
gps_X_noise = 0.03 # tuned ('run_LPF_OverallParamTuning.m')
gps_Y_noise = 0.03 # tuned ('run_LPF_OverallParamTuning.m')
gps_Psi_noise = 0.002 # tuned ('run_LPF_OverallParamTuning.m')
gps_V_lateral_noise = 0.002 # tuned ('run_LPF_OverallParamTuning.m')
dot_Psi_noise = 0.005 # tuned ('run_LPF_OverallParamTuning.m')
a_r0_noise = 0.005 # tuned ('run_LPF_OverallParamTuning.m')
a_l0_noise = 0.005 # tuned ('run_LPF_OverallParamTuning.m')
a_r1_noise = 0.0005 # tuned ('run_LPF_OverallParamTuning.m')
a_l1_noise = 0.0005 # tuned ('run_LPF_OverallParamTuning.m')

# vehicle parameters
mass = 1840.0    # [kg] mass of test vehicle
Izz = 3477.0     # [kg*m^2] yaw moment of inertia
lf  = 1.105    # [m] distance from center of mass to front axis
lr  = 1.738    # [m] distance from center of mass to rear axis

# Pacejka tire model parameters
B_f = 0.1447
B_r = 0.1678
C_f = 0.1463
C_r = 0.1709
D_f = 1.9227e6
D_r = 2.2490e6
E_f = -0.3782
E_r = -0.4420

# thresholds for different functions
Psi_rel_threshold = 1e-6       # maximum Psi_rel to be considered aligned [rad]
Lane_threshold = 5             # maximum acceptable distance to lanes [m]
DynamicModel_threshold = 5     # minimum velocity to run dynamic model [m/s]

# fraction of particles receiving throughput measurements
frac_through = 0.1


## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ ##
## ----------------- ADJUST PARTICLE FILTER SETTINGS HERE ----------------- ##
##############################################################################



# create GPS_available vector (determines access to GPS measurements) and initialize GPS_count
GPS_accept = np.concatenate((np.ones(GPS_accept_interval),np.zeros(GPS_deny_interval)),axis=0)
GPS_count = -1.



def callback1(pose):
    global X_meas, Y_meas, Psi_meas, GPS_available
    X_meas = pose.x
    Y_meas = pose.y
    Psi_meas = pose.theta
    GPS_available = 1
    #rospy.loginfo("Received GPS measurement.")

def callback2(speed_straight):
    global V_straight_meas, V_straight_available
    V_straight_meas = speed_straight.data
    V_straight_available = 1
    #rospy.loginfo(rospy.get_caller_id() + "Velocity = %f\n", v)

def callback3(sas):
    global del_f_meas, del_f_available
    del_f_meas = sas.data
    del_f_available = 1
    #rospy.loginfo(rospy.get_caller_id() + "Steering Angle = %f\n", del_f)
    
def callback4(speed_lateral):
    global V_lateral_meas, V_lateral_available
    V_lateral_meas = speed_lateral.data
    V_lateral_available = 1
    #rospy.loginfo(rospy.get_caller_id() + "Steering Angle = %f\n", del_f)
    
def callback5(dot_Psi):
    global dot_Psi_meas, dot_Psi_available
    dot_Psi_meas = dot_Psi.data
    dot_Psi_available = 1
    #rospy.loginfo(rospy.get_caller_id() + "Steering Angle = %f\n", del_f)

def callback6(leftLane):
    global a_l0, a_l1, a_l2, a_l3, a_l_quality, LeftLane_available
    #left = (leftLane.a0,leftLane.a1,leftLane.a2,leftLane.a3,leftLane.quality)
    a_l0 = leftLane.a0
    a_l1 = leftLane.a1
    a_l2 = leftLane.a2
    a_l3 = leftLane.a3
    a_l_quality = leftLane.quality
    LeftLane_available = 1
    #rospy.loginfo(rospy.get_caller_id() + "Left lane measurements received\n")

def callback7(rightLane):
    global a_r0, a_r1, a_r2, a_r3, a_r_quality, RightLane_available
    #right = (rightLane.a0,rightLane.a1,rightLane.a2,rightLane.a3,rightLane.quality)
    a_r0 = rightLane.a0
    a_r1 = rightLane.a1
    a_r2 = rightLane.a2
    a_r3 = rightLane.a3
    a_r_quality = rightLane.quality
    RightLane_available = 1
    #rospy.loginfo(rospy.get_caller_id() + "Right lane measurements received\n")


def location_sender():
    global GPS_count, GPS_available, V_straight_available, del_f_available, V_lateral_available, dot_Psi_available, RightLane_available, LeftLane_available
    
    # initialize node
    rospy.init_node('location_sender', anonymous=True)
    
    # subscriptions
    # once location_sender receives the data it uses callback functions to update the variables
    rospy.Subscriber("pose", Pose2D, callback1)    
    rospy.Subscriber("speed_straight", Float64, callback2)
    rospy.Subscriber("sas", Float64, callback3)
    rospy.Subscriber("speed_lateral", Float64, callback4)
    rospy.Subscriber("dot_Psi", Float64, callback5)
    rospy.Subscriber("leftLane", LaneMeasure,callback6)
    rospy.Subscriber("rightLane",LaneMeasure,callback7)

    # publications
    pub = rospy.Publisher('location', Pose2D, queue_size=10)
    pub2 = rospy.Publisher('runtime', Int32, queue_size=10)
    
    # define filter rate
    rate = rospy.Rate(frequency) # 50hz
    
    
    
    # run localization particle filter
    while not rospy.is_shutdown():


        tic = rospy.get_rostime()
        
        
        GPS_count = (GPS_count+1) % (GPS_accept_interval+GPS_deny_interval)

        
        (X_estimate, Y_estimate, Psi_estimate) = pf.LPF_main( \
            1.0/frequency, num_particles, GPS_available*GPS_accept[GPS_count], \
            activate_LaneMeas, activate_DynamicModel, activate_GPS_reset, \
            X_meas, Y_meas, Psi_meas, dot_Psi_meas, \
            V_straight_meas, V_lateral_meas, del_f_meas, \
            a_r0, a_r1, a_r2, a_r3, a_r_quality, \
            a_l0, a_l1, a_l2, a_l3, a_l_quality, \
            straight_position_noise, turn_position_noise, lateral_velocity_noise, turn_velocity_noise, \
            gps_X_noise, gps_Y_noise, gps_Psi_noise, gps_V_lateral_noise, dot_Psi_noise, \
            a_r0_noise, a_l0_noise, a_r1_noise, a_l1_noise, \
            gps_V_lateral_bias, mass, Izz, lf, lr, \
            Psi_rel_threshold, Lane_threshold, DynamicModel_threshold, \
            B_f, B_r, C_f, C_r, D_f, D_r, E_f, E_r, \
            np.floor(frac_through*num_particles))
        
        
        toc = rospy.get_rostime()
        difference = (toc.nsecs - tic.nsecs)
        rospy.loginfo("%i\n",difference)
        
        
        
        #dif = str(difference)
        #logging.info("%s\n",dif)
        pose_estimate = Pose2D(X_estimate,Y_estimate,Psi_estimate) #Assign particle filter output to the message for publishing
        #rospy.loginfo(pose_out) # log the output from particle filter
        pub.publish(pose_estimate) # Publish the output from particle filter
        pub2.publish(difference)
        
        
        
        # reset GPS arrived flag to 0
        GPS_available = 0
        V_straight_available = 0
        del_f_available = 0
        V_lateral_available = 0
        dot_Psi_available = 0
        RightLane_available = 0
        LeftLane_available = 0
        
        

        # sleep for rest of time        
        rate.sleep()



if __name__ == '__main__':
    try:
        location_sender()
    except rospy.ROSInterruptException:
        pass
