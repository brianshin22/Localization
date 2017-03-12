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
import LPF_UnpackCANData_WT as ld
import logging

import rospkg

logging.basicConfig(filename='/home/mpc/Localization/workspace/src/localize/log/test1.log',level=logging.DEBUG)

rospack = rospkg.RosPack()

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
frequency = rospy.get_param("/location_sender/frequency")


## 2.) Track Data (un-comment one option below)
################################################
#filepath = '../../../../Data Files/CAN/CPG_Oval/t1.mat'
#filepath = '../../../../Data Files/CAN/CPG_WindingTrack/WT_LSRL_clean.mat'


## 3.) Origin of GPS coordinate frame
#######################################
GPS_Pos0 = [-118.0266,35.0536]


## 4.) Model and Sensor Setting
################################
activate_DynamicModel = rospy.get_param("/location_sender/activate_DynamicModel")
activate_LaneMeas = rospy.get_param("/location_sender/activate_LaneMeas")


## 5.) GPS Acceptance Settings
###############################
GPS_accept_interval = rospy.get_param("/location_sender/GPS_accept_interval")
GPS_deny_interval = rospy.get_param("/location_sender/GPS_deny_interval")
activate_GPS_reset = rospy.get_param("/location_sender/activate_GPS_reset")


## 6.) Filter Parameters
##########################

# number of particles
num_particles = rospy.get_param("/location_sender/num_particles")

# bias of V_lateral signal (guessed by Ashwin, needs to be measured)
gps_V_lateral_bias = rospy.get_param("/location_sender/gps_V_lateral_bias")

# standard deviation of process noise for dt = 0.02s
straight_position_noise = rospy.get_param("/location_sender/straight_position_noise") # tuned ('run_LPF_OverallParamTuning.m')
turn_position_noise = rospy.get_param("/location_sender/turn_position_noise") # tuned ('run_LPF_OverallParamTuning.m')
lateral_velocity_noise = rospy.get_param("/location_sender/lateral_velocity_noise") # tuned ('run_LPF_OverallParamTuning.m')
turn_velocity_noise = rospy.get_param("/location_sender/turn_velocity_noise") # nuned ('run_LPF_OverallParamTuning.m')

# standard deviation of measurement noise
gps_X_noise = rospy.get_param("/location_sender/gps_X_noise") # tuned ('run_LPF_OverallParamTuning.m')
gps_Y_noise = rospy.get_param("/location_sender/gps_Y_noise") # tuned ('run_LPF_OverallParamTuning.m')
gps_Psi_noise = rospy.get_param("/location_sender/gps_Psi_noise") # tuned ('run_LPF_OverallParamTuning.m')
gps_V_lateral_noise = rospy.get_param("/location_sender/gps_V_lateral_noise") # tuned ('run_LPF_OverallParamTuning.m')
dot_Psi_noise = rospy.get_param("/location_sender/dot_Psi_noise") # tuned ('run_LPF_OverallParamTuning.m')
a_r0_noise = rospy.get_param("/location_sender/a_r0_noise") # tuned ('run_LPF_OverallParamTuning.m')
a_l0_noise = rospy.get_param("/location_sender/a_l0_noise") # tuned ('run_LPF_OverallParamTuning.m')
a_r1_noise = rospy.get_param("/location_sender/a_r1_noise") # tuned ('run_LPF_OverallParamTuning.m')
a_l1_noise = rospy.get_param("/location_sender/a_l1_noise") # tuned ('run_LPF_OverallParamTuning.m')

# vehicle parameters
mass = rospy.get_param("/location_sender/mass")  # [kg] mass of test vehicle
Izz = rospy.get_param("/location_sender/Izz")    # [kg*m^2] yaw moment of inertia
lf  = rospy.get_param("/location_sender/lf")     # [m] distance from center of mass to front axis
lr  = rospy.get_param("/location_sender/lr")     # [m] distance from center of mass to rear axis

# Pacejka tire model parameters
B_f = rospy.get_param("/location_sender/B_f")
B_r = rospy.get_param("/location_sender/B_r")
C_f = rospy.get_param("/location_sender/C_f")
C_r = rospy.get_param("/location_sender/C_r")
D_f = rospy.get_param("/location_sender/D_f")
D_r = rospy.get_param("/location_sender/D_r")
E_f = rospy.get_param("/location_sender/E_f")
E_r = rospy.get_param("/location_sender/E_r")

# thresholds for different functions
Psi_rel_threshold =  rospy.get_param("/location_sender/Psi_rel_threshold")     # maximum Psi_rel to be considered aligned [rad]
Lane_threshold = rospy.get_param("/location_sender/Lane_threshold")             # maximum acceptable distance to lanes [m]
DynamicModel_threshold = rospy.get_param("/location_sender/DynamicModel_threshold")     # minimum velocity to run dynamic model [m/s]

# fraction of particles receiving throughput measurements
frac_through = rospy.get_param("/location_sender/frac_through")


## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ ##
## ----------------- ADJUST PARTICLE FILTER SETTINGS HERE ----------------- ##
##############################################################################



# create GPS_available vector (determines access to GPS measurements) and initialize GPS_count
GPS_accept = np.concatenate((np.ones(GPS_accept_interval),np.zeros(GPS_deny_interval)),axis=0)
GPS_count = -1.



def callback1(pose):
    global X_meas, Y_meas, Psi_meas, GPS_available
    (X_meas,Y_meas) = ld.LatLon_to_XY_one(pose.x,pose.y,GPS_Pos0)
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
        
        GPS_count = (GPS_count+1) % (GPS_accept_interval+GPS_deny_interval)

        tic = rospy.get_rostime()
		
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
        
        saveline = str(toc) + "," + str(difference) + "," + str(X_meas) + "," + str(Y_meas) + "," + str(Psi_meas) + ","+str(X_estimate)+","+str(Y_estimate)+","+str(Psi_estimate)+"\n"
        filepath = rospack.get_path('localize') + "/log/"
        fileconfig = str(activate_DynamicModel) + "_" + str(activate_LaneMeas) + "_" + str(GPS_accept_interval) + "_" + str(GPS_deny_interval)
        filename = filepath + fileconfig + ".csv"
        saveFile=open(filename,"a")
        saveFile.write(saveline)
        saveFile.close()
        
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
