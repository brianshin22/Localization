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
# This file is a testscript to run a particle filter that estimates of the 
# car's current position and orientation based on currently available 
# measurements. Possible measurement types are 
#   (i)  GPS measurements
#   (ii) Lane measurements
# The code of the executed functions can be found in 'LPF_functions.py'.
#
# REVISION HISTORY
# [Mar 18, 2016] created
# [Mar 30, 2016] added GPS_Pos0 as filter setting
#
#-----------------------------------------------------------------------------



import numpy as np
import LPF_functions as pf
import LPF_UnpackCANData_WT as ld
import matplotlib.pyplot as plt
import writeCSV as wr
import datetime



##############################################################################
## ----------------- ADJUST PARTICLE FILTER SETTINGS HERE ----------------- ##
## vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv ##


## 1.) Iteration Frequency [Hz]
################################
frequency = 50


## 2.) Track Data (un-comment one option below)
################################################
#filepath = '../../../../Data Files/CAN/CPG_Oval/t1.mat'
filepath = '../../../../Data Files/CAN/CPG_WindingTrack/WT_LSRL_clean.mat'


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



#########################################
## INITIALIZE DATA FOR PARTICLE FILTER ##
#########################################

# load recorded measurement data
(time_meas, X_meas, Y_meas, Psi_meas, V_straight_meas, del_f_meas, V_lateral_meas, dot_Psi_meas, \
a_r0, a_r1, a_r2, a_r3, a_r_quality, a_l0, a_l1, a_l2, a_l3, a_l_quality) = ld.loadCANdata(filepath,frequency, GPS_Pos0)

# determine number of time steps contained on measurement data
num_timesteps = len(time_meas)

# initialize vectors to contain the estimated car pose
X_estimate = np.zeros(num_timesteps)
Y_estimate = np.zeros(num_timesteps)
Psi_estimate = np.zeros(num_timesteps)

# create GPS_available vector (determines access to GPS measurements) and initialize GPS_count
GPS_available = np.concatenate((np.ones(GPS_accept_interval),np.zeros(GPS_deny_interval)),axis=0)
GPS_count = (-1)



#############################
## ITERATE PARTICLE FILTER ##
#############################

#start = datetime.datetime.now()

for i in range(num_timesteps):
    
    GPS_count = (GPS_count+1) % (GPS_accept_interval+GPS_deny_interval)
    
    start = datetime.datetime.now()
    
    (X_estimate[i], Y_estimate[i], Psi_estimate[i]) = \
    pf.LPF_main(i, 1.0/frequency, num_particles, GPS_available[GPS_count], \
    activate_LaneMeas, activate_DynamicModel, activate_GPS_reset, \
    X_meas[i], Y_meas[i], Psi_meas[i], dot_Psi_meas[i], \
    V_straight_meas[i], V_lateral_meas[i], del_f_meas[i], \
    a_r0[i], a_r1[i], a_r2[i], a_r3[i], a_r_quality[i], \
    a_l0[i], a_l1[i], a_l2[i], a_l3[i], a_l_quality[i], \
    straight_position_noise, turn_position_noise, lateral_velocity_noise, turn_velocity_noise, \
    gps_X_noise, gps_Y_noise, gps_Psi_noise, gps_V_lateral_noise, dot_Psi_noise, \
    a_r0_noise, a_l0_noise, a_r1_noise, a_l1_noise, \
    gps_V_lateral_bias, mass, Izz, lf, lr, \
    Psi_rel_threshold, Lane_threshold, DynamicModel_threshold, \
    B_f, B_r, C_f, C_r, D_f, D_r, E_f, E_r, \
    np.floor(frac_through*num_particles))
    
    end = datetime.datetime.now()
    print(end-start)
    
#end = datetime.datetime.now()
#print(end-start)
 


##########################################
## COMPUTE MEASURES OF ESTIMATION ERROR ##
##########################################

# compute error RMS and MAX of D = sqrt(X^2+Y^2)
pos_error_rms = np.sqrt( (1.0/num_timesteps) * np.sum( (X_estimate-X_meas)**2+(Y_estimate-Y_meas)**2 ) )
pos_error_max = np.max(np.sqrt((X_estimate-X_meas)**2 + (Y_estimate-Y_meas)**2))

# compute error RMS and MAX of Psi
Psi_error_rms = np.sqrt( (1.0/num_timesteps) * np.sum( (Psi_estimate-Psi_meas)**2 ) )
Psi_error_max = np.max(np.abs(Psi_estimate-Psi_meas))

# Combine both into one error vector
EstimationError = [pos_error_rms, pos_error_max, Psi_error_rms, Psi_error_max]
print(EstimationError)



##################################################
## PLOT GPS POSITION AGAINST ESTIMATED POSITION ##
##################################################

plt.plot(X_meas,Y_meas,'r.')
plt.plot(X_estimate,Y_estimate, 'b.')
plt.axes().set_aspect('equal', 'datalim')
plt.show()



######################################
## SAVE POSE ESTIMATES IN .CSV FILE ##
######################################

fileout = '../log/test_OvalTrack_t1.csv'
wr.writeCSV(fileout, time_meas, X_estimate, Y_estimate, Psi_estimate)
