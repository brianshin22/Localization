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
# This file contains Python code implementing the functions required to run a 
# particle filter that estimates of the car's current position and orientation 
# based on currently available measurements. Possible measurement types are 
#   (i)  GPS measurements
#   (ii) Lane measurements
#
# REVISION HISTORY
# [Mar 18, 2016] created
# [Mar 30, 2016] replaced 'GPS_count' by local 'LPF_main.previous_GPS_missing'
#                to keep track of whether GPS was available at last time step,
#                and ensured that lane orientations are at +Inf/-Inf if lane 
#                is not detected
#
#-----------------------------------------------------------------------------



import numpy as np
import math as m
from scipy.stats import norm
import random as rnd



def quad_poly_roots(a,b,c):
#-----------------------------------------------------------------------------
# ABOUT THIS FUNCTION
# Given a set of N second order polynomial equations of the form 
# { a*x^2 + b*x + c = 0 }, this function computes the two corresponding roots 
# for each equation and returns them. In the case of complex conjugate roots, 
# this functions sets both roots to zero.
#
# INPUT
# a : Nx1 vector of coefficients a in the set { a*x^2 + b*x + c = 0 }
# b : Nx1 vector of coefficients b in the set { a*x^2 + b*x + c = 0 }
# c : Nx1 vector of coefficients c in the set { a*x^2 + b*x + c = 0 }
#
# OUTPUT
# roots : Nx1 vector containing roots of respective second order equation
#         (if roots are complex conjugate, they are both set to zero)
#-----------------------------------------------------------------------------
    
    # initialize roots vector
    roots = np.zeros([len(a),2])
    
    # compute determinant
    det = b**2-4.0*a*c
    
    # compute the two roots of a*x^2 + b*x + c = 0
    roots[det>=0,0] = 1.0/(2.0*a[det>=0]) * (-b[det>=0] + np.sqrt(b[det>=0]**2-4.0*a[det>=0]*c[det>=0]))
    roots[det>=0,1] = 1.0/(2.0*a[det>=0]) * (-b[det>=0] - np.sqrt(b[det>=0]**2-4.0*a[det>=0]*c[det>=0]))
    
    return roots



def LPF_initialize(num_particles, X_meas, Y_meas, Psi_meas, V_lateral_meas, dot_Psi_meas, gps_V_lateral_bias, \
    gps_X_noise, gps_Y_noise, gps_Psi_noise, gps_V_lateral_noise, dot_Psi_noise):

#-----------------------------------------------------------------------------
# ABOUT THIS FUNCTION
# Given a set of sensor measurements and the respective sensor noise values, 
# this function initializes the particles according to a normal distribution 
# around the sensor measurement values and returns these particles.
#
# INPUT
# num_particles       : number of particles to be initialized
# X_meas              : X measurement [m]
# Y_meas              : Y measurement [m]
# Psi_meas            : Psi measurement [rad]
# V_lateral_meas      : lateral velocity measurement [m/s]
# dot_Psi_meas        : dot_Psi measurement [rad/s]
# gps_V_lateral_bias  : bias of lateral velocity signal [m/s]
# gps_X_noise         : std. dev. of X measurement [m]
# gps_Y_noise         : std. dev. of Y measurement [m]
# gps_Psi_noise       : std. dev. of Psi measurement [rad]
# gps_V_lateral_noise : std. dev. of lateral velocity measurement [m/s]
# dot_Psi_noise       : std. dev. of dot_Psi measurement [rad/s]
#
# OUTPUT
# particles : (num_particles)x11 matrix containing initialized particle values
#             having the following structure: [X, Y, Psi, a0_RL, a0_LL, a1_RL, 
#             a1_LL, V_lateral, dot_Psi, V_lateral_bias, dX, dY, dPsi, Probability]
#-----------------------------------------------------------------------------
    
    # normal distributions around first available measurements    
    init_X_values = X_meas*np.ones(num_particles) + np.random.normal(0.0, gps_X_noise, num_particles)
    init_Y_values = Y_meas*np.ones(num_particles) + np.random.normal(0.0, gps_Y_noise, num_particles)
    init_Psi_values = Psi_meas*np.ones(num_particles) + np.random.normal(0.0, gps_Psi_noise, num_particles)
    init_V_lateral_values = (V_lateral_meas*np.ones(num_particles) - gps_V_lateral_bias*np.ones(num_particles)) + np.random.normal(0.0,gps_V_lateral_noise,num_particles)
    init_dot_Psi_values = dot_Psi_meas*np.ones(num_particles) + np.random.normal(0.0,dot_Psi_noise,num_particles)
    init_V_lateral_bias_values = gps_V_lateral_bias * np.ones(num_particles)
    
    # initialize particles
    particles = np.zeros((14,num_particles))                                        
    particles = np.concatenate(([init_X_values],[init_Y_values],[init_Psi_values],
                                [float('inf')*np.ones(num_particles)],[float('-inf')*np.ones(num_particles)],
                                [float('inf')*np.ones(num_particles)],[float('-inf')*np.ones(num_particles)],
                                [init_V_lateral_values],[init_dot_Psi_values],[init_V_lateral_bias_values],
                                [np.zeros(num_particles)],[np.zeros(num_particles)],[np.zeros(num_particles)],
                                [np.ones(num_particles)/num_particles]),axis=0)
    particles = np.transpose(particles)
    
    # return initialized particles
    return particles
    
    

def LPF_propagate(particles, num_particles, dt, V_straight_meas, del_f_meas, \
    a_r0, a_r1, a_r2, a_r3, a_r_quality, a_l0, a_l1, a_l2, a_l3, a_l_quality, \
    straight_position_noise, turn_position_noise, lateral_velocity_noise, turn_velocity_noise, \
    Psi_rel_threshold, Lane_threshold, DynamicModel_threshold, \
    activate_LaneMeas, activate_LidarMeas, activate_DynamicModel, \
    mass, Izz, lf, lr, B_f, B_r, C_f, C_r, D_f, D_r, E_f, E_r):
        
#-----------------------------------------------------------------------------
# ABOUT THIS FUNCTION (NOT USED IN FINAL IMPLEMENTATION!!)
# Given the current particles, motion model setting, the measurement intake 
# setting, this function carries out the propagation step of the particle 
# filter in a non-vectorized fashion and returns the propagated particles.
#
# INPUT
# particles               : current particles
# num_particles           : number of particles to be initialized
# dt                      : propagation time increment [s]
# V_straight_meas         : straight velocity measurement [m/s]
# del_f_meas              : steering angle measurement [rad]
# a_r0                    : a_0 coefficient of right lane [m]
# a_r1                    : a_1 coefficient of right lane [m^(-1)]
# a_r2                    : a_2 coefficient of right lane [m^(-2)]
# a_r3                    : a_3 coefficient of right lane [m^(-3)]
# a_r_quality             : measurement quality of right lane
# a_l0                    : a_0 coefficient of left lane [m]
# a_l1                    : a_1 coefficient of left lane [m^(-1)]
# a_l2                    : a_2 coefficient of left lane [m^(-2)]
# a_l3                    : a_3 coefficient of left lane [m^(-3)]
# a_l_quality             : measurement quality of left lane
# straight_position_noise : std. dev. of process noise on straight position
# turn_position_noise     : std. dev. of process noise on Psi
# lateral_velocity_noise  : std. dev. of process noise on lateral velocity
# turn_velocity_noise     : std. dev. of process noise on dot_Psi
# Psi_rel_threshold       : threshold for considering two frames aligned
# Lane_threshold          : maximal acceptable distance from lanes
# DynamicModel_threshold  : minimum velocity to apply dynamic bicycle model
# activate_LaneMeas       : boolean activating lane measurements
# activate_LidarMeas      : boolean activating LIDAR measurements
# activate_DynamicModel   : boolean activating dynamic bicycle model
# mass                    : car mass [kg]
# Izz                     : car moment of inertia at center of mass [kg*m^2]
# lf                      : distance center of mass to front wheel axis [m]
# lr                      : distance center of mass to rear wheel axis [m]
# B_f                     : tire force coefficient for Pacejka tire model
# B_r                     : tire force coefficient for Pacejka tire model
# C_f                     : tire force coefficient for Pacejka tire model
# C_r                     : tire force coefficient for Pacejka tire model
# D_f                     : tire force coefficient for Pacejka tire model
# D_r                     : tire force coefficient for Pacejka tire model
# E_f                     : tire force coefficient for Pacejka tire model
# E_r                     : tire force coefficient for Pacejka tire model
#
# OUTPUT
# particles : (num_particles)x11 matrix containing propagated particle values
#-----------------------------------------------------------------------------
    
    # threshold for imaginary root detection
    imag_threshold = 1e-6
    
    # extract certain particle values
    Psi_vals = particles[:,2]
    V_lateral_vals = particles[:,7]
    dot_Psi_vals = particles[:,8]  
    

    ######################################################
    ## PROPAGATE PARTICLES USING SPECIFIED MOTION MODEL ##
    ######################################################

    if (activate_DynamicModel and V_straight_meas >= DynamicModel_threshold):
    
        # compute differential position increments using dynamic bicycle model
        dX = V_straight_meas*np.cos(Psi_vals) - V_lateral_vals*np.sin(Psi_vals)
        dY = V_straight_meas*np.sin(Psi_vals) + V_lateral_vals*np.cos(Psi_vals)

        # compute relative displacement of particles in global frame
        X_rel_GF = dt*dX + np.random.normal(0.0,straight_position_noise,num_particles)*np.cos(Psi_vals+np.arctan(V_lateral_vals/V_straight_meas))
        Y_rel_GF = dt*dY + np.random.normal(0.0,straight_position_noise,num_particles)*np.sin(Psi_vals+np.arctan(V_lateral_vals/V_straight_meas))
        Psi_rel_GF = dt*dot_Psi_vals + np.random.normal(0.0,turn_position_noise,num_particles)

        # compute tire forces according to Pacejka tire model
        alpha_f = del_f_meas * np.ones(num_particles) - (V_lateral_vals+dot_Psi_vals*lf)/V_straight_meas
        alpha_r = (-1.0) * (V_lateral_vals-dot_Psi_vals*lr)/V_straight_meas
        F_y_f = 2.0*D_f * np.sin(C_f*np.arctan((1.0-E_f)*B_f*alpha_f + E_f*np.arctan(B_f*alpha_f))) * np.cos(del_f_meas)
        F_y_r = 2.0*D_r * np.sin(C_r*np.arctan((1.0-E_r)*B_r*alpha_r + E_r*np.arctan(B_r*alpha_r)))

        # compute differential velocity increments using dynamic bicycle model
        ddot_Psi = 2.0/Izz * (lf*F_y_f-lr*F_y_r)
        ddot_Y = (-1.0) * dot_Psi_vals * V_straight_meas + 2.0/mass * (F_y_f*np.cos(del_f_meas)+F_y_r)

        # compute new particle velocity values
        particles[:,7] = particles[:,7] + dt*ddot_Y + np.random.normal(0.0,lateral_velocity_noise,num_particles)
        particles[:,8] = particles[:,8] + dt*ddot_Psi + np.random.normal(0.0,turn_velocity_noise,num_particles)
    
    else:
    
        # compute differential increments using kinematic bicycle model
        beta = m.atan(lr*m.tan(del_f_meas)/(lf+lr))*np.ones(num_particles)
        dX = np.multiply(V_straight_meas,np.cos(Psi_vals + beta))
        dY = np.multiply(V_straight_meas,np.sin(Psi_vals + beta))
        dPsi = np.multiply(V_straight_meas,np.sin(beta)/lr)

        # compute relative displacement of particles in global frame
        X_rel_GF = dt*dX + np.random.normal(0.0,straight_position_noise,num_particles) * np.cos(Psi_vals+beta)
        Y_rel_GF = dt*dY + np.random.normal(0.0,straight_position_noise,num_particles) * np.sin(Psi_vals+beta)
        Psi_rel_GF = dt*dPsi + np.random.normal(0.0,turn_position_noise,num_particles)
       
    # compute propagated particles as current particles plus displacement
    particles[:,0] = particles[:,0] + X_rel_GF
    particles[:,1] = particles[:,1] + Y_rel_GF
    particles[:,2] = (particles[:,2] + Psi_rel_GF) % (2.0*m.pi)
    
    
    ########################################################
    ## IF ACTIVATED, UPDATE LANE DISTANCE AND ORIENTATION ##
    ########################################################

    if (activate_LaneMeas):
    
        # reset expected a0 and a1 coefficient of the right lane for all particles
        particles[:,3] = np.ones(num_particles)*float('inf')
        particles[:,5] = np.ones(num_particles)*float('inf')
    
        # reset expected a0 and a1 coefficient of the left lane for all particles
        particles[:,4] = np.ones(num_particles)*float('-inf')
        particles[:,6] = np.ones(num_particles)*float('-inf')
    
        if (a_r_quality >= 2 or a_l_quality >= 2):
        
            # transform relative displacement of particles from global frame to local frame of current particles
            X_rel_CLF = X_rel_GF*np.cos(particles[:,2]) + Y_rel_GF*np.sin(particles[:,2])
            Y_rel_CLF = X_rel_GF*np.sin(particles[:,2]) - Y_rel_GF*np.cos(particles[:,2])
            Psi_rel_CLF = (-1) * Psi_rel_GF
    

        ############################################
        ## UPDATE RIGHT LANE DISTANCE INFORMATION ##
        ############################################
    
        if (a_r_quality >= 2):
        
            # initialize coefficients for right lane polynomial and its derivative
            pr = [a_r3,a_r2,a_r1,a_r0]
            dpr = [3*a_r3, 2*a_r2, a_r1]
        
            # go through all particles
            for i1 in range(num_particles):
            
                # if frames of current and propagated particles are almost aligned
                if np.abs(Psi_rel_CLF[i1]) < Psi_rel_threshold:
                
                    # compute expected a0 and a1 coefficients of right lane
                    particles[i1,3] = np.polyval(pr,X_rel_CLF[i1])
                    particles[i1,5] = np.polyval(dpr,X_rel_CLF[i1])
                
                # if frames of current and propagated particles are disaligned
                else:
                
                    # compute roots of intersection between lane polynomial and
                    # transversal line of car (which are both expressed in local 
                    # frame of current particle)
                    roots_right = np.roots([a_r3, a_r2, a_r1+(1/np.tan(Psi_rel_CLF[i1])), a_r0-(1/np.tan(Psi_rel_CLF[i1]))*(X_rel_CLF[i1]-Y_rel_CLF[i1])])
                
                    # go through all roots
                    for i2 in range(len(roots_right)):
                    
                        # if computed root is real (i.e. imaginary part is zero)
                        if abs(roots_right[i2].imag) < imag_threshold:
                        
                            # compute (and store as candidate) the coordinates of
                            # the intersection in local frame of current particle
                            Ar_x_cand_CLF = roots_right[i2].real
                            Ar_y_cand_CLF = np.polyval(pr,roots_right[i2].real)
                        
                            # compute the y-coordinate of the intersection
                            # in local frame of the propagated particle
                            Ar_y_cand_PLF = -np.sin(Psi_rel_CLF[i1])*(Ar_x_cand_CLF - X_rel_CLF[i1]) + np.cos(Psi_rel_CLF[i1])*(Ar_y_cand_CLF-Y_rel_CLF[i1])
                        
                            # check whether y-coordinate is on the right-hand side of the car and whether it is closer 
                            # to the car than any other previously computed candidate
                            if (0 <= Ar_y_cand_PLF and Ar_y_cand_PLF < particles[i1,3] and Ar_y_cand_PLF < Lane_threshold):  
                                
                                # save expected a0 and a1 coefficients of current candidate
                                particles[i1,3] = Ar_y_cand_PLF
                                particles[i1,5] = np.tan(np.arctan(np.polyval(dpr,Ar_x_cand_CLF))-Psi_rel_CLF[i1])


        ############################################
        ## UPDATE RIGHT LANE DISTANCE INFORMATION ##
        ############################################
    
        if (a_l_quality >= 2):
        
            # initialize coefficients for left lane polynomial and its derivative
            pl = [a_l3,a_l2,a_l1,a_l0]
            dpl = [3*a_l3, 2*a_l2, a_l1]
        
            # go through all particles
            for i1 in range(num_particles):
            
                # if frames of current and propagated particles are almost aligned
                if np.abs(Psi_rel_CLF[i1]) < Psi_rel_threshold:
                
                    # compute expected a0 and a1 coefficients of left lane
                    particles[i1,4] = np.polyval(pl,X_rel_CLF[i1])
                    particles[i1,6] = np.polyval(dpl,X_rel_CLF[i1])
                
                # if frames of current and propagated particles are disaligned
                else:
                
                    # compute roots of intersection between lane polynomial and
                    # transversal line of car (which are both expressed in local 
                    # frame of current particle)
                    roots_left = np.roots([a_l3, a_l2, a_l1+(1/np.tan(Psi_rel_CLF[i1])), a_l0-(1/np.tan(Psi_rel_CLF[i1]))*(X_rel_CLF[i1]-Y_rel_CLF[i1])])
                
                    # go through all roots
                    for i2 in range(len(roots_left)):
                    
                        # if computed root is real (i.e. imaginary part is zero)
                        if abs(roots_left[i2].imag) < imag_threshold:
                        
                            # compute (and store as candidate) the coordinates of
                            # the intersection in local frame of current particle
                            Al_x_cand_CLF = roots_left[i2].real
                            Al_y_cand_CLF = np.polyval(pl,roots_left[i2].real)
                        
                            # compute the y-coordinate of the intersection
                            # in local frame of the propagated particle
                            Al_y_cand_PLF = -np.sin(Psi_rel_CLF[i1])*(Al_x_cand_CLF - X_rel_CLF[i1]) + np.cos(Psi_rel_CLF[i1])*(Al_y_cand_CLF-Y_rel_CLF[i1])
                        
                            # check whether y-coordinate is on the left-hand side of the car and whether it is closer 
                            # to the car than any other previously computed candidate
                            if (0 >= Al_y_cand_PLF and Al_y_cand_PLF > particles[i1,4] and Al_y_cand_PLF > -Lane_threshold):  
                                
                                # save expected a0 and a1 coefficients of current candidate
                                particles[i1,4] = Al_y_cand_PLF
                                particles[i1,6] = np.tan(np.arctan(np.polyval(dpl,Al_x_cand_CLF))-Psi_rel_CLF[i1])
                                
                                
    ###################################################################
    ## IF ACTIVATED, UPDATE PREDICTED DISPLACEMENT MEASURED BY LIDAR ##
    ###################################################################
    
    if activate_LidarMeas:

        # transform relative displacement of particles from global frame to local frame of current particles
        X_rel_CLF = X_rel_GF*np.cos(particles[:,2]) + Y_rel_GF*np.sin(particles[:,2])
        Y_rel_CLF = X_rel_GF*np.sin(particles[:,2]) - Y_rel_GF*np.cos(particles[:,2])
        Psi_rel_CLF = (-1) * Psi_rel_GF
            
        # relative displacement of particles in current local frame is what should be measured by LIDAR
        particles[:,10] = X_rel_CLF
        particles[:,11] = Y_rel_CLF
        particles[:,12] = Psi_rel_CLF
      
      
    # return propagated particles  
    return particles



def LPF_propagate_vec(particles, num_particles, dt, V_straight_meas, del_f_meas, \
    a_r0, a_r1, a_r2, a_r3, a_r_quality, a_l0, a_l1, a_l2, a_l3, a_l_quality, \
    straight_position_noise, turn_position_noise, lateral_velocity_noise, turn_velocity_noise, \
    Psi_rel_threshold, Lane_threshold, DynamicModel_threshold, \
    activate_LaneMeas, activate_LidarMeas, activate_DynamicModel, \
    mass, Izz, lf, lr, B_f, B_r, C_f, C_r, D_f, D_r, E_f, E_r):
    
#-----------------------------------------------------------------------------
# ABOUT THIS FUNCTION
# Given the current particles, motion model setting, the measurement intake 
# setting, this function carries out the propagation step of the particle 
# filter in a vectorized fashion and returns the propagated particles.
#
# INPUT
# particles               : current particles
# num_particles           : number of particles to be initialized
# dt                      : propagation time increment [s]
# V_straight_meas         : straight velocity measurement [m/s]
# del_f_meas              : steering angle measurement [rad]
# a_r0                    : a_0 coefficient of right lane [m]
# a_r1                    : a_1 coefficient of right lane [m^(-1)]
# a_r2                    : a_2 coefficient of right lane [m^(-2)]
# a_r3                    : a_3 coefficient of right lane [m^(-3)]
# a_r_quality             : measurement quality of right lane
# a_l0                    : a_0 coefficient of left lane [m]
# a_l1                    : a_1 coefficient of left lane [m^(-1)]
# a_l2                    : a_2 coefficient of left lane [m^(-2)]
# a_l3                    : a_3 coefficient of left lane [m^(-3)]
# a_l_quality             : measurement quality of left lane
# straight_position_noise : std. dev. of process noise on straight position
# turn_position_noise     : std. dev. of process noise on Psi
# lateral_velocity_noise  : std. dev. of process noise on lateral velocity
# turn_velocity_noise     : std. dev. of process noise on dot_Psi
# Psi_rel_threshold       : threshold for considering two frames aligned
# Lane_threshold          : maximal acceptable distance from lanes
# DynamicModel_threshold  : minimum velocity to apply dynamic bicycle model
# activate_LaneMeas       : boolean activating lane measurements
# activate_LidarMeas      : boolean activating LIDAR measurements
# activate_DynamicModel   : boolean activating dynamic bicycle model
# mass                    : car mass [kg]
# Izz                     : car moment of inertia at center of mass [kg*m^2]
# lf                      : distance center of mass to front wheel axis [m]
# lr                      : distance center of mass to rear wheel axis [m]
# B_f                     : tire force coefficient for Pacejka tire model
# B_r                     : tire force coefficient for Pacejka tire model
# C_f                     : tire force coefficient for Pacejka tire model
# C_r                     : tire force coefficient for Pacejka tire model
# D_f                     : tire force coefficient for Pacejka tire model
# D_r                     : tire force coefficient for Pacejka tire model
# E_f                     : tire force coefficient for Pacejka tire model
# E_r                     : tire force coefficient for Pacejka tire model
#
# OUTPUT
# particles : (num_particles)x11 matrix containing propagated particle values
#-----------------------------------------------------------------------------    
    
    # extract certain particle values
    Psi_vals = particles[:,2]
    V_lateral_vals = particles[:,7]
    dot_Psi_vals = particles[:,8]  
    

    ######################################################
    ## PROPAGATE PARTICLES USING SPECIFIED MOTION MODEL ##
    ######################################################

    if (activate_DynamicModel and V_straight_meas >= DynamicModel_threshold):
    
        # compute differential position increments using dynamic bicycle model
        dX = V_straight_meas*np.cos(Psi_vals) - V_lateral_vals*np.sin(Psi_vals)
        dY = V_straight_meas*np.sin(Psi_vals) + V_lateral_vals*np.cos(Psi_vals)

        # compute relative displacement of particles in global frame
        X_rel_GF = dt*dX + np.random.normal(0.0,straight_position_noise,num_particles)*np.cos(Psi_vals+np.arctan(V_lateral_vals/V_straight_meas))
        Y_rel_GF = dt*dY + np.random.normal(0.0,straight_position_noise,num_particles)*np.sin(Psi_vals+np.arctan(V_lateral_vals/V_straight_meas))
        Psi_rel_GF = dt*dot_Psi_vals + np.random.normal(0.0,turn_position_noise,num_particles)

        # compute tire forces according to Pacejka tire model
        alpha_f = del_f_meas * np.ones(num_particles) - (V_lateral_vals+dot_Psi_vals*lf)/V_straight_meas
        alpha_r = (-1.0) * (V_lateral_vals-dot_Psi_vals*lr)/V_straight_meas
        F_y_f = 2.0*D_f * np.sin(C_f*np.arctan((1.0-E_f)*B_f*alpha_f + E_f*np.arctan(B_f*alpha_f))) * np.cos(del_f_meas)
        F_y_r = 2.0*D_r * np.sin(C_r*np.arctan((1.0-E_r)*B_r*alpha_r + E_r*np.arctan(B_r*alpha_r)))

        # compute differential velocity increments using dynamic bicycle model
        ddot_Psi = 2.0/Izz * (lf*F_y_f-lr*F_y_r)
        ddot_Y = (-1.0) * dot_Psi_vals * V_straight_meas + 2.0/mass * (F_y_f*np.cos(del_f_meas)+F_y_r)

        # compute new particle velocity values
        particles[:,7] = particles[:,7] + dt*ddot_Y + np.random.normal(0.0,lateral_velocity_noise,num_particles)
        particles[:,8] = particles[:,8] + dt*ddot_Psi + np.random.normal(0.0,turn_velocity_noise,num_particles)
    
    else:
    
        # compute differential increments using kinematic bicycle model
        beta = m.atan(lr*m.tan(del_f_meas)/(lf+lr))*np.ones(num_particles)
        dX = np.multiply(V_straight_meas,np.cos(Psi_vals + beta))
        dY = np.multiply(V_straight_meas,np.sin(Psi_vals + beta))
        dPsi = np.multiply(V_straight_meas,np.sin(beta)/lr)

        # compute relative displacement of particles in global frame
        X_rel_GF = dt*dX + np.random.normal(0.0,straight_position_noise,num_particles) * np.cos(Psi_vals+beta)
        Y_rel_GF = dt*dY + np.random.normal(0.0,straight_position_noise,num_particles) * np.sin(Psi_vals+beta)
        Psi_rel_GF = dt*dPsi + np.random.normal(0.0,turn_position_noise,num_particles)
       
    # compute propagated particles as current particles plus displacement
    particles[:,0] = particles[:,0] + X_rel_GF
    particles[:,1] = particles[:,1] + Y_rel_GF
    particles[:,2] = (particles[:,2] + Psi_rel_GF) % (2.0*m.pi)
    
    
    ########################################################
    ## IF ACTIVATED, UPDATE LANE DISTANCE AND ORIENTATION ##
    ########################################################

    if (activate_LaneMeas):
    
        # reset expected a0 and a1 coefficient of the right lane for all particles
        particles[:,3] = np.ones(num_particles)*float('inf')
        particles[:,5] = np.ones(num_particles)*float('inf')
    
        # reset expected a0 and a1 coefficient of the left lane for all particles
        particles[:,4] = np.ones(num_particles)*float('-inf')
        particles[:,6] = np.ones(num_particles)*float('-inf')
    
        if (a_r_quality >= 2 or a_l_quality >= 2):
        
            # transform relative displacement of particles from global frame to local frame of current particles
            X_rel_CLF = X_rel_GF*np.cos(particles[:,2]) + Y_rel_GF*np.sin(particles[:,2])
            Y_rel_CLF = X_rel_GF*np.sin(particles[:,2]) - Y_rel_GF*np.cos(particles[:,2])
            Psi_rel_CLF = (-1) * Psi_rel_GF
    

        ################################################
        ## UPDATE RIGHT LANE DISTANCE AND ORIENTATION ##
        ################################################

        if (a_r_quality >= 2):
        
            # initialize coefficients for right lane polynomial and its derivative
            pr = [a_r3,a_r2,a_r1,a_r0]
            dpr = [3.0*a_r3, 2.0*a_r2, a_r1]
            
            # initialize matrix of zeros to store x-coordinates of candidate intersection points in local frame of current particles
            Ar_x_cand_CLF = np.zeros([num_particles,2])
            
            # determine indices of particles that are considered whose propagation is considered 'aligned' or 'disaligned'
            aligned = np.nonzero(np.abs(Psi_rel_CLF)<=Psi_rel_threshold)
            disaligned = np.nonzero(np.abs(Psi_rel_CLF)>Psi_rel_threshold)
            num_disaligned = np.count_nonzero(np.abs(Psi_rel_CLF)>Psi_rel_threshold)
            
            # if frames of current and propagated particles are almost aligned, the x-coordinates of the intersection point is X_rel_CLF
            Ar_x_cand_CLF[aligned,0] = X_rel_CLF[aligned]
            
            # compute x-coordinates of candidate intesection points in local frame of current particles based only on a_l2, a_l1, and a_l0,
            # since vectorized computation of roots of 3rd order polynomial yields weaker performance (empirical observation)
            # (NOTICE: if roots are complex conjugate, quad_poly_roots sets them both to zero)
            Ar_x_cand_CLF[disaligned] = quad_poly_roots( \
            a_r2*np.ones(num_disaligned), \
            a_r1*np.ones(num_disaligned) + (1.0/np.tan(Psi_rel_CLF[disaligned])), \
            a_r0*np.ones(num_disaligned) - (1.0/np.tan(Psi_rel_CLF[disaligned]))*X_rel_CLF[disaligned] - Y_rel_CLF[disaligned])
            
            # compute corresponding y-coordinates of candidate intersection points in local frame of current particles
            Ar_y_cand_CLF = np.polyval(pr,Ar_x_cand_CLF)
            
            # compute corrsponding y-coordinate of candidate intersection points in local frame of propagated particles
            Ar_y_cand_PLF = np.transpose(np.concatenate(([-np.sin(Psi_rel_CLF)],[-np.sin(Psi_rel_CLF)]),axis=0))*(Ar_x_cand_CLF - np.transpose(np.concatenate(([X_rel_CLF],[X_rel_CLF]),axis=0))) + \
                            np.transpose(np.concatenate(([np.cos(Psi_rel_CLF)],[np.cos(Psi_rel_CLF)]),axis=0))*(Ar_y_cand_CLF - np.transpose(np.concatenate(([Y_rel_CLF],[Y_rel_CLF]),axis=0)))
            
            # eliminate candidate intersection points that are either on the the wrong car side or too far away according to Lane_threshold 
            # by setting their y-coordinate to +Inf
            Ar_y_cand_PLF[Ar_y_cand_PLF<=0] = float('inf')
            Ar_y_cand_PLF[Ar_y_cand_PLF>Lane_threshold] = float('inf')
            
            # the estimated lane distance for the propagated particles can now be computed as row-wise minimal entry in the matrix of candidate 
            # intersection point y-coordinates in the propagated local frame 
            particles[:,3] = np.min(Ar_y_cand_PLF,axis=1)

            # identify where the above row-wise minima happen in Ar_y_cand_PLF ('index_min' is a 0-1-matrix of same dimension as Ar_y_cand_PLF, 
            # where the 1's indicate row-wise minimal entries of Ar_y_cand_PLF and the 0's indicate all other entries of Ar_y_cand_PLF)
            index_min = Ar_y_cand_PLF == np.transpose(np.concatenate(([np.min(Ar_y_cand_PLF,axis=1)],[np.min(Ar_y_cand_PLF,axis=1)]),axis=0))
            
            # if there is more than one minimum in a row, discard the one in the second column
            index_min[np.sum(index_min,axis=1)>1,1] = 0
            
            # compute the estimated lane orientation for the propagated particles using the entries of Ar_x_cand_CLF 
            # that correspond to the row-wise minima of Ar_y_cand_PLF
            particles[:,5] = np.tan(np.arctan(np.polyval(dpr,Ar_x_cand_CLF[index_min]))-Psi_rel_CLF)

            # if the estimated lane distance for the propagated particles is +Inf, 
            # set also the corresponding lane orientation to +Inf
            particles[np.isinf(particles[:,3]),5] = float('inf')


        ###############################################
        ## UPDATE LEFT LANE DISTANCE AND ORIENTATION ##
        ###############################################

        if (a_l_quality >= 2):
        
            # initialize coefficients for left lane polynomial and its derivative
            pl = [a_l3,a_l2,a_l1,a_l0]
            dpl = [3.0*a_l3, 2.0*a_l2, a_l1]
            
            # initialize matrix of zeros to store x-coordinates of candidate intersection points in local frame of current particles
            Al_x_cand_CLF = np.zeros([num_particles,2])
            
            # determine indices of particles that are considered whose propagation is considered 'aligned' or 'disaligned'
            aligned = np.nonzero(np.abs(Psi_rel_CLF)<=Psi_rel_threshold)
            disaligned = np.nonzero(np.abs(Psi_rel_CLF)>Psi_rel_threshold)
            num_disaligned = np.count_nonzero(np.abs(Psi_rel_CLF)>Psi_rel_threshold)
            
            # if frames of current and propagated particles are almost aligned, the x-coordinates of the intersection point is X_rel_CLF
            Al_x_cand_CLF[aligned,0] = X_rel_CLF[aligned]
            
            # compute x-coordinates of candidate intesection points in local frame of current particles based only on a_l2, a_l1, and a_l0,
            # since vectorized computation of roots of 3rd order polynomial yields weaker performance (empirical observation)
            # (NOTICE: if roots are complex conjugate, quad_poly_roots sets them both to zero)
            Al_x_cand_CLF[disaligned] = quad_poly_roots( \
            a_l2*np.ones(num_disaligned), \
            a_l1*np.ones(num_disaligned) + (1.0/np.tan(Psi_rel_CLF[disaligned])), \
            a_l0*np.ones(num_disaligned) - (1.0/np.tan(Psi_rel_CLF[disaligned]))*X_rel_CLF[disaligned] - Y_rel_CLF[disaligned])
            
            # compute corresponding y-coordinates of candidate intersection points in local frame of current particles
            Al_y_cand_CLF = np.polyval(pl,Al_x_cand_CLF)
            
            # compute corrsponding y-coordinate of candidate intersection points in local frame of propagated particles
            Al_y_cand_PLF = np.transpose(np.concatenate(([-np.sin(Psi_rel_CLF)],[-np.sin(Psi_rel_CLF)]),axis=0))*(Al_x_cand_CLF - np.transpose(np.concatenate(([X_rel_CLF],[X_rel_CLF]),axis=0))) + \
                            np.transpose(np.concatenate(([np.cos(Psi_rel_CLF)],[np.cos(Psi_rel_CLF)]),axis=0))*(Al_y_cand_CLF - np.transpose(np.concatenate(([Y_rel_CLF],[Y_rel_CLF]),axis=0)))
            
            # eliminate candidate intersection points that are either on the the wrong car side or too far away according to Lane_threshold 
            # by setting their y-coordinate to +Inf
            Al_y_cand_PLF[Al_y_cand_PLF>=0] = float('-inf')
            Al_y_cand_PLF[Al_y_cand_PLF<(-Lane_threshold)] = float('-inf')
            
            # the estimated lane distance for the propagated particles can now be computed as row-wise maximal entry in the matrix of candidate 
            # intersection point y-coordinates in the propagated local frame 
            particles[:,4] = np.max(Al_y_cand_PLF,axis=1)
            
            # identify where the above row-wise maxima happen in Al_y_cand_PLF ('index_max' is a 0-1-matrix of same dimension as Al_y_cand_PLF, 
            # where the 1's indicate row-wise maximal entries of Al_y_cand_PLF and the 0's indicate all other entries of Al_y_cand_PLF)
            index_max = Al_y_cand_PLF == np.transpose(np.concatenate(([np.min(Al_y_cand_PLF,axis=1)],[np.min(Al_y_cand_PLF,axis=1)]),axis=0))
            
            # if there is more than one maximum in a row, discard the one in the second column
            index_max[np.sum(index_max,axis=1)>1,1] = 0
            
            # compute the estimated lane orientation for the propagated particles using the entries of Al_x_cand_CLF 
            # that correspond to the row-wise maxima of Al_y_cand_PLF
            particles[:,6] = np.tan(np.arctan(np.polyval(dpl,Ar_x_cand_CLF[index_max]))-Psi_rel_CLF)

            # if the estimated lane distance for the propagated particles is -Inf, 
            # set also the corresponding lane orientation to -Inf
            particles[np.isinf(particles[:,4]),6] = float('-inf')
            
            
    ###################################################################
    ## IF ACTIVATED, UPDATE PREDICTED DISPLACEMENT MEASURED BY LIDAR ##
    ###################################################################
    
    if activate_LidarMeas:

        # transform relative displacement of particles from global frame to local frame of current particles
        X_rel_CLF = X_rel_GF*np.cos(particles[:,2]) + Y_rel_GF*np.sin(particles[:,2])
        Y_rel_CLF = X_rel_GF*np.sin(particles[:,2]) - Y_rel_GF*np.cos(particles[:,2])
        Psi_rel_CLF = (-1) * Psi_rel_GF
            
        # relative displacement of particles in current local frame is what should be measured by LIDAR
        particles[:,10] = X_rel_CLF
        particles[:,11] = Y_rel_CLF
        particles[:,12] = Psi_rel_CLF
        
    
    # return propagated particles    
    return particles
                                
                                

def LPF_resample(particles, num_particles, X_meas, Y_meas, Psi_meas, V_lateral_meas, dot_Psi_meas, \
    a_r0, a_l0, a_r1, a_l1, a_r_quality, a_l_quality, dX_meas, dY_meas, dPsi_meas, \
    gps_X_noise, gps_Y_noise, gps_Psi_noise, a_r0_noise, a_l0_noise, a_r1_noise, a_l1_noise, \
    gps_V_lateral_noise, dot_Psi_noise, lidar_dX_noise, lidar_dY_noise, lidar_dPsi_noise, \
    activate_LaneMeas, activate_LidarMeas, activate_DynamicModel, GPS_available, Lidar_available):
        
#-----------------------------------------------------------------------------
# ABOUT THIS FUNCTION
# Given the current particles and measurements, this function carries out the 
# resampling step of the particle filter, that is, it computes the probability 
# of each particle and resamples from that set of particles according to the 
# computed probabilities. Then, it returns the resampled particles.
#
# INPUT
# particles             : current particles
# num_particles         : number of particles to be initialized
# X_meas                : X measurement [m]
# Y_meas                : Y measurement [m]
# Psi_meas              : Psi measurement [rad]
# V_lateral_meas        : lateral velocity measurement [m/s]
# dot_Psi_meas          : dot_Psi measurement [rad/s]
# a_r0                  : a_0 coefficient of right lane [m]
# a_r1                  : a_1 coefficient of right lane [m^(-1)]
# a_l0                  : a_0 coefficient of left lane [m]
# a_l1                  : a_1 coefficient of left lane [m^(-1)]
# a_r_quality           : measurement quality of right lane
# a_l_quality           : measurement quality of left lane
# dX_meas               : dX measurement from LIDAR [m]
# dY_meas               : dY measurement from LIDAR [m]
# dPsi_meas             : dPsi measurement from LIDAR [rad]
# gps_X_noise           : std. dev. of X measurement [m]
# gps_Y_noise           : std. dev. of Y measurement [m]
# gps_Psi_noise         : std. dev. of Psi measurement [rad]        
# a_r0_noise            : std. dev. of a_r0 coefficient [m]
# a_l0_noise            : std. dev. of a_l0 coefficient [m]
# a_r1_noise            : std. dev. of a_r1 coefficient [m^(-1)]
# a_l1_noise            : std. dev. of a_l1 coefficient [m^(-1)]
# gps_V_lateral_noise   : std. dev. of lateral velocity measurement [m/s]
# dot_Psi_noise         : std. dev. of dot_Psi measurement [rad/s]
# lidar_dX_noise        : std. dev. of lidar-based dX measurement [m]
# lidar_dY_noise        : std. dev. of lidar-based dY measurement [m] 
# lidar_dPsi_noise      : std. dev. of lidar-based dPsi measurement [rad]
# activate_LaneMeas     : boolean activating lane measurements
# activate_LidarMeas    : boolean activating LIDAR measurements
# activate_DynamicModel : boolean activating dynamic bicycle model
# GPS_available         : boolean indicating whether GPS is available
# Lidar_available       : boolean indicating whether LIDAR is available
#
# OUTPUT
# particles : (num_particles)x11 matrix containing resampled particle values
#-----------------------------------------------------------------------------
        
    # initialize probability vectors
    gps_X_prob = np.ones(num_particles)
    gps_Y_prob = np.ones(num_particles)
    gps_Psi_prob = np.ones(num_particles)
    a_r0_prob = np.ones(num_particles)
    a_l0_prob = np.ones(num_particles)
    a_r1_prob = np.ones(num_particles)
    a_l1_prob = np.ones(num_particles)
    V_lateral_prob = np.ones(num_particles)
    dot_Psi_prob = np.ones(num_particles)
    lidar_dX_prob = np.ones(num_particles)
    lidar_dY_prob = np.ones(num_particles)
    lidar_dPsi_prob = np.ones(num_particles)
    
    
    ####################################
    ## COMPUTE PARTICLE PROBABILITIES ##
    ####################################

    # if GPS is available, update corresponding probability vectors
    if GPS_available >= 1:
        gps_X_prob = norm.pdf(X_meas*np.ones(num_particles),particles[:,0],gps_X_noise*np.ones(num_particles))
        gps_Y_prob = norm.pdf(Y_meas*np.ones(num_particles),particles[:,1],gps_Y_noise*np.ones(num_particles))
        gps_Psi_prob = norm.pdf(Psi_meas*np.ones(num_particles),particles[:,2],gps_Psi_noise*np.ones(num_particles))
        if activate_DynamicModel:
            V_lateral_prob = norm.pdf(V_lateral_meas*np.ones(num_particles),particles[:,7]+particles[:,9],gps_V_lateral_noise*np.ones(num_particles))
            dot_Psi_prob = norm.pdf(dot_Psi_meas*np.ones(num_particles),particles[:,8],dot_Psi_noise*np.ones(num_particles))
    
    # if right lane measurement is good enough, update corresponding probability vectors
    if (activate_LaneMeas and a_r_quality >= 2 and GPS_available == 0):
        a_r0_prob = norm.pdf(a_r0*np.ones(num_particles),particles[:,3],a_r0_noise*np.ones(num_particles))
        a_r1_prob = norm.pdf(a_r1*np.ones(num_particles),particles[:,5],a_r1_noise*np.ones(num_particles))
        
    # if left lane measurement is good enough, update corresponding probability vectors
    if (activate_LaneMeas and a_l_quality >= 2 and GPS_available == 0):
        a_l0_prob = norm.pdf(a_l0*np.ones(num_particles),particles[:,4],a_l0_noise*np.ones(num_particles))
        a_l1_prob = norm.pdf(a_l1*np.ones(num_particles),particles[:,6],a_l1_noise*np.ones(num_particles))
    
    # if LIDAR is activated and there is a measurement available, update corresponding probability vectors
    if (activate_LidarMeas and Lidar_available and GPS_available == 0):
        lidar_dX_prob = norm.pdf(dX_meas*np.ones(num_particles),particles[:,10],lidar_dX_noise*np.ones(num_particles))
        lidar_dY_prob = norm.pdf(dY_meas*np.ones(num_particles),particles[:,11],lidar_dY_noise*np.ones(num_particles))
        lidar_dPsi_prob = norm.pdf(dPsi_meas*np.ones(num_particles),particles[:,12],lidar_dPsi_noise*np.ones(num_particles))
    
    # compute non-normalized probability of particles
    particles[:,13] = 1e0 * gps_X_prob * gps_Y_prob * gps_Psi_prob * \
                            a_r0_prob * a_l0_prob * a_r1_prob * a_l1_prob * \
                            V_lateral_prob * dot_Psi_prob * \
                            lidar_dX_prob * lidar_dY_prob * lidar_dPsi_prob
    
    # compute normalization constant
    norm_constant = sum(particles[:,13])

    # If narmalization constant is zero, do not resample. Return original particles instead.
    if norm_constant == 0:
        #print('norm_constant is zero')
        return particles

    # compute normalized probability of each particle
    particles[:,13] = particles[:,13]/norm_constant
    
    
    ########################     
    ## RESAMPLE PARTICLES ##
    ########################

    # initialize auxiliary vector for new particles
    new_particles = np.zeros((num_particles,14))

    for i in range(num_particles):
    
        # draw uniformly distributed random number between 0 and 1
        tmp1 = rnd.uniform(0,1)
    
        # initialize aggregated probability as probability of first particle
        tmp2 = particles[0,13]
    
        # initialize particle count
        count = 0
    
        # as long as aggragated probability of particles is less than the drawn
        # uniform random number and count had not reached last particle
        while (tmp2 < tmp1 and count < num_particles-1): 
        
            # update count
            count = count + 1
        
            # aggregate particle probability
            tmp2 = tmp2 + particles[count,13]
    
        # once aggregated particle probability exceeds drawn uniform random
        # number, select last particle that was aggregated as new particle
        new_particles[i,:] = particles[count,:]


    # assign values of auxiliary vector to persistent particles
    particles = new_particles
    
    # return resampled particles
    return particles
    
    
    
def LPF_throughput(particles, num_particles, num_through, GPS_available, activate_DynamicModel, \
    X_meas, Y_meas, Psi_meas, V_lateral_meas, dot_Psi_meas, \
    gps_X_noise, gps_Y_noise, gps_Psi_noise, gps_V_lateral_noise, dot_Psi_noise):
    
#-----------------------------------------------------------------------------
# ABOUT THIS FUNCTION
# Given the current particles, measurements, sensor noise, and the number of 
# particles that shall be used for measurement throughput, this functions 
# randomly picks the specified number of particles, re-initializes them 
# according to a normal distribution around the current measurements, and
# then returns the new particles.
#
# INPUT
# particles             : current particles
# num_particles         : number of particles to be initialized
# num_through           : number of particles to be used for throghput
# GPS_available         : boolean indicating whether GPS is available
# activate_DynamicModel : boolean activating dynamic bicycle model
# X_meas                : X measurement [m]
# Y_meas                : Y measurement [m]
# Psi_meas              : Psi measurement [rad]
# V_lateral_meas        : lateral velocity measurement [m/s]
# dot_Psi_meas          : dot_Psi measurement [rad/s]
# gps_X_noise           : std. dev. of X measurement [m]
# gps_Y_noise           : std. dev. of Y measurement [m]
# gps_Psi_noise         : std. dev. of Psi measurement [rad]        
# gps_V_lateral_noise   : std. dev. of lateral velocity measurement [m/s]
# dot_Psi_noise         : std. dev. of dot_Psi measurement [rad/s]
#
# OUTPUT
# particles : (num_particles)x11 matrix containing the particle values after 
#             throughput has been carried out
#-----------------------------------------------------------------------------    
    
    # randomly draw indices of particles to be reinitialized
    through_indices = np.random.choice(num_particles, num_through)
    
    # If GPS is available, throughput X_meas, Y_meas, and Psi_meas
    if GPS_available:
        
        # throughput values = normal distributions around current measurements
        init_X_values = X_meas*np.ones(num_through) + np.random.normal(0.0, gps_X_noise, num_through)
        init_Y_values = Y_meas*np.ones(num_through) + np.random.normal(0.0, gps_Y_noise, num_through)
        init_Psi_values = Psi_meas*np.ones(num_through) + np.random.normal(0.0, gps_Psi_noise, num_through)
        
        # assign throughput values
        new_particles = np.concatenate(([init_X_values],[init_Y_values],[init_Psi_values],
                                [particles[through_indices,[3]]],[particles[through_indices,[4]]],
                                [particles[through_indices,[5]]],[particles[through_indices,[6]]],
                                [particles[through_indices,[7]]],[particles[through_indices,[8]]],
                                [particles[through_indices,[9]]],[particles[through_indices,[10]]],
                                [particles[through_indices,[11]]],[particles[through_indices,[12]]],
                                [particles[through_indices,[13]]]),axis=0)
        particles[through_indices,:] = np.transpose(new_particles)        
        
    # If Dynamic Model is active, throughput V_lateral_meas and dot_Psi_meas       
    if activate_DynamicModel:
        
        # throughput values = normal distributions around current measurements
        init_V_lateral_values = (V_lateral_meas*np.ones(num_through) - np.mean(particles[:,9])*np.ones(num_through)) + np.random.normal(0.0,gps_V_lateral_noise,num_through)
        init_dot_Psi_values = dot_Psi_meas*np.ones(num_through) + np.random.normal(0.0,dot_Psi_noise,num_through)
        
        # assign throughput values
        new_particles = np.concatenate(([particles[through_indices,[0]]],[particles[through_indices,[1]]],[particles[through_indices,[2]]],
                                [particles[through_indices,[3]]],[particles[through_indices,[4]]],
                                [particles[through_indices,[5]]],[particles[through_indices,[6]]],
                                [init_V_lateral_values],[init_dot_Psi_values],
                                [particles[through_indices,[9]]],[particles[through_indices,[10]]],
                                [particles[through_indices,[11]]],[particles[through_indices,[12]]],
                                [particles[through_indices,[13]]]),axis=0)
        particles[through_indices,:] = np.transpose(new_particles)        
    
    # return throughput particles    
    return particles



def LPF_roughening(particles, num_particles, activate_DynamicModel, K):
    
#-----------------------------------------------------------------------------
# ABOUT THIS FUNCTION (NOT USED IN FINAL IMPLEMENTATION!!)
# Given the current particles, this function takes the particle values, adds 
# some noise to them, and returns the perturbed particles.
#
# INPUT
# particles             : current particles
# num_particles         : number of particles to be initialized
# activate_DynamicModel : boolean activating dynamic bicycle model
# K                     : tuning parameter, the bigger the more noise
#
# OUTPUT
# particles : (num_particles)x11 matrix containing roughened particle values
#----------------------------------------------------------------------------- 
    
    # compute dimension of state space depending on motion model
    if (activate_DynamicModel):
        d = 5
    else:
        d = 3
    
    # compute maximum inter-sample variability for each estimation state   
    E_X = np.ptp(particles[:,0])
    E_Y = np.ptp(particles[:,1])
    E_Psi = np.ptp(particles[:,2])
    if (activate_DynamicModel):
        E_V_lateral = np.ptp(particles[:,7])
        E_dot_Psi = np.ptp(particles[:,8])
    
    # compute standard deviation of roughening for each estimation state
    sigma_X = K * E_X * np.power(num_particles,-1/d)
    sigma_Y = K * E_Y * np.power(num_particles,-1/d)        
    sigma_Psi = K * E_Psi * np.power(num_particles,-1/d)
    if (activate_DynamicModel):
        sigma_V_lateral = K * E_V_lateral * np.power(num_particles,-1/d)
        sigma_dot_Psi = K * E_dot_Psi * np.power(num_particles,-1/d)
    
    # roughen particles according to respective standard deviation
    particles[:,0] = particles[:,0] + np.random.uniform(-np.sqrt(3)*sigma_X,np.sqrt(3)*sigma_X,num_particles)    
    particles[:,1] = particles[:,1] + np.random.uniform(-np.sqrt(3)*sigma_Y,np.sqrt(3)*sigma_Y,num_particles) 
    particles[:,2] = particles[:,2] + np.random.uniform(-np.sqrt(3)*sigma_Psi,np.sqrt(3)*sigma_Psi,num_particles)  
    if (activate_DynamicModel):
        particles[:,7] = particles[:,7] + np.random.uniform(-np.sqrt(3)*sigma_V_lateral,np.sqrt(3)*sigma_V_lateral,num_particles)
        particles[:,8] = particles[:,8] + np.random.uniform(-np.sqrt(3)*sigma_dot_Psi,np.sqrt(3)*sigma_dot_Psi,num_particles)

    # return roughened particles
    return particles
    
    
    
def LPF_main(iter_num, dt, num_particles, GPS_available, Lidar_available, \
    activate_LaneMeas, activate_LidarMeas, activate_DynamicModel, activate_GPS_reset,  \
    X_meas, Y_meas, Psi_meas, dot_Psi_meas, \
    V_straight_meas, V_lateral_meas, del_f_meas, \
    a_r0, a_r1, a_r2, a_r3, a_r_quality, \
    a_l0, a_l1, a_l2, a_l3, a_l_quality, \
    dX_meas, dY_meas, dPsi_meas, \
    straight_position_noise, turn_position_noise, lateral_velocity_noise, turn_velocity_noise, \
    gps_X_noise, gps_Y_noise, gps_Psi_noise, gps_V_lateral_noise, dot_Psi_noise, \
    a_r0_noise, a_l0_noise, a_r1_noise, a_l1_noise, \
    lidar_dX_noise, lidar_dY_noise, lidar_dPsi_noise, \
    gps_V_lateral_bias, mass, Izz, lf, lr, \
    Psi_rel_threshold, Lane_threshold, DynamicModel_threshold, \
    B_f, B_r, C_f, C_r, D_f, D_r, E_f, E_r, \
    num_through):
        
#-----------------------------------------------------------------------------
# ABOUT THIS FUNCTION
# This is the main function putting together all the components of the particle 
# filter in the correct order. It takes in the currently available measurements, 
# runs one iteration of the particle filter, and returns the current estimate 
# of the car's position and orientation. It need be, it initializes particles.
#
# INPUT
# iter_num                : current iteration number
# dt                      : propagation time increment [s]
# num_particles           : number of particles to be initialized
# GPS_available           : boolean indicating whether GPS is available
# Lidar_available         : boolean indicating whether LIDAR is available
# activate_LaneMeas       : boolean activating lane measurements
# activate_LidarMeas      : boolean activating LIDAR measurements
# activate_DynamicModel   : boolean activating dynamic bicycle model
# activate_GPS_reset      : boolean activating reset at sudden GPS measurement
# X_meas                  : X measurement [m]
# Y_meas                  : Y measurement [m]
# Psi_meas                : Psi measurement [rad]
# dot_Psi_meas            : dot_Psi measurement [rad/s]
# V_straight_meas         : straight velocity measurement [m/s]
# V_lateral_meas          : lateral velocity measurement [m/s]
# del_f_meas              : steering angle measurement [rad]
# a_r0                    : a_0 coefficient of right lane [m]
# a_r1                    : a_1 coefficient of right lane [m^(-1)]
# a_r2                    : a_2 coefficient of right lane [m^(-2)]
# a_r3                    : a_3 coefficient of right lane [m^(-3)]
# a_r_quality             : measurement quality of right lane
# a_l0                    : a_0 coefficient of left lane [m]
# a_l1                    : a_1 coefficient of left lane [m^(-1)]
# a_l2                    : a_2 coefficient of left lane [m^(-2)]
# a_l3                    : a_3 coefficient of left lane [m^(-3)]
# dX_meas                 : dX measurement from LIDAR [m]
# dY_meas                 : dY measurement from LIDAR [m]
# dPsi_meas               : dPsi measurement from LIDAR [rad]
# a_l_quality             : measurement quality of left lane
# straight_position_noise : std. dev. of process noise on straight position
# turn_position_noise     : std. dev. of process noise on Psi
# lateral_velocity_noise  : std. dev. of process noise on lateral velocity
# turn_velocity_noise     : std. dev. of process noise on dot_Psi
# gps_X_noise             : std. dev. of X measurement [m]
# gps_Y_noise             : std. dev. of Y measurement [m]
# gps_Psi_noise           : std. dev. of Psi measurement [rad]        
# gps_V_lateral_noise     : std. dev. of lateral velocity measurement [m/s]
# dot_Psi_noise           : std. dev. of dot_Psi measurement [rad/s]
# a_r0_noise              : std. dev. of a_r0 coefficient [m]
# a_l0_noise              : std. dev. of a_l0 coefficient [m]
# a_r1_noise              : std. dev. of a_r1 coefficient [m^(-1)]
# a_l1_noise              : std. dev. of a_l1 coefficient [m^(-1)]
# lidar_dX_noise          : std. dev. of lidar-based dX measurement [m]
# lidar_dY_noise          : std. dev. of lidar-based dY measurement [m] 
# lidar_dPsi_noise        : std. dev. of lidar-based dPsi measurement [rad]
# gps_V_lateral_bias      : bias of lateral velocity signal [m/s]        
# mass                    : car mass [kg]
# Izz                     : car moment of inertia at center of mass [kg*m^2]
# lf                      : distance center of mass to front wheel axis [m]
# lr                      : distance center of mass to rear wheel axis [m]
# Psi_rel_threshold       : threshold for considering two frames aligned
# Lane_threshold          : maximal acceptable distance from lanes
# DynamicModel_threshold  : minimum velocity to apply dynamic bicycle model
# B_f                     : tire force coefficient for Pacejka tire model
# B_r                     : tire force coefficient for Pacejka tire model
# C_f                     : tire force coefficient for Pacejka tire model
# C_r                     : tire force coefficient for Pacejka tire model
# D_f                     : tire force coefficient for Pacejka tire model
# D_r                     : tire force coefficient for Pacejka tire model
# E_f                     : tire force coefficient for Pacejka tire model
# E_r                     : tire force coefficient for Pacejka tire model        
# num_through             : number of particles to be used for throghput
#
# OUTPUT
# X_estimate   : scalar estimate of the car's X coordinate (X position)
# Y_estimate   : scalar estimate of the car's Y coordinate (Y position)
# Psi_estimate : scalar estimate of the car's Psi coordinate (orientation)
#-----------------------------------------------------------------------------
    
    #############################################################
    ## INITIALIZATION OF PARTICLES AROUND CURRENT MEASUREMENTS ##   
    #############################################################
    
    if (iter_num == 0):
        
        # initialize particles around current measurements                           
        LPF_main.particles = LPF_initialize(num_particles, X_meas, Y_meas, Psi_meas, V_lateral_meas, dot_Psi_meas, gps_V_lateral_bias, \
                                 gps_X_noise, gps_Y_noise, gps_Psi_noise, gps_V_lateral_noise, dot_Psi_noise)
        
        # state estimate is equal to current (unfiltered) measurements
        X_estimate = X_meas
        Y_estimate = Y_meas
        Psi_estimate = Psi_meas
        
        # propagate particles
        LPF_main.particles = LPF_propagate_vec(LPF_main.particles, num_particles, dt, V_straight_meas, del_f_meas, \
                                 a_r0, a_r1, a_r2, a_r3, a_r_quality, a_l0, a_l1, a_l2, a_l3, a_l_quality, \
                                 straight_position_noise, turn_position_noise, lateral_velocity_noise, turn_velocity_noise, \
                                 Psi_rel_threshold, Lane_threshold, DynamicModel_threshold, \
                                 activate_LaneMeas, activate_LidarMeas, activate_DynamicModel, \
                                 mass, Izz, lf, lr, B_f, B_r, C_f, C_r, D_f, D_r, E_f, E_r)
                                 
        # intitialize previous_GPS_available
        LPF_main.previous_GPS_missing = 0

        # return state estimates and particles 
        return (X_estimate, Y_estimate, Psi_estimate)
               
               
    ########################################################################           
    ## RESAMPLE PARTICLES (EXECUTED ONLY IF NEW MEASUREMENT IS AVAILABLE) ##          
    ######################################################################## 
                      
    if activate_GPS_reset and GPS_available and LPF_main.previous_GPS_missing:
        
        # re-initialize particles                          
        LPF_main.particles = LPF_initialize(num_particles, X_meas, Y_meas, Psi_meas, V_lateral_meas, dot_Psi_meas, gps_V_lateral_bias, \
                                 gps_X_noise, gps_Y_noise, gps_Psi_noise, gps_V_lateral_noise, dot_Psi_noise)
     
    elif (GPS_available or a_r_quality >= 2 or a_l_quality >= 2):
        
        # resample particles
        LPF_main.particles = LPF_resample(LPF_main.particles, num_particles, X_meas, Y_meas, Psi_meas, V_lateral_meas, dot_Psi_meas, \
                                 a_r0, a_l0, a_r1, a_l1, a_r_quality, a_l_quality, dX_meas, dY_meas, dPsi_meas, \
                                 gps_X_noise, gps_Y_noise, gps_Psi_noise, a_r0_noise, a_l0_noise, a_r1_noise, a_l1_noise, \
                                 gps_V_lateral_noise, dot_Psi_noise, lidar_dX_noise, lidar_dY_noise, lidar_dPsi_noise, \
                                 activate_LaneMeas, activate_LidarMeas, activate_DynamicModel, GPS_available, Lidar_available)
        
        # throughput sensor measurements
        LPF_main.particles = LPF_throughput(LPF_main.particles, num_particles, num_through, GPS_available, activate_DynamicModel, \
                                 X_meas, Y_meas, Psi_meas, V_lateral_meas, dot_Psi_meas, \
                                 gps_X_noise, gps_Y_noise, gps_Psi_noise, gps_V_lateral_noise, dot_Psi_noise)
                             

    ##########################################################
    ## COMPUTE STATE ESTIMATE AS AVERAGE OVER ALL PARTICLES ##
    ##########################################################

    # compute particle averages
    X_estimate = np.mean(LPF_main.particles[np.isfinite(LPF_main.particles[:,0]),0])
    Y_estimate = np.mean(LPF_main.particles[np.isfinite(LPF_main.particles[:,1]),1])
    Psi_estimate = np.mean(LPF_main.particles[np.isfinite(LPF_main.particles[:,2]),2])


    ############################################################
    ## PROPAGATE PARTICLES (EXECUTED UNDER ALL CIRCUMSTANCES) ##
    ############################################################

    # propagate particles
    LPF_main.particles = LPF_propagate_vec(LPF_main.particles, num_particles, dt, V_straight_meas, del_f_meas, \
                             a_r0, a_r1, a_r2, a_r3, a_r_quality, a_l0, a_l1, a_l2, a_l3, a_l_quality, \
                             straight_position_noise, turn_position_noise, lateral_velocity_noise, turn_velocity_noise, \
                             Psi_rel_threshold, Lane_threshold, DynamicModel_threshold, \
                             activate_LaneMeas, activate_LidarMeas, activate_DynamicModel, \
                             mass, Izz, lf, lr, B_f, B_r, C_f, C_r, D_f, D_r, E_f, E_r)


    ###############################################
    ## UPDATE GPS INFO AND RETURN STATE ESTIMATE ##
    ###############################################

    # update LPF_main.previous_GPS_missing
    if GPS_available:
        LPF_main.previous_GPS_missing = 0
    else:
        LPF_main.previous_GPS_missing = 1

    # return state estimate
    return (X_estimate, Y_estimate, Psi_estimate)
