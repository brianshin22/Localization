#!/usr/bin/env python
# license removed for brevity

import numpy as np
import math as m
from scipy.stats import norm

def initialize(num_particles, X_ref, Y_ref, Psi_ref, straight_noise, turn_noise):
    particles = np.zeros([6,num_particles])
    init_X_values = X_ref + np.random.normal(0, straight_noise,
                                             num_particles)
    init_Y_values = Y_ref + np.random.normal(0, straight_noise,
                                             num_particles)
    init_Psi_values = Psi_ref + np.random.normal(0, turn_noise,
                                             num_particles)
                                             
    particles = np.concatenate(([init_X_values],[init_Y_values],[init_Psi_values],
                                [float('inf')*np.ones(num_particles)],
                                 [float('-inf')*np.ones(num_particles)],
                                  [np.ones(num_particles)/num_particles]),axis=0)  

    particles = np.transpose(particles)
    return particles

def propagate(particles, v, del_f,a_r0, a_r1, a_r2, a_r3, a_l0, a_l1, a_l2, 
              a_l3, a_r_quality, a_l_quality,num_particles, lf, lr, dt, 
              straight_noise, turn_noise, Psi_rel_threshold, lane_threshold):

    
    # threshold for imaginary root detection
    imag_threshold = 1e-5    
    
    ## PROPAGATE PARTICLE POSITION
    
    Psi_vals = particles[:,2]
    X_rel_GF = np.zeros(num_particles)
    Y_rel_GF = np.zeros(num_particles)
    Psi_rel_GF = np.zeros(num_particles)    
    
    # compute differential increments using kinematic bicycle model
    beta = m.atan(lr*m.tan(del_f)/(lr+lf))*np.ones(num_particles)
    dx = np.multiply(v,np.cos(Psi_vals + beta))
    dy = np.multiply(v,np.sin(Psi_vals + beta))
    dpsi = np.multiply(v,np.sin(beta)/lr)
    
    # compute relative displacement of particles in global frame
    X_rel_GF = dt*dx + np.random.normal(0,straight_noise,
        num_particles) * np.cos(Psi_vals + beta)
    Y_rel_GF = dt*dy + np.random.normal(0,straight_noise,
        num_particles) * np.sin(Psi_vals + beta)
    Psi_rel_GF = dt*dpsi + np.random.normal(0,turn_noise,num_particles)
       
    # compute propagated particles as current particles plus displacement
    particles[:,0] = particles[:,0] + X_rel_GF
    particles[:,1] = particles[:,1] + Y_rel_GF
    particles[:,2] = (particles[:,2] + Psi_rel_GF) % (2*m.pi)
    
    
    ## RESET LANE DISTANCE INFORMATION
    
    # reset value and flag of distance to the right lane for all particles
    particles[:,3] = np.ones(num_particles)*float('inf')

    
    # reset value and flag of distance to the left lane for all particles
    particles[:,5] = np.ones(num_particles)*float('-inf')

    
    ## IF AT LEAST ONE GOOD LANE MEASUREMENT, UPDATE LANE DISTANCE INFORMATION
    
    if ((a_r_quality >= 2) or (a_l_quality >= 2)):
        
        # transform relative displacement of particles from global frame to
        # local frame of current particles
        X_rel_CLF = (X_rel_GF)*np.cos(particles[:,2]) + (Y_rel_GF)*np.sin(particles[:,2])
        Y_rel_CLF = (X_rel_GF)*np.sin(particles[:,2]) - (Y_rel_GF)*np.cos(particles[:,2])
        Psi_rel_CLF = (-1)*Psi_rel_GF

    
    ##########################################
    # UPDATE RIGHT LANE DISTANCE INFORMATION 
    ##########################################
    
    if (a_r_quality >= 2):
        
        # initialize coefficients for right lane polynomial
        pr = [a_r3,a_r2,a_r1,a_r0]
        
        # go through all particles
        for i1 in range(0,2):
            
            # if frames of current and propagated particles are almost aligned
            if np.abs(Psi_rel_CLF[i1]) < Psi_rel_threshold:
                
                # compute distance to right lane and update flag
                particles[i1,3] = np.polyval(pr,X_rel_CLF[i1])
                
            # if frames of current and propagated particles are disaligned
            else:
                
                # compute roots of intersection between lane polynomial and
                # transversal line of car (which are both expressed in local 
                # frame of current particle)
                roots_right = np.roots([a_r3,a_r2,a_r1+(1/np.tan(Psi_rel_CLF[i1])),
                    a_r0-(1/np.tan(Psi_rel_CLF[i1]))*(X_rel_CLF[i1]-Y_rel_CLF[i1])])
                
                # go through all roots
                for i2 in range(0,len(roots_right)):
                    
                    # if computed root is real (i.e. imaginary part is zero)
                    if abs(roots_right[i2].imag) < imag_threshold:
                        
                        # compute (and store as candidate) the coordinates of
                        # the intersection in local frame of current particle
                        Ar_x_cand_CLF = roots_right[i2].real
                        Ar_y_cand_CLF = np.polyval(pr,roots_right[i2].real)
                        
                        # compute the y-coordinate of the intersection
                        # in local frame of the propagated particle
                        Ar_y_cand_PLF = -np.sin(Psi_rel_CLF[i1])*(
                        Ar_x_cand_CLF - X_rel_CLF[i1]) 
                        + np.cos(Psi_rel_CLF[i1])*(Ar_y_cand_CLF-Y_rel_CLF[i1])
                        
                        # check whether y-coordinate is on the right-hand side
                        # of the car and whether it is closer to the car than
                        # any other previously computed candidate
                        if (0 <= Ar_y_cand_PLF and Ar_y_cand_PLF < particles[i1,3] and
                        Ar_y_cand_PLF < lane_threshold):  
                            # save current candidate and update flag
                            particles[i1,3] = Ar_y_cand_PLF


    ##########################################
    # UPDATE LEFT LANE DISTANCE INFORMATION 
    ##########################################
    
    if (a_l_quality >= 2):
        
        # initialize coefficients for left lane polynomial
        pl = [a_l3,a_l2,a_l1,a_l0]
        
        # go through all particles
        for i1 in range(0,2):
            
            # if frames of current and propagated particles are almost aligned
            if np.abs(Psi_rel_CLF[i1]) < Psi_rel_threshold:
                
                # compute distance to left lane and update flag
                particles[i1,3] = np.polyval(pl,X_rel_CLF[i1])
                
            # if frames of current and propagated particles are disaligned
            else:
                
                # compute roots of intersection between lane polynomial and
                # transversal line of car (which are both expressed in local 
                # frame of current particle)
                roots_left = np.roots([a_l3,a_l2,a_l1+(1/np.tan(Psi_rel_CLF[i1])),
                    a_l0-(1/np.tan(Psi_rel_CLF[i1]))*(X_rel_CLF[i1]-Y_rel_CLF[i1])])
                
                # go through all roots
                for i2 in range(0,len(roots_left)):
                    
                    # if computed root is real (i.e. imaginary part is zero)
                    if abs(roots_left[i2].imag) < imag_threshold:
                        
                        # compute (and store as candidate) the coordinates of
                        # the intersection in local frame of current particle
                        Al_x_cand_CLF = np.roots_left[i2].real
                        Al_y_cand_CLF = np.polyval(pl,roots_left[i2].real)
                        
                        # compute the y-coordinate of the intersection
                        # in local frame of the propagated particle
                        Al_y_cand_PLF = -np.sin(Psi_rel_CLF[i1])*(
                        Al_x_cand_CLF - X_rel_CLF[i1]) 
                        + np.cos(Psi_rel_CLF[i1])*(Al_y_cand_CLF-Y_rel_CLF[i1])
                        
                        # check whether y-coordinate is on the left-hand side
                        # of the car and whether it is closer to the car than
                        # any other previously computed candidate
                        if (0 >= Al_y_cand_PLF and Al_y_cand_PLF > particles[i1,3] and
                        Al_y_cand_PLF > -lane_threshold):  
                            # save current candidate and update flag
                            particles[i1,3] = Al_y_cand_PLF

def resample(particles, num_particles, X_ref, Y_ref, Psi_ref, gps_flag, 
             a_r0, a_l0,a_r_quality, a_l_quality, gps_X_noise, gps_Y_noise, 
             gps_Psi_noise, a_r0_noise, a_l0_noise):
     
     ## COMPUTE PARTICLE PROBABILITIES
            
    aux_factor = 1
    gps_trust = 1
    lane_trust = 1
     
    # initialize probability vectors
    gps_X_prob = np.ones(num_particles)
    gps_Y_prob = np.ones(num_particles)
    a_r0_prob = np.ones(num_particles)
    a_l0_prob = np.ones(num_particles)
    gps_Psi_prob = np.ones(num_particles)
     
    # if GPS is available, update corresponding probability vectors
    if gps_flag >= 1:
        gps_X_prob = norm.pdf(X_ref*np.ones(num_particles), particles[:,0],
                              gps_X_noise*np.ones(num_particles))
        gps_Y_prob = norm.pdf(Y_ref*np.ones(num_particles), particles[:,1],
                              gps_Y_noise*np.ones(num_particles)) 
        gps_Psi_prob = norm.pdf(Psi_ref*np.ones(num_particles), particles[:,1],
                              gps_Psi_noise*np.ones(num_particles))
     
    if (gps_flag == 0 and a_r_quality >= 2):
        a_r0_prob = norm.pdf(a_r0*np.ones(num_particles), particles[:,3],
                              a_r0_noise*np.ones(num_particles))
    if (gps_flag == 0 and a_l_quality >= 2):
        a_l0_prob = norm.pdf(a_l0*np.ones(num_particles), particles[:,4],
                              a_r0_noise*np.ones(num_particles))                          
    
    particles[:,5] = aux_factor * gps_trust*gps_X_prob * \
                                  gps_trust*gps_Y_prob * \
                                  lane_trust*a_r0_prob * \
                                  lane_trust*a_l0_prob * \
                                  gps_trust*gps_Psi_prob
    norm_constant = sum(particles[:,5])
    if norm_constant == 0:
        return
    particles[:,5] = particles[:,5]/norm_constant
                                  
    ## RESAMPLE PARTICLES
    new_particles = np.zeros((num_particles,6))                                  
    
    for i in range(0,num_particles-1):
        tmp1 = np.random.uniform(0,1,1)
        tmp2 = particles[0,5]
        count = 0
        while (tmp2 < tmp1 and count < num_particles-1):                              
            count = count+1
            tmp2 = tmp2+particles[count,5]
        new_particles[i,:] = particles[count,:]
    particles = new_particles

"""

def roughening(particles, num_particles, K, d):
    # compute maximum inter-sample variability for each estimation state    
    E_X = np.ptp(particles[:,0])
    E_Y = np.ptp(particles[:,1])
    E_Psi = np.ptp(particles[:,2])
    
    # compute standard deviation of roughening for each estimation state
    sigma_X = K*E_X*np.power(num_particles, -1/d)
    sigma_Y = K*E_Y*np.power(num_particles, -1/d)        
    sigma_Psi = K*E_Psi*np.power(num_particles, -1/d)
    
    # roughen particles according to respective standard deviation
    particles[:,0] = particles[:,0] + np.random.normal(0,sigma_X,num_particles)    
    particles[:,1] = particles[:,1] + np.random.normal(0,sigma_Y,num_particles) 
    particles[:,2] = particles[:,2] + np.random.normal(0,sigma_Psi,num_particles)     

    return particles

"""

# merged_filter    
# Inputs: 
# Outputs: X_out,Y_out,Psi_out state after iteration of particle filter
# using either GPS or lane measurements

def merged_filter(gps_present, X_ref, Y_ref,Psi_ref,v, del_f, a_r0,a_r1, 
                  a_r2, a_r3, a_l0, a_l1, a_l2, a_l3, a_r_quality, a_l_quality):
                      
    # define number of particles used                  
    num_particles = 300

    # define standard deviation of process noise
    straight_noise_GPS = .3
    turn_noise_GPS = 0.05
    straight_noise_lane = .1
    turn_noise_lane = 0.0005
    
    # define standard deviation of measurement noise 
    gps_X_noise = 0.3
    gps_Y_noise = 0.3
    gps_Psi_noise = 0.1
    
    a_r0_noise = 0.1
    a_l0_noise = 0.1
    
    
    # define parameters for particle roughening
    K = 0.01    # tuning parameter for spreading of roughening
    d = 3       # dimension of the state space to be roughened
    
    # define parameters for propagation of particles
    lf  = 1.91 # [m] distance from center of mass to front axis
    lr  = 3    # [m] distance from center of mass to rear axis
    dt  = 0.1  # [s] time increment for integration
    
    # define thresholds for different functions
    Psi_rel_threshold = 1e-6   # maximum Psi_rel to be considered aligned [rad]
    lane_threshold = 5         # maximum acceptable distance to lanes [m]

    
    try:
        gps_flag
        gps_last = gps_flag
    except NameError:
        gps_last = 0
    
    gps_flag = gps_present
        
    
    try:
        dummy = particles[0]
    except NameError:
        particles = initialize(num_particles, X_ref, Y_ref, Psi_ref, straight_noise_GPS, 
                   turn_noise_GPS)
        x_avg = np.mean(particles[:,0])
        y_avg = np.mean(particles[:,1])
        psi_avg = np.mean(particles[:,2])
        return (x_avg,y_avg,psi_avg)
                   
    if (gps_flag == 0 and (a_r_quality >= 2 or a_l_quality >= 2)):
        particles = propagate(particles, v, del_f, a_r0, a_r1, a_r2, a_r3, a_l0,
                              a_l1, a_l2, a_l3, a_r_quality, a_l_quality, 
                              num_particles, lf, lr, dt,straight_noise_lane, 
                              turn_noise_lane, Psi_rel_threshold, lane_threshold)    
    else:
        particles = propagate(particles, v, del_f, a_r0, a_r1, a_r2, a_r3, a_l0,
                              a_l1, a_l2, a_l3, a_r_quality, a_l_quality, 
                              num_particles, lf, lr, dt,straight_noise_GPS, 
                              turn_noise_GPS, Psi_rel_threshold, lane_threshold)                               

## IF ANY MEASUREMENT IS AVAILABLE: RESAMPLE PARTICLES                              
    if (gps_flag >= 1 and gps_last == 0):
        # reinitialize particles
        particles = initialize(num_particles, X_ref, Y_ref, Psi_ref, straight_noise_GPS, 
                   turn_noise_GPS)     
    elif (gps_flag >= 1 or a_r_quality >=2 or a_l_quality >= 2):
        particles = resample(particles, num_particles, X_ref, Y_ref, Psi_ref, gps_flag, 
             a_r0, a_l0,a_r_quality, a_l_quality, gps_X_noise, gps_Y_noise, 
             gps_Psi_noise, a_r0_noise, a_l0_noise)

    # roughen particles to ensure enough spreading
    # particles = roughening(particles,num_particles,K,d)

## COMPUTE AVERAGE POSITION OF PARTICLES = CURRENT STATE ESTIMATE
    
    x_avg = np.mean(particles[:,0])
    y_avg = np.mean(particles[:,1])
    psi_avg = np.mean(particles[:,2])
    
    return (x_avg,y_avg,psi_avg)
    
