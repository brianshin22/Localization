# -*- coding: utf-8 -*-
"""
Created on Sun Nov 22 16:56:53 2015

@author: Brian
"""
import numpy as np
import math as m
from scipy.stats import norm


num_particles = 10
X_ref = 10
Y_ref = 10
Psi_ref = 0
straight_noise = 0.3
turn_noise = 0.05

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

particles = initialize(num_particles, X_ref, Y_ref, Psi_ref, straight_noise, turn_noise)

imag_threshold = 1e-5

v = 13
del_f = 0
a_r0 = 0
a_r1 = 0
a_r2 = 0
a_r3 = 0
a_l0 = 0
a_l1 = 0
a_l2 = 0
a_l3 = 0
a_r_quality = 2
a_l_quality = 2

lf=1.91
lr = 3
dt = 0.1
Psi_rel_threshold = 1e-6
lane_threshold = 5


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
particles[:,2] = particles[:,2] + Psi_rel_GF


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

if (a_r_quality >= 2):
    
    # initialize coefficients for right lane polynomial
    pr = [a_r3,a_r2,a_r1,a_r0]
    
    # go through all particles
    for i1 in range(0,2):
        
        # if frames of current and propagated particles are almost aligned
        if np.abs(Psi_rel_CLF[i1]) < Psi_rel_threshold:
            
            # compute distance to right lane and update flag
            particles[i1,3] = polyval(pr,X_rel_CLF[i1])
            
        # if frames of current and propagated particles are disaligned
        else:
            
            # compute roots of intersection between lane polynomial and
            # transversal line of car (which are both expressed in local 
            # frame of current particle)
            roots_right = roots([a_r3,a_r2,a_r1+(1/np.tan(Psi_rel_CLF[i1])),
                a_r0-(1/np.tan(Psi_rel_CLF[i1]))*(X_rel_CLF[i1]-Y_rel_CLF[i1])])
            
            # go through all roots
            for i2 in range(0,len(roots_right)):
                
                # if computed root is real (i.e. imaginary part is zero)
                if abs(roots_right[i2].imag) < imag_threshold:
                    
                    # compute (and store as candidate) the coordinates of
                    # the intersection in local frame of current particle
                    Ar_x_cand_CLF = roots_right[i2].real
                    Ar_y_cand_CLF = polyval(pr,roots_right[i2].real)
                    
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