#!/usr/bin/env python
# license removed for brevity
#import rospy

# this particle filter only uses GPS
# it only works well right now if GPS measurements are constantly moving
# (ie are not holding for many values then jumping)

#import matplotlib.pyplot as plt
import numpy as np
import math as m
from scipy.stats import norm

"""
Inputs: numpy arrays X, Y, Psi, v, del_f
Outputs: numpy arrays Xout, Yout, Psiout
"""
"""
num_particles = 5
X = np.array(([1,1,1,1,1]))
Y = X + 10
Psi = Y + 10
v = Psi + 10
del_f = 0.3
"""

def roughening(particles,num_particles,K,d):
    E_X = max(particles[0,:]) - min(particles[0,:])
    #print E_X
    E_Y = max(particles[1,:]) - min(particles[1,:])
    E_Psi = max(particles[2,:]) - min(particles[2,:])
    
    sigma_X = K*E_X*num_particles**(-1/d)  
    sigma_Y = K*E_Y*num_particles**(-1/d)
    sigma_Psi = K*E_Psi*num_particles**(-1/d)
    
    #particles[0,:] = particles[0,:] + np.random.normal(0.,sigma_X,num_particles)
    #particles[1,:] = particles[1,:] + np.random.normal(0.,sigma_Y,num_particles)
    #particles[2,:] = particles[2,:] + np.random.normal(0.,sigma_Psi,num_particles)
    
        
    particles[0,:] = particles[0,:] + np.random.uniform(-m.sqrt(3)*sigma_X,
    m.sqrt(3)*sigma_X,num_particles)
    particles[1,:] = particles[1,:] + np.random.uniform(-m.sqrt(3)*sigma_Y,
    m.sqrt(3)*sigma_Y,num_particles)
    particles[2,:] = particles[2,:] + np.random.uniform(-m.sqrt(3)*sigma_Psi,
    m.sqrt(3)*sigma_Psi,num_particles)
    
    return particles


def particle_filter(gps_present, rate, X, Y, Psi, v, del_f):
    num_particles = 100    
    
    # real sensor values
    #sensor_noise = 0.03
    #straight_noise = 0.02
    #turn_noise = 0.002
    #psi_noise = 0.001
    
    sensor_noise = 0.3
    psi_noise = 0.1
    straight_noise = 0.03
    turn_noise = 0.001
    
    
    try:
        dummy = particle_filter.particles[0]
    except:
        particle_filter.particles = np.zeros([4,num_particles])
        
        X_particle_values = X + np.random.normal(0, straight_noise,
        num_particles)
        Y_particle_values = Y + np.random.normal(0, straight_noise,
        num_particles)
        Psi_particle_values = Psi + np.random.normal(0,turn_noise,
        num_particles)
        particle_filter.particles = np.concatenate(([X_particle_values], [Y_particle_values],
                      [Psi_particle_values], 
                      [np.ones(num_particles)/num_particles]),axis=0)
        
        #particles_transpose = np.transpose(particle_filter.particles)       
        #best_particle = particles_transpose[int(round(
        #(num_particles - 1)*np.random.random_sample())),:]
        
        X_out = X
        Y_out = Y
        Psi_out = Psi
        return (X_out,Y_out,Psi_out)
        
    
    lf = 1.105
    lr = 1.738
    dt = 1./rate
    
    X_vals = particle_filter.particles[0,:]
    Y_vals = particle_filter.particles[1,:]        
    Psi_vals = particle_filter.particles[2,:]
    X_new = np.zeros((1,num_particles))
    Y_new = np.zeros((1,num_particles))
    Psi_new = np.zeros((1,num_particles))
    
    # kinematics bicycle model
    beta = m.atan(lr*m.tan(del_f)/(lr+lf))*np.ones(num_particles)
    dx = np.multiply(v,np.cos(Psi_vals + beta))
    dy = np.multiply(v,np.sin(Psi_vals + beta))
    dpsi = np.multiply(v,np.sin(beta)/lr)
    
    X_new = X_vals + dt*dx + np.random.normal(0,straight_noise,
        num_particles) * np.cos(Psi_vals + beta)
    Y_new = Y_vals + dt*dy + np.random.normal(0,straight_noise,
        num_particles) * np.sin(Psi_vals + beta)
    Psi_new = np.mod(Psi_vals + dt*dpsi + np.random.normal(0,turn_noise,num_particles),2*m.pi)
    
    particle_filter.particles = np.concatenate(([X_new], [Y_new],
                      [Psi_new], 
                      [particle_filter.particles[3,:]]),axis=0)

    ### Resample if GPS is present
    if (gps_present == 1):
        
                    
        ones = np.ones(num_particles)
        
        particle_filter.particles[3,:] = norm.pdf(X*ones,particle_filter.particles[0,:], sensor_noise*ones) * \
            norm.pdf(Y*ones,particle_filter.particles[1,:], sensor_noise*ones) *\
            norm.pdf(Psi*ones,particle_filter.particles[2,:], psi_noise*ones)
        
        sum_particles = sum(particle_filter.particles[3,:])
        if sum_particles > 0:
            particle_filter.particles[3,:] = particle_filter.particles[3,:]/sum_particles
        
    
            #best_particle = particle_filter.particles[:,np.argmax(particle_filter.particles[3,:])]
            new_particles = np.zeros([4,num_particles])    
            
            for i in range(num_particles):
                tmp1 = np.random.uniform(0,1.0,1)[0]
                tmp2 = particle_filter.particles[3,0]
                count = 0
                
                while (tmp2 < tmp1 and count < num_particles-1):
                    count = count + 1
                    tmp2 = tmp2 + particle_filter.particles[3,count]
                    
                new_particles[:,i] = particle_filter.particles[:,count]
            particle_filter.particles = new_particles
    
        # roughening
            particle_filter.particles = roughening(particle_filter.particles,num_particles,
                                               0.3,3)
        else:
            particle_filter.particles = np.zeros([4,num_particles])  
            
            X_particle_values = X + np.random.normal(0, straight_noise,
            num_particles)
            Y_particle_values = Y + np.random.normal(0, straight_noise,
            num_particles)
            Psi_particle_values = Psi + np.random.normal(0,turn_noise,
            num_particles)
            particle_filter.particles = np.concatenate(([X_particle_values], [Y_particle_values],
                          [Psi_particle_values], 
                          [np.ones(num_particles)/num_particles]),axis=0)
                                          
        
    X_out = np.mean(particle_filter.particles[0,:])
    Y_out = np.mean(particle_filter.particles[1,:])    
    Psi_out = np.mean(particle_filter.particles[2,:])  
        
    
    X_dif = max(particle_filter.particles[0,:]) - min(particle_filter.particles[0,:])
    Y_dif = max(particle_filter.particles[1,:]) - min(particle_filter.particles[1,:])
    
    #print X_dif
    
    #plt.figure(1)
    #plt.clf()
    ##plt.plot(X_dif,'.')
    
    return (X_out,Y_out,Psi_out) 

    
