# -*- coding: utf-8 -*-
"""
Created on Fri Nov 06 14:01:23 2015

@author: Brian
"""

import numpy as np
#import loader as ld
import UnpackCANData as ld2
import particle_filter as pf
import merged_filter as mf
import csv

#filepath = '../../../../Data Files/20140722_winding_track_good.mat'
filepath = '../../../../Data Files/CAN/CPG_Oval/t1.mat'


def writeCSV(fileout, X_out, Y_out, Psi_out):
    with open(fileout,'wb') as csvfile:
        writer = csv.writer(csvfile,delimiter = ',',lineterminator='\n',)
        writer.writerow(['X','Y','Psi'])
        for x in zip(X_out,Y_out,Psi_out):
            writer.writerow([x[0],x[1],x[2]])

# accept every GPS_accept^th measurement
GPS_accept = 5

# same as Simulink model
num_particles = 100

#(X,Y,Psi,v,del_f) = ld.loader(filepath)
#(X,Y,Psi,v,del_f) = ld2.load(filepath)
(X,Y,Psi,v,del_f,a_l0,a_l1,a_l2,a_l3,a_r0,a_r1,a_r2,a_r3,a_l_quality,
 a_r_quality) = ld2.load2(filepath)
 
# slice arrays to match Matlab 
dt=0.05
dt_CAN=.001

X = X[0:len(X)-1:dt/dt_CAN]
Y = Y[0:len(Y)-1:dt/dt_CAN]
Psi = Psi[0:len(Psi)-1:dt/dt_CAN]
v = v[0:len(v)-1:dt/dt_CAN]
del_f = del_f[0:len(del_f)-1:dt/dt_CAN]

a_l0 = a_l0[0:len(a_l0)-1:dt/dt_CAN]
a_l1 = a_l1[0:len(a_l1)-1:dt/dt_CAN]
a_l2 = a_l2[0:len(a_l2)-1:dt/dt_CAN]
a_l3 = a_l3[0:len(a_l3)-1:dt/dt_CAN]

a_r0 = a_r0[0:len(a_r0)-1:dt/dt_CAN]
a_r1 = a_r1[0:len(a_r1)-1:dt/dt_CAN]
a_r2 = a_r2[0:len(a_r2)-1:dt/dt_CAN]
a_r3 = a_r3[0:len(a_r3)-1:dt/dt_CAN]

length = len(X)
X_out = np.zeros(length)
Y_out = np.zeros(length)
Psi_out = np.zeros(length)

for i in range(length):
    if i % GPS_accept == 0:
        (X_out[i], Y_out[i], Psi_out[i]) = mf.merged_filter(1,X[i],Y[i],
        Psi[i],v[i],del_f[i],a_r0[i],a_r1[i], a_r2[i], a_r3[i], a_l0[i], a_l1[i],
        a_l2[i], a_l3[i], a_r_quality[i], a_l_quality[i])
    else:
        (X_out[i], Y_out[i], Psi_out[i]) = mf.merged_filter(0,X[i],Y[i],
        Psi[i],v[i],del_f[i],a_r0[i],a_r1[i], a_r2[i], a_r3[i], a_l0[i], a_l1[i],
        a_l2[i], a_l3[i], a_r_quality[i], a_l_quality[i])
    #(X_out[i], Y_out[i], Psi_out[i]) = pf.particle_filter(num_particles,X[i],Y[i],
    #Psi[i],v[i],del_f[i])
    

#fileout = '../log/GPS_winding_python.csv'
fileout = '../log/oval_t1_python_5.csv'
writeCSV(fileout,X_out,Y_out,Psi_out)
#writeCSV(fileout,X,Y,Psi)

