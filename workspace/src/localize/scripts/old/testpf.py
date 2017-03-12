# -*- coding: utf-8 -*-
"""
Created on Thu Feb 18 03:05:21 2016

@author: Brian
"""

import matplotlib.pyplot as plt
import numpy as np
import csv
import particle_filter as pf
import UnpackCANData as ld

filepath = '../../../../Data Files/15_39_sim.mat'
#filepath = '../../../../Data Files/CAN/CPG_Oval/t1.mat'

def writeCSV(fileout, time, X_out, Y_out, Psi_out):
    with open(fileout,'wb') as csvfile:
        writer = csv.writer(csvfile,delimiter = ',',lineterminator='\n',)
        writer.writerow(['t','X','Y','Psi'])
        for x in zip(time,X_out,Y_out,Psi_out):
            writer.writerow([x[0],x[1],x[2],x[3]])


(rawtime, rawX,rawY,rawPsi,rawvelocity,rawdel_f) = ld.load(filepath)



data_dt = rawtime[1] - rawtime[0]
data_rate = 1./data_dt
filter_rate = 50
filter_dt = 1./ filter_rate

length = 5000
#length = int(len(rawX)*data_rate/filter_rate)

time = np.zeros(length)
X = np.zeros(length)
Y = np.zeros(length)
Psi = np.zeros(length)
velocity= np.zeros(length)
del_f = np.zeros(length)


for i in range(length):
    sam = i*(data_rate/filter_rate)
    time[i] = rawtime[sam]
    X[i] = rawX[sam]
    #print (sam)
    Y[i] = rawY[sam]
    Psi[i] = rawPsi[sam]
    velocity[i] = rawvelocity[sam]
    del_f[i] = rawdel_f[sam]

X_out = np.zeros(length)
Y_out = np.zeros(length)
Psi_out = np.zeros(length)
particles = np.zeros([length, 4,100])
#X_dist = np.zeros(length)
#Y_dist = np.zeros(length)


for i in range(length):
    (X_out[i],Y_out[i],Psi_out[i]) = pf.particle_filter(1,
    filter_rate,X[i],Y[i],Psi[i],velocity[i],del_f[i])
    #print(i)
    #print("\n")        
        


X_cut = X[0:len(X_out)]
Y_cut = Y[0:len(Y_out)]
Psi_cut = Psi[0:len(Psi_out)]

X_dif = X_out - X_cut
Y_dif = Y_out - Y_cut
Psi_dif = Psi_out - Psi_cut

#plt.plot(X_dist, '.')

plt.plot(X_out,Y_out, '.')
plt.plot(X,Y,'r.')
plt.show()




fileout = '../log/15_39_sim_filtered10000-test.csv'
writeCSV(fileout,time,X_out,Y_out,Psi_out)