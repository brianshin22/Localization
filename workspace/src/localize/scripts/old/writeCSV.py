# -*- coding: utf-8 -*-
"""
Created on Thu Mar 10 13:31:49 2016

@author: Kilian
"""


import csv

def writeCSV(fileout, time, X_out, Y_out, Psi_out):
    with open(fileout,'wb') as csvfile:
        writer = csv.writer(csvfile,delimiter = ',',lineterminator='\n',)
        writer.writerow(['t','X','Y','Psi'])
        for x in zip(time,X_out,Y_out,Psi_out):
            writer.writerow([x[0],x[1],x[2],x[3]])
            
            