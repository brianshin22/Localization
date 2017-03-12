# -*- coding: utf-8 -*-
"""
loader.py

Loads the winding track good data
The file is not CAN data but is a Matlab struct that is manually unpacked
Unpacking CAN data will be different

Created on Wed Nov 04 23:02:17 2015

@author: Brian
"""

import scipy.io as io
import math as m

# filepath = '../../../../Data Files/20140722_winding_track_good.mat'

def loader(filepath):
    
    data = io.loadmat(filepath)
    time = data['winding_track_good']['X'][0][0][0]['Data'][0][0]
    
    steer_scale = 14.5
    
    # check mat file for structure and indices of desired data
    
    # GPS_Local_Pos Out111
    X = data['winding_track_good']['Y'][0][0][0]['Data'][2][0]
    
    # GPS_Local_Pos Out112
    Y = data['winding_track_good']['Y'][0][0][0]['Data'][3][0]
    
    # INS_Heading Out1
    Psi = data['winding_track_good']['Y'][0][0][0]['Data'][5][0]
    
    v = data['winding_track_good']['Y'][0][0][0]['Data'][119][0]
    
    del_f = (m.pi/(180*steer_scale)) * data['winding_track_good']['Y'][0][0][0]['Data'][173][0]
    
        
    return (X,Y,Psi,v,del_f)
    
    