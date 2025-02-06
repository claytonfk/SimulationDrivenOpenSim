# -*- coding: utf-8 -*-
"""
Created on Sat Mar 12 02:00:06 2022

@author: clayt
"""

import opensim
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter
import glob
import os
from scipy.spatial.transform import Rotation as R
import pandas as pd
import scipy
import cv2
import math

def read_error_files(paths):
    average = []
    
    for path in paths:
        d = pd.read_csv(path, header = 6, sep = "\t")    
        average.append(np.mean(d.values[:, 2]))
        
    return np.mean(average), np.std(average)

def get_coordinate_constraints(model):
    d = {}
    for coord in model.getCoordinateSet():
        name = coord.getName()
        minmax = [ coord.getRangeMin(), coord.getRangeMax()]
        d[name] = minmax
        
    return d

def get_marker_error(ik_paths, trc_paths):
    
    distance_errors = []
    num = 0
    
    for ik_path, trc_path in zip(ik_paths, trc_paths):
        ik_data = pd.read_csv(ik_path, header = 6, sep='\t')
        trc_data = pd.read_csv(trc_path, header = 4, sep='\t')
        with open(trc_paths[0], "r") as f:
            lines = f.readlines()
        trc_header = lines[3].replace("\n", "").split("\t")[2:]
        mod_trc_header = []
        
        for t in trc_header:           
            mod_trc_header.append(t + "_tx")
            mod_trc_header.append(t + "_ty")
            mod_trc_header.append(t + "_tz")
            
        trc_data.columns = mod_trc_header
        
        mod_trc_header = [a for a in mod_trc_header if "WRIST" not in a and "THUMB_CMC" not in a]
        trc_header =  [a for a in trc_header if "WRIST" not in a and "THUMB_CMC" not in a]

        ik_data = ik_data[mod_trc_header]
        trc_data = trc_data[mod_trc_header]

        for t in trc_header:
            marker_header = [t + "_tx", t + "_ty", t + "_tz"]
            distance_squared = np.sum(np.square(ik_data[marker_header].values - trc_data[marker_header].values), axis = 1)
            distance = np.sqrt(distance_squared)
            distance_errors = np.concatenate([distance_errors, distance], axis = 0)
        
        num  += ik_data.shape[0]*ik_data.shape[1]

    return distance_errors, num
        

def read_mot_file_for_violation(mots, coordnames, constraints):
    total_steps = 0
    num_violations = 0
    angles = []
    
    for mot in mots:
        data = pd.read_csv(mot, sep = "\t", skiprows = 10)
        data = data[coordnames]

        
        total_steps += data.shape[0]
    
        for coordname in coordnames:
            data[coordname] =  data[coordname] % 360
            mask            = data[coordname] > 180
            data[coordname] = data[coordname] - 360*mask
            data[coordname] = data[coordname]*math.pi/180

            
            minv, maxv = constraints[coordname]
            max_violations = data[coordname] > constraints[coordname][1]
            min_violations = data[coordname] < constraints[coordname][0]
            num_violations += np.sum(min_violations) 
            num_violations += np.sum(max_violations)
            
            min_angle_violations = np.maximum(constraints[coordname][0] - data[coordname], 0)
            max_angle_violations = np.maximum(data[coordname] - constraints[coordname][1], 0)
            
            angle_violation = max_angle_violations + min_angle_violations
            angle_violation = angle_violation[angle_violation != 0].values.tolist()
            
            for k in angle_violation:
                if k > 10:
                    print(mot)
        
            
            angles += angle_violation
            
    violation_freq = num_violations/(total_steps*len(coordnames))
        
    return num_violations, violation_freq, angles

def is_query_present(filter_query, filename):
    for e in filter_query:
        if e in filename:
            return True
        
    return False
    

# File paths for various data
motion_files = glob.glob("./examples/interhand/generated_mot/**/*.mot", recursive=True)
ik_error_files = glob.glob("./examples/interhand/generated_mot/**/*.trc_ik_marker_errors", recursive=True)
processed_trc_files = glob.glob("./examples/interhand/processed_trc/**/*.trc", recursive=True)

# Ensure processed TRC files correspond to motion files
processed_trc_files = [file.replace("generated_mot", "processed_trc").replace(".mot", "") for file in motion_files]

# Paths to marker location files
marker_location_files = glob.glob("./examples/interhand/generated_mot/**/*.trc_ik_model_marker_locations", recursive=True)

# Model path for OpenSim
model_path = r"models/modified/hand_model_nomuscles.osim"

# Load the human model in OpenSim
model = opensim.Model(model_path)

# Define the joint angles (coordinates) for the analysis
joint_coordinates = [
    "cmc_flexion", "cmc_abduction", "mp_flexion", "ip_flexion", "mcp2_flexion", 
    "mcp2_abduction", "pm2_flexion", "md2_flexion", "mcp3_flexion", "mcp3_abduction", 
    "pm3_flexion", "md3_flexion", "mcp4_flexion", "mcp4_abduction", "pm4_flexion", 
    "md4_flexion", "mcp5_flexion", "mcp5_abduction", "pm5_flexion", "md5_flexion"
]

# Compute marker errors from the IK marker location files
marker_errors = get_marker_error(marker_location_files, processed_trc_files)
print(f"Avg. marker error [m]: {np.mean(marker_errors[0]):.4f}")
print(f"Std. marker error [m]: {np.std(marker_errors[0]):.4f}")

# Read motion files and check for constraint violations
violations_count, violation_frequency, violation_angles = read_mot_file_for_violation(
    motion_files, joint_coordinates, get_coordinate_constraints(model)
)

# Print results for constraint violations
print(f"Number of constraint violations: {violations_count}")
print(f"Constraint violation frequency: {violation_frequency:.4f}")
print(f"Avg. angle of constraint violation: {np.mean(violation_angles) * 180 / math.pi:.2f}°")
print(f"Std. angle of constraint violation: {np.std(violation_angles) * 180 / math.pi:.2f}°")

