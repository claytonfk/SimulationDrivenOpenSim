# -*- coding: utf-8 -*-
"""
Created on Tue Nov 16 23:40:23 2021

@author: clayt
"""

import opensim, math, os
import numpy as np
import glob


def run_ik(model, marker_path, save_dir, name):  
    model.finalizeFromProperties()    
    _  = model.initSystem()
    
    ikTool = opensim.InverseKinematicsTool() #'iksetting.xml')
    ikTool.setModel(model)
    markerData = opensim.MarkerData(marker_path)
    

    initial_time = markerData.getStartFrameTime()
    final_time   = markerData.getLastFrameTime()
    
    
    save_path = os.path.join(save_dir, name + ".mot")
    ikTool.setName(name)
    ikTool.set_accuracy(1e-5)
    ikTool.setMarkerDataFileName(marker_path)
    

    ikTool.setStartTime(initial_time)
    ikTool.setEndTime(final_time)
    ikTool.setOutputMotionFileName(save_path)
    ikTool.set_report_marker_locations(True)
    ikTool.set_results_directory(save_dir)
    
    try:
        ikTool.run()
    except Exception as e:
        pass
    
# Define file paths
trc_file_pattern = "./examples/interhand/processed_trc/**/*.trc"
model_path = r"./models/modified/hand_model_nomuscles.osim"

# Load the OpenSim model
model = opensim.Model(model_path)

# Find all TRC files
trc_files = glob.glob(trc_file_pattern, recursive=True)

# Process each TRC file
for trc_file in trc_files:
    # Extract filename and create the corresponding save directory
    file_name = os.path.basename(trc_file)
    save_dir = os.path.dirname(trc_file).replace("processed_trc", "generated_mot")
    
    # Ensure the save directory exists
    os.makedirs(save_dir, exist_ok=True)

    # Run inverse kinematics (IK) and save the result
    run_ik(model, trc_file, save_dir, file_name)
    
    # Print the result
    print(f"Processed: {os.path.join(save_dir, file_name)}")