# -*- coding: utf-8 -*-
"""
Created on Thu Nov 18 00:45:54 2021

@author: clayt
"""

import opensim
import numpy as np
import glob
import os
from scipy.spatial.transform import Rotation as R

def str_vec3_to_list(vec3str):
    vec3str = vec3str.replace("~", "").replace("[", "").replace("]", "")
    vec3str = vec3str.split(",")
    vec3str = [float(e) for e in vec3str]
    
    return vec3str

def list_to_vec3(l):
    return opensim.Vec3(*l)

def scale_body(body, model, init_state, scale_multiplier):
    scaleset = opensim.ScaleSet()
    scale    = opensim.Scale()
    scale.setSegmentName(body.getName())
    scale.setScaleFactors(opensim.Vec3(scale_multiplier))
    scale.setApply(True)
    scaleset.adoptAndAppend(scale)
    model.scale(init_state, scaleset, False)

    
    
def scale_hand(model, init_state, hand_scale_multiplier, ring_percentage_of_index):
    # Default finger lengths (in meters)
    finger_lengths = {
        "thumb": 0.098712307173665,
        "index": 0.08583738259532561,
        "middle": 0.09193367309746416,
        "ring": 0.08336285044453702,
        "pinky": 0.07418052848847119
    }
    
    thumb_length = finger_lengths["thumb"]
    index_length = finger_lengths["index"]
    middle_length = finger_lengths["middle"]
    ring_length = finger_lengths["ring"]
    pinky_length = finger_lengths["pinky"]
    
    desired_ring_length = ring_percentage_of_index*index_length
    ring_ratio          = desired_ring_length/ring_length
    
    for body in model.getBodyList():
        if body.getName() not in ["distph4", "proxph4", "midph4"]:
            scale_body(body,  model, init_state, hand_scale_multiplier)
            
    for body in model.getBodyList():
        if body.getName() in ["distph4", "proxph4", "midph4"]:
            scale_body(body, model, init_state, ring_ratio)
            
def get_marker_locations_from_model(model):
    state = model.initSystem()
    l     = []
    names = []
    data_dict = {}
    for marker in model.getMarkerSet():
        names.append(marker.getName())
        l.append(list(marker.getLocationInGround(state).to_numpy()))
        data_dict[marker.getName()] = list(marker.getLocationInGround(state).to_numpy())
        
    return data_dict

    
def rotation_matrix_in_str_to_quaternion_list(s):
    s_ = s.split("\n")[1:4]
    out = []
    for e in s_:
        e = e[1:-1]
        e = e.split(",")
        e = [float(el) for el in e]
        out.append(e)
        
    out    = np.asarray(out)
    out    = np.linalg.inv(out)
    matrix = R.from_matrix(out)
    quaternion = matrix.as_quat()
        
    return list(quaternion)

def simulate(model_path, motion_path, scale_multiplier = 1.112, ring_scale_multiplier = 1):
    model          = opensim.Model(model_path)
    state          = model.initSystem()
    
    if scale_multiplier != 1.112:
        scale_hand(model, state, scale_multiplier, ring_scale_multiplier)

    model.set_gravity(opensim.Vec3(0,0,-9.81))
    
    marker_names = []
    all_frames   = []
    all_marker_locations  = []
    
    for marker in model.getMarkerSet():
        frame  = marker.getParentFrame()
        marker_frame = opensim.PhysicalOffsetFrame()
        marker_frame.setParentFrame(frame)
        marker_location = marker.get_location()
        marker_frame.setOffsetTransform(opensim.Transform(marker_location)) 
        marker.addComponent(marker_frame)
        marker_names.append(marker.getName())
        all_frames.append(marker_frame)
        all_marker_locations.append(str_vec3_to_list(marker_location.toString()))
        model.initSystem()
    
    marker_names_acc_x = [e + "_ACC_X" for e in marker_names]
    marker_names_acc_y = [e + "_ACC_Y" for e in marker_names]
    marker_names_acc_z = [e + "_ACC_Z" for e in marker_names]
    marker_names_gyro_x = [e + "_GYRO_X" for e in marker_names]
    marker_names_gyro_y = [e + "_GYRO_Y" for e in marker_names]
    marker_names_gyro_z = [e + "_GYRO_Z" for e in marker_names]
    marker_names_qua_x = [e + "_QUA_X" for e in marker_names]
    marker_names_qua_y = [e + "_QUA_Y" for e in marker_names]
    marker_names_qua_z = [e + "_QUA_Z" for e in marker_names]
    marker_names_qua_w = [e + "_QUA_W" for e in marker_names]
    
    
    acc_header = np.reshape(np.transpose(np.reshape([marker_names_acc_x + marker_names_acc_y + marker_names_acc_z], (3,-1)), axes = (1, 0)), (-1)).tolist()
    gyro_header = np.reshape(np.transpose(np.reshape([marker_names_gyro_x + marker_names_gyro_y + marker_names_gyro_z], (3,-1)), axes = (1, 0)), (-1)).tolist()
    qua_header = np.reshape(np.transpose(np.reshape([marker_names_qua_x + marker_names_qua_y + marker_names_qua_z + marker_names_qua_w], (4,-1)), axes = (1, 0)), (-1)).tolist()
    
    header = acc_header + gyro_header + qua_header
    
    with open(motion_path, "r") as f:
        lines = f.readlines()
        end_time = float(lines[-1].split("\t")[0])
        time_inc = end_time/(len(lines[11:])-1)
    
    analyze = opensim.AnalyzeTool(model)
    analyze.setName('analyze')
    analyze.setCoordinatesFileName(motion_path)
    analyze.loadStatesFromFile(state)
    
    reporter = opensim.StatesReporter()       
    reporter.setStartTime(0)
    reporter.setEndTime(end_time)
        
    analyze.updAnalysisSet().cloneAndAppend(reporter)    
    analyze.setPrintResultFiles(True)
    
    analyze.setStartTime(0)
    analyze.setFinalTime(end_time)
    analyze.run()
    
    states = opensim.StatesTrajectory.createFromStatesStorage(model, analyze.getStatesStorage())
    
    gyroscope = {}
    velocity  = {}
    gravity   = {}
    orientation = {}
    
    for marker_name in marker_names:
        gyroscope[marker_name] = []
        velocity[marker_name]  = []
        gravity[marker_name]   = []
        orientation[marker_name]   = []
        
    for itime in range(states.getSize()):
        state = states[itime]
        #time = state.getTime()
        model.realizeReport(state)

        for marker in model.getMarkerSet():
            #ground_vec     = marker.getVelocityInGround(state)
            marker_frame   = marker.getComponent("physicaloffsetframe")
            
            ground_vec     = marker.getVelocityInGround(state)
            ground_ang_vel = marker_frame.getAngularVelocityInGround(state)
            rot            = marker_frame.getTransformInGround(state).R()
            marker_name    = marker.getName()
            
            velocity_entry  = ground_vec.toString()
            gyroscope_entry = ground_ang_vel.toString()
            orientation_entry = rot.toString()
    
            gyroscope[marker_name].append(str_vec3_to_list(gyroscope_entry))
            velocity[marker_name].append(str_vec3_to_list(velocity_entry))
            orientation[marker_name].append(rotation_matrix_in_str_to_quaternion_list(orientation_entry))
        
    index = 0
    for key, value in gyroscope.items():
        if index == 0:
            total_gyro = np.asarray(value)
        else:
            total_gyro = np.concatenate([total_gyro, np.asarray(value)], axis = 1)
        index += 1
     
    index = 0
    for key, value in velocity.items():
        if index == 0:
            total_vec = np.asarray(value)
        else:
            total_vec = np.concatenate([total_vec, np.asarray(value)], axis = 1)
        index += 1
        
    index = 0
    for key, value in orientation.items():
        if index == 0:
            total_ori = np.asarray(value)
        else:
            total_ori = np.concatenate([total_ori, np.asarray(value)], axis = 1)
        index += 1 
    
    total_acc = np.gradient(total_vec, time_inc, axis = 0)
    total     = np.concatenate([total_acc, total_gyro,
                                total_ori], axis = 1)
    total     = np.round(total, 6)
    save_path = motion_path.replace("generated_mot", "simulations_raw")
    
    if scale_multiplier == 1.112:
        save_path = save_path.replace(".mot", ".csv")
    else:
        save_path = save_path.replace(".mot", str(scale_multiplier) + ".csv")
    
    base_path = save_path.replace(os.path.basename(save_path), "")
    if not os.path.exists(base_path):
        os.makedirs(base_path)
    
    with open(save_path, "w") as f:
        f.write(",".join(header) + "\n")
        for i in range(0, total.shape[0]):
            line = [str(e) for e in total[i]]
            line = ",".join(line) + "\n"
            f.write(line)
                
    print("Concluded: " + motion_path)

def is_query_present(filter_query, filename):
    for e in filter_query:
        if e in filename:
            return True
        
    return False

def main():
    # Define paths  
    scale_multiplier = 1.112
    motion_paths = glob.glob("./examples/interhand/generated_mot/**/*.mot", recursive=True)  
    model_path = r"models/modified/hand_model_nomuscles.osim"

    # Run simulation for each motion file  
    for motion_path in motion_paths:  
        simulate(model_path, motion_path, scale_multiplier=scale_multiplier)

if __name__ == "__main__":
    main()