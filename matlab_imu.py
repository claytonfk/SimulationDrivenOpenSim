# -*- coding: utf-8 -*-
"""
Created on Thu Jan 20 00:43:01 2022

@author: clayt
"""

import numpy as np
import math
import os
import matlab.engine
from scipy.spatial.transform import Rotation as R
import glob
import pandas as pd

# Start MATLAB engine
eng = matlab.engine.start_matlab()

# Define paths
SIM_FILES_PATH = "./examples/interhand/simulations_raw/**/*.csv"
DATASET_NAME = "interhand"
OUTPUT_FOLDER = "processed_simulations"

# Retrieve all simulation CSV files
sim_files = glob.glob(SIM_FILES_PATH, recursive=True)


def get_imu_matlab(accel_data, gyro_data, sample_time):
    """
    Simulates IMU sensor readings (accelerometer, gyroscope, and magnetometer)
    using MATLAB's imuSensor function.
    
    Args:
        accel_data (numpy.ndarray): Acceleration data.
        gyro_data (numpy.ndarray): Gyroscope data.
        sample_time (float): Sample time.

    Returns:
        numpy.ndarray: Simulated IMU readings.
    """
    eng.workspace['a'] = matlab.double(accel_data.tolist())
    eng.workspace['b'] = matlab.double(gyro_data.tolist())
    eng.workspace['c'] = matlab.double(sample_time)

    eng.workspace['IMU'] = eng.eval("imuSensor('accel-gyro-mag')")
    imu_output = eng.eval("IMU(a, b, c)", nargout=3)

    return np.asarray(imu_output)


def get_unique_sim_file(sim_file):
    """
    Extracts the unique identifier from a simulation file name.

    Args:
        sim_file (str): File name.

    Returns:
        str: Unique identifier.
    """
    return ".".join(sim_file.split(".")[:2])

if __name__ == "__main__":

    # Process
    unique_filenames = {}
    saved_files  = []
    
    for sim_file in sim_files:
        
        # THIS WILL ASSIGN CLASSES TO THE FILES
        # Dictionary mapping file keywords to class labels
        class_mapping = {
            "thumbup": 0,
            "fist": 1,
            "rocker": 2,
            "fingerspread": 3
        }
        
        # Assign class based on filename
        cl = next((class_mapping[key] for key in class_mapping if key in sim_file), -1)

        data        = pd.read_csv(sim_file)
        num_sensors = int(data.shape[1]/10)
    
        columns     = list(data.columns)
        values      = data.values
        
        if cl != -1:
            combined = []
            combined_header = []
            
            for sensor_idx in range(num_sensors):
                # Extract sensor-specific data from values array
                acc_start, gyro_start, qua_start = 3 * sensor_idx, 3 * num_sensors, 6 * num_sensors
                sensor_acc = values[:, acc_start : acc_start + 3]
                sensor_gyro = values[:, gyro_start + acc_start : gyro_start + acc_start + 3]
                sensor_qua = values[:, qua_start + acc_start : qua_start + acc_start + 4]  # Quaternion (4 values)
            
                # Convert quaternion to rotation matrices
                sensor_matrix = []
                for q in range(0, sensor_qua.shape[0]):
                    matrix = R.from_quat(sensor_qua[q]).as_matrix().tolist()
                    sensor_matrix.append(matrix)
                
                sensor_matrix = np.reshape(sensor_matrix, (3,3,-1)).tolist()
            
                # Simulate IMU data using MATLAB
                imu_data = get_imu_matlab(sensor_acc, sensor_gyro, sensor_matrix)  # Returns [acc, gyro, mag]
            
                # Reshape IMU data to match expected structure
                imu_data = np.transpose(imu_data, axes=[1, 0, 2])  # Swap axes for correct alignment
                imu_data = imu_data.reshape(-1, 9)  # Flatten into [time_steps, 9] (acc, gyro, mag)
            
                # Create column headers for the sensor
                sensor_header = columns[acc_start : acc_start + 3] + \
                                columns[gyro_start + acc_start : gyro_start + acc_start + 3] + \
                                [col.replace("GYRO", "MAG") for col in columns[gyro_start + acc_start : gyro_start + acc_start + 3]]
            
                # Append sensor data to combined array
                if sensor_idx == 0:
                    combined = imu_data
                    combined_header = sensor_header
                else:
                    combined = np.concatenate([combined, imu_data], axis=1)
                    combined_header += sensor_header  # Extend header list
        

            output_dir = f"./{DATASET_NAME}/{OUTPUT_FOLDER}"
            os.makedirs(output_dir, exist_ok=True)
            
            # Process each simulation file
            unique_filename = get_unique_sim_file(sim_file) 
            
            # Maintain a list of saved unique filenames
            if unique_filename not in saved_files:
                saved_files.append(unique_filename)
            
            # Assign a unique file ID based on its index in saved_files
            file_id = saved_files.index(unique_filename)
            save_filename = f"{output_dir}/class{cl}_fileid{file_id}.csv"
            
            # Save the processed data
            print(f"Saving: {save_filename}")
            pd.DataFrame(combined).to_csv(save_filename, header=combined_header, index=False)
                        
            