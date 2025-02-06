# -*- coding: utf-8 -*-
"""
Created on Tue Jan 12 20:43:26 2021

@author: Clayton
"""

import numpy as np
import os, glob, math
from scipy.spatial.transform import Rotation as R
from scipy.signal import savgol_filter

class TRCFile(): 
    def __init__(self, filepath):
        
        file         = open(filepath, "r")
        all_lines    = file.readlines()
        
        self.raw_metadata = all_lines[:5]
        self.header       = all_lines[0].replace("\n", "").split("\t")
        
        self.properties_names   = all_lines[1].replace("\n", "").split("\t")
        self.properties_values  = all_lines[2].replace("\n", "").split("\t")
        self.marker_names       = all_lines[3].replace("\n", "").split("\t")
        
        
        # A QUICK FIX TO THE NAMING SCHEME
        self.marker_coordinates = all_lines[4].replace("\n", "").split("\t")
        
        self.num_markers        = int(self.properties_values[3])
        self.x_indices          = np.asarray([2 + 3*i for i in range(0, self.num_markers)])
        self.y_indices          = self.x_indices + 1
        self.z_indices          = self.x_indices + 2
        
        
        for idx, line in enumerate(all_lines[5:]):
            if idx == 0:
                self.data = self.read_line_of_values(line)
            else:
                if len(line) > 1:
                    line      = self.read_line_of_values(line)
                    self.data = np.concatenate([self.data, line], axis = 0)
        
        file.close()

    def read_line_of_values(self, line):
        line = line.replace("\n", "").split("\t")
        line = [float(k) for k in line]
        return np.expand_dims(np.asarray(line), axis = 0)
    
    def define_origin_marker(self, marker_name, x, y, z):
        marker_id = self.get_marker_id(marker_name)
        indices   = self.get_xyz_indices(marker_id)
        
        for k in range(self.data.shape[0]):
            mx, my, mz = self.data[k, indices]
            self.data[k, self.x_indices] += x - mx
            self.data[k, self.y_indices] += y - my
            self.data[k, self.z_indices] += z - mz

    def get_marker_id(self, marker_name):
        marker_id = 1
        for k in self.marker_names[2:]:
            if k == marker_name:
                return marker_id
            elif k != "":
                marker_id += 1
                
        return -1
    
    def get_xyz_indices(self, marker_id):
        start = 2 + 3*(marker_id - 1)
        end   = start + 3
        return [i for i in range(start, end)]

    def dilate_time(self, k):
        self.data[:, 1] = self.data[:, 1]*k

    def translate(self, x, y, z):
        self.data[:, self.x_indices] += x
        self.data[:, self.y_indices] += y
        self.data[:, self.z_indices] += z
        
    def save_file(self, filepath):
        file = open(filepath, "w")
        file.writelines(self.raw_metadata)
        
        for k in range(0, self.data.shape[0]):
            line = self.data[k, :].tolist()
            line = [str(int(line[0]))]+ [str(e) for e in line[1:]]
            line = "\t".join(line)
            file.write(line + "\n")
        
        file.close()
        
    def reduce_framerate(self, ratio):
        indices   = np.linspace(0, self.data.shape[0] - 1, int(self.data.shape[0]/ratio))
        indices   = [int(e) for e in indices]
        self.data = self.data[indices, :]
        
    def find_normal_to_hand_palm(self):
        marker_1 = "WRIST"
        marker_2 = "INDEX_MCP"
        marker_3 = "RING_MCP"
        
        marker_1 = self.get_marker_id(marker_1)
        marker_2 = self.get_marker_id(marker_2)        
        marker_3 = self.get_marker_id(marker_3)
        
        marker1_indices = self.get_xyz_indices(marker_1)
        marker2_indices = self.get_xyz_indices(marker_2)
        marker3_indices = self.get_xyz_indices(marker_3)
        
        vector_1 = self.data[0, marker1_indices]
        vector_2 = self.data[0, marker2_indices]
        vector_3 = self.data[0, marker3_indices]
         
        Q = vector_1 - vector_2
        R = vector_3 - vector_2
        
        normal = np.cross(Q, R)
        normal = normal/np.linalg.norm(normal)
        
        return normal

        
    def rescale(self, data_dict):
        # Bone lengths from model
        index_mcp = np.asarray(data_dict['INDEX_MCP'])
        ring_mcp = np.asarray(data_dict['RING_MCP'])        
        index_pip = np.asarray(data_dict['INDEX_PIP'])
        index_dip = np.asarray(data_dict['INDEX_DIP'])              
        middle_mcp = np.asarray(data_dict['MIDDLE_MCP'])
        middle_pip = np.asarray(data_dict['MIDDLE_PIP'])        
        middle_dip = np.asarray(data_dict['MIDDLE_DIP'])
        ring_pip = np.asarray(data_dict['RING_PIP'])          
        ring_dip = np.asarray(data_dict['RING_DIP'])
        pinky_mcp = np.asarray(data_dict['PINKY_MCP'])        
        pinky_pip = np.asarray(data_dict['PINKY_PIP'])
        pinky_dip = np.asarray(data_dict['PINKY_DIP'])          
        thumb_cmc = np.asarray(data_dict['THUMB_CMC'])
        thumb_mcp = np.asarray(data_dict['THUMB_MCP'])        
        thumb_ip = np.asarray(data_dict['THUMB_IP'])
        wrist = np.asarray(data_dict['WRIST'])            
        index_tip = np.asarray(data_dict['INDEX_TIP'])
        middle_tip = np.asarray(data_dict['MIDDLE_TIP'])        
        ring_tip = np.asarray(data_dict['RING_TIP'])
        pinky_tip = np.asarray(data_dict['PINKY_TIP'])          
        thumb_tip = np.asarray(data_dict['THUMB_TIP'])
        
        index1 = np.linalg.norm(index_mcp - wrist)
        middle1 = np.linalg.norm(middle_mcp - wrist)
        ring1 = np.linalg.norm(ring_mcp - wrist)
        pinky1 = np.linalg.norm(pinky_mcp - wrist)
        thumb1 = np.linalg.norm(thumb_cmc - wrist)

        index2 = np.linalg.norm(index_pip - index_mcp)
        middle2 = np.linalg.norm(middle_pip - middle_mcp)
        ring2 = np.linalg.norm(ring_pip - ring_mcp)
        pinky2 = np.linalg.norm(pinky_pip - pinky_mcp)
        thumb2 = np.linalg.norm(thumb_mcp - thumb_cmc)

        index3 = np.linalg.norm(index_dip - index_pip)
        middle3 = np.linalg.norm(middle_dip - middle_pip)
        ring3 = np.linalg.norm(ring_dip - ring_pip)
        pinky3 = np.linalg.norm(pinky_dip - pinky_pip)
        thumb3 = np.linalg.norm(thumb_ip - thumb_mcp)

        index4 = np.linalg.norm(index_tip - index_dip)
        middle4 = np.linalg.norm(middle_tip - middle_dip)
        ring4 = np.linalg.norm(ring_tip - ring_dip)
        pinky4 = np.linalg.norm(pinky_tip - pinky_dip)
        thumb4 = np.linalg.norm(thumb_tip - thumb_ip)
        self.rescaled_data = np.copy(self.data)
        
        wrist_indices = self.get_xyz_indices(self.get_marker_id("WRIST"))
        index_mcp_indices = self.get_xyz_indices(self.get_marker_id("INDEX_MCP"))
        middle_mcp_indices = self.get_xyz_indices(self.get_marker_id("MIDDLE_MCP"))  
        ring_mcp_indices = self.get_xyz_indices(self.get_marker_id("RING_MCP"))
        pinky_mcp_indices = self.get_xyz_indices(self.get_marker_id("PINKY_MCP"))
        thumb_cmc_indices = self.get_xyz_indices(self.get_marker_id("THUMB_CMC"))
        
        index_pip_indices = self.get_xyz_indices(self.get_marker_id("INDEX_PIP"))
        middle_pip_indices = self.get_xyz_indices(self.get_marker_id("MIDDLE_PIP"))  
        ring_pip_indices = self.get_xyz_indices(self.get_marker_id("RING_PIP"))
        pinky_pip_indices = self.get_xyz_indices(self.get_marker_id("PINKY_PIP"))
        thumb_mcp_indices = self.get_xyz_indices(self.get_marker_id("THUMB_MCP"))
        
        index_dip_indices = self.get_xyz_indices(self.get_marker_id("INDEX_DIP"))
        middle_dip_indices = self.get_xyz_indices(self.get_marker_id("MIDDLE_DIP"))  
        ring_dip_indices = self.get_xyz_indices(self.get_marker_id("RING_DIP"))
        pinky_dip_indices = self.get_xyz_indices(self.get_marker_id("PINKY_DIP"))
        thumb_ip_indices = self.get_xyz_indices(self.get_marker_id("THUMB_IP"))
        
        index_tip_indices = self.get_xyz_indices(self.get_marker_id("INDEX_TIP"))
        middle_tip_indices = self.get_xyz_indices(self.get_marker_id("MIDDLE_TIP"))  
        ring_tip_indices = self.get_xyz_indices(self.get_marker_id("RING_TIP"))
        pinky_tip_indices = self.get_xyz_indices(self.get_marker_id("PINKY_TIP"))
        thumb_tip_indices = self.get_xyz_indices(self.get_marker_id("THUMB_TIP"))
        
        for timestamp in range(0, self.data.shape[0]):
            # First bones scaling
            index1_ratio = np.linalg.norm(self.data[timestamp, index_mcp_indices] - self.data[timestamp, wrist_indices])/index1
            middle1_ratio = np.linalg.norm(self.data[timestamp, middle_mcp_indices] - self.data[timestamp, wrist_indices])/middle1
            ring1_ratio = np.linalg.norm(self.data[timestamp, ring_mcp_indices] - self.data[timestamp, wrist_indices])/ring1
            pinky1_ratio = np.linalg.norm(self.data[timestamp, pinky_mcp_indices] - self.data[timestamp, wrist_indices])/pinky1
            thumb1_ratio = np.linalg.norm(self.data[timestamp, thumb_cmc_indices] - self.data[timestamp, wrist_indices])/thumb1

            
            self.rescaled_data[timestamp, index_mcp_indices] = (self.data[timestamp, index_mcp_indices] - self.data[timestamp, wrist_indices])/index1_ratio + self.rescaled_data[timestamp, wrist_indices]
            self.rescaled_data[timestamp, middle_mcp_indices] = (self.data[timestamp, middle_mcp_indices] - self.data[timestamp, wrist_indices])/middle1_ratio + self.rescaled_data[timestamp, wrist_indices]
            self.rescaled_data[timestamp, ring_mcp_indices] = (self.data[timestamp, ring_mcp_indices] - self.data[timestamp, wrist_indices])/ring1_ratio + self.rescaled_data[timestamp, wrist_indices]
            self.rescaled_data[timestamp, pinky_mcp_indices] = (self.data[timestamp, pinky_mcp_indices] - self.data[timestamp, wrist_indices])/pinky1_ratio + self.rescaled_data[timestamp, wrist_indices]
            self.rescaled_data[timestamp, thumb_cmc_indices] = (self.data[timestamp, thumb_cmc_indices] - self.data[timestamp, wrist_indices])/thumb1_ratio + self.rescaled_data[timestamp, wrist_indices]

            # Second bones rescaling
            index2_ratio = np.linalg.norm(self.data[timestamp, index_pip_indices] - self.data[timestamp, index_mcp_indices])/index2
            middle2_ratio = np.linalg.norm(self.data[timestamp, middle_pip_indices] - self.data[timestamp, middle_mcp_indices])/middle2
            ring2_ratio = np.linalg.norm(self.data[timestamp, ring_pip_indices] - self.data[timestamp, ring_mcp_indices])/ring2
            pinky2_ratio = np.linalg.norm(self.data[timestamp, pinky_pip_indices] - self.data[timestamp, pinky_mcp_indices])/pinky2
            thumb2_ratio = np.linalg.norm(self.data[timestamp, thumb_mcp_indices] - self.data[timestamp, thumb_cmc_indices])/thumb2
            
            self.rescaled_data[timestamp, index_pip_indices] = (self.data[timestamp, index_pip_indices] - self.data[timestamp, index_mcp_indices])/index2_ratio + self.rescaled_data[timestamp, index_mcp_indices]
            self.rescaled_data[timestamp, middle_pip_indices] = (self.data[timestamp, middle_pip_indices] - self.data[timestamp, middle_mcp_indices])/middle2_ratio + self.rescaled_data[timestamp, middle_mcp_indices]
            self.rescaled_data[timestamp, ring_pip_indices] = (self.data[timestamp, ring_pip_indices] - self.data[timestamp, ring_mcp_indices])/ring2_ratio + self.rescaled_data[timestamp, ring_mcp_indices]
            self.rescaled_data[timestamp, pinky_pip_indices] = (self.data[timestamp, pinky_pip_indices] - self.data[timestamp, pinky_mcp_indices])/pinky2_ratio + self.rescaled_data[timestamp, pinky_mcp_indices]
            self.rescaled_data[timestamp, thumb_mcp_indices] = (self.data[timestamp, thumb_mcp_indices] - self.data[timestamp, thumb_cmc_indices])/thumb2_ratio + self.rescaled_data[timestamp, thumb_cmc_indices]

            # Third bones rescaling
            index3_ratio = np.linalg.norm(self.data[timestamp, index_dip_indices] - self.data[timestamp, index_pip_indices])/index3
            middle3_ratio = np.linalg.norm(self.data[timestamp, middle_dip_indices] - self.data[timestamp, middle_pip_indices])/middle3
            ring3_ratio = np.linalg.norm(self.data[timestamp, ring_dip_indices] - self.data[timestamp, ring_pip_indices])/ring3
            pinky3_ratio = np.linalg.norm(self.data[timestamp, pinky_dip_indices] - self.data[timestamp, pinky_pip_indices])/pinky3
            thumb3_ratio = np.linalg.norm(self.data[timestamp, thumb_ip_indices] - self.data[timestamp, thumb_mcp_indices])/thumb3
            
            self.rescaled_data[timestamp, index_dip_indices] = (self.data[timestamp, index_dip_indices] - self.data[timestamp, index_pip_indices])/index3_ratio + self.rescaled_data[timestamp, index_pip_indices]
            self.rescaled_data[timestamp, middle_dip_indices] = (self.data[timestamp, middle_dip_indices] - self.data[timestamp, middle_pip_indices])/middle3_ratio + self.rescaled_data[timestamp, middle_pip_indices]
            self.rescaled_data[timestamp, ring_dip_indices] = (self.data[timestamp, ring_dip_indices] - self.data[timestamp, ring_pip_indices])/ring3_ratio + self.rescaled_data[timestamp, ring_pip_indices]
            self.rescaled_data[timestamp, pinky_dip_indices] = (self.data[timestamp, pinky_dip_indices] - self.data[timestamp, pinky_pip_indices])/pinky3_ratio + self.rescaled_data[timestamp, pinky_pip_indices]
            self.rescaled_data[timestamp, thumb_ip_indices] = (self.data[timestamp, thumb_ip_indices] - self.data[timestamp, thumb_mcp_indices])/thumb3_ratio + self.rescaled_data[timestamp, thumb_mcp_indices]

            # Fourth bones rescaling
            index4_ratio = np.linalg.norm(self.data[timestamp, index_tip_indices] - self.data[timestamp, index_dip_indices])/index4
            middle4_ratio = np.linalg.norm(self.data[timestamp, middle_tip_indices] - self.data[timestamp, middle_dip_indices])/middle4
            ring4_ratio = np.linalg.norm(self.data[timestamp, ring_tip_indices] - self.data[timestamp, ring_dip_indices])/ring4
            pinky4_ratio = np.linalg.norm(self.data[timestamp, pinky_tip_indices] - self.data[timestamp, pinky_dip_indices])/pinky4
            thumb4_ratio = np.linalg.norm(self.data[timestamp, thumb_tip_indices] - self.data[timestamp, thumb_ip_indices])/thumb4
            
            self.rescaled_data[timestamp, index_tip_indices] = (self.data[timestamp, index_tip_indices] - self.data[timestamp, index_dip_indices])/index4_ratio + self.rescaled_data[timestamp, index_dip_indices]
            self.rescaled_data[timestamp, middle_tip_indices] = (self.data[timestamp, middle_tip_indices] - self.data[timestamp, middle_dip_indices])/middle4_ratio + self.rescaled_data[timestamp, middle_dip_indices]
            self.rescaled_data[timestamp, ring_tip_indices] = (self.data[timestamp, ring_tip_indices] - self.data[timestamp, ring_dip_indices])/ring4_ratio + self.rescaled_data[timestamp, ring_dip_indices]
            self.rescaled_data[timestamp, pinky_tip_indices] = (self.data[timestamp, pinky_tip_indices] - self.data[timestamp, pinky_dip_indices])/pinky4_ratio + self.rescaled_data[timestamp, pinky_dip_indices]
            self.rescaled_data[timestamp, thumb_tip_indices] = (self.data[timestamp, thumb_tip_indices] - self.data[timestamp, thumb_ip_indices])/thumb4_ratio + self.rescaled_data[timestamp, thumb_ip_indices]

        self.data = self.rescaled_data 
        
    def kabsch(self, model_set, data_set_list = -1):

        if isinstance(data_set_list, int):
            data_set_list = ["INDEX_MCP",
                            "RING_MCP",
                            "INDEX_PIP",
                            "INDEX_DIP",
                            "MIDDLE_MCP",
                            "MIDDLE_PIP",
                            "MIDDLE_DIP",
                            "RING_PIP",
                            "RING_DIP",
                            "PINKY_MCP",
                            "PINKY_PIP",
                            "PINKY_DIP",
                            "THUMB_CMC",
                            "THUMB_MCP",
                            "THUMB_IP",
                            "WRIST",
                            "INDEX_TIP",
                            "MIDDLE_TIP",
                            "RING_TIP",
                            "PINKY_TIP",
                            "THUMB_TIP"]
        
        model_set_list = []
        indices = []
        
        for marker in data_set_list:
            marker_id = self.get_marker_id(marker)
            indices += self.get_xyz_indices(marker_id)
            model_set_list.append(model_set[marker])
                
        wrist_id       = self.get_marker_id("WRIST")
        wrist_indices  = self.get_xyz_indices(wrist_id)
        model_set_list = np.asarray(model_set_list)
            
        data_set = self.data[0, indices]
        data_set = np.reshape(data_set, model_set_list.shape)
        
        data_set -= self.data[0, wrist_indices]
        model_set_list -= np.asarray(model_set["WRIST"])
        
        rotation, error = R.align_vectors(model_set_list, data_set)
        matrix = rotation.as_matrix()
        
        self.transformed_data = np.ones(self.data.shape)
        
        for timestamp in range(0, self.data.shape[0]):
            for marker_id in range(1, self.num_markers + 1):
                self.transformed_data[timestamp, self.get_xyz_indices(marker_id)] = np.matmul(matrix, self.data[timestamp,  self.get_xyz_indices(marker_id)])
                
        self.transformed_data[:, :2] = self.data[:, :2]
        self.data = self.transformed_data
        
    
    def angle_rot(self, angle, axis = 'x'):

        if axis == 'x':
            R = np.asarray([[1, 0, 0],
                            [0, math.cos(angle), math.sin(angle)],
                            [0, -math.sin(angle), math.cos(angle)]])
        elif axis == 'z':
            R = np.asarray([[math.cos(angle), math.sin(angle), 0],
                            [-math.sin(angle), math.cos(angle), 0],
                            [0, 0, 1]])
        elif axis == 'y':
            R = np.asarray([[math.cos(angle), 0, math.sin(angle)],
                            [0, 1, 0],
                            [-math.sin(angle), 0, math.cos(angle)]])

        self.transformed_data = np.ones(self.data.shape)
        
        for timestamp in range(0, self.data.shape[0]):
            for marker_id in range(1, self.num_markers + 1):
                self.transformed_data[timestamp, self.get_xyz_indices(marker_id)] = np.matmul(R, self.data[timestamp,  self.get_xyz_indices(marker_id)]) 
                
        self.transformed_data[:, :2] = self.data[:, :2]
        self.data = self.transformed_data

    def apply_filter(self, window, polyorder):
        for i in range(0, self.data.shape[1]):
             self.data[:, i] = savgol_filter(self.data[:, i], window, polyorder)
             
             
    def preprocess_with_preset_and_save(self, savepath, data_dict):
        self.apply_filter(17, 3)
        self.rescale(data_dict)

        self.kabsch(data_dict)
        self.define_origin_marker("WRIST", data_dict['WRIST'][0], data_dict['WRIST'][1], data_dict['WRIST'][2]) 
        self.save_file(savepath)
        
def get_marker_locations_from_model(model_path):
    import opensim
    model = opensim.Model(model_path)

    state = model.initSystem()
    l     = []
    names = []
    data_dict = {}
    for marker in model.getMarkerSet():
        names.append(marker.getName())
        l.append(list(marker.getLocationInGround(state).to_numpy()))
        data_dict[marker.getName()] = list(marker.getLocationInGround(state).to_numpy())
        
    return data_dict


# Define paths
RAW_TRC_PATH = "./examples/interhand/raw_trc/**/*.trc"
MODEL_PATH = "./models/modified/hand_model_nomuscles.osim"
PROCESSED_TRC_PATH = "./examples/interhand/processed_trc/"

# Load marker locations from the model
data_dict = get_marker_locations_from_model(MODEL_PATH)


def process_trc_file(file_path):
    """Process a single TRC file and save the preprocessed output."""
    save_path = file_path.replace("raw_trc", "processed_trc")
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    trc_file = TRCFile(file_path)

    try:
        trc_file.preprocess_with_preset_and_save(save_path, data_dict)
        print(f"Processed: {file_path} → {save_path}")
    except Exception as e:
        print(f"Error processing {file_path}: {e}")


def main():
    """Find and process all TRC files."""
    files = glob.glob(RAW_TRC_PATH, recursive=True)

    for file_path in files:
        process_trc_file(file_path)


if __name__ == "__main__":
    main()