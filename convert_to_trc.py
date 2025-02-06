# -*- coding: utf-8 -*-
"""
Script to process raw InterHand dataset text files and convert them into TRC format.

Created on Thu Dec 16 01:08:12 2021
@author: clayt
"""

import os
import glob

# Base directories
BASE_PATH = "./examples/interhand/raw_txt/"
BASE_SAVE_PATH = "./examples/interhand/trc/"
DATA_IN_MM = True  # Convert mm to meters

# Define marker names (fixed order)
MARKER_NAMES = [
    "THUMB_TIP", "THUMB_IP", "THUMB_MCP", "THUMB_CMC",
    "INDEX_TIP", "INDEX_DIP", "INDEX_PIP", "INDEX_MCP",
    "MIDDLE_TIP", "MIDDLE_DIP", "MIDDLE_PIP", "MIDDLE_MCP",
    "RING_TIP", "RING_DIP", "RING_PIP", "RING_MCP",
    "PINKY_TIP", "PINKY_DIP", "PINKY_PIP", "PINKY_MCP",
    "WRIST"
]

def get_input_files(base_path):
    """Retrieve all .txt files from subdirectories."""
    return glob.glob(os.path.join(base_path, "*/*.txt"))

def parse_file(file_path):
    """Read and parse the content of a file."""
    with open(file_path, "r") as file:
        lines = file.readlines()
    return [line.strip() for line in lines]

def generate_headers(file_name, num_frames):
    """Generate TRC file headers."""
    header1 = f"PathFileType\t4\t(X/Y/Z)\t{file_name[:-4]}.trc"
    header2 = "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames"
    header3 = f"30.0\t30.0\t{num_frames}\t{len(MARKER_NAMES)}\tm\t30.0\t1\t{num_frames}"
    header4 = "Frame#\tTime\t" + "\t".join(MARKER_NAMES)
    header5 = "\t".join([f"{axis}{i+1}" for i in range(len(MARKER_NAMES)) for axis in ["X", "Y", "Z"]])
    
    return [header1, header2, header3, header4, header5]

def process_lines(lines):
    """Process lines and format coordinate values."""
    processed_lines = []
    
    for line in lines[1:]:  # Skip header
        values = line.replace(" ", ",").split(",")
        frame_data = values[:2]  # Frame number and time
        coordinates = [val for val in values[2:] if val]  # Remove empty values
        
        if DATA_IN_MM:
            coordinates = [str(float(coord) / 1000) for coord in coordinates]  # Convert mm to meters
        
        processed_lines.append("\t".join(frame_data + coordinates))
    
    return processed_lines

def construct_save_path(file_path):
    """Construct and clean the save path for the TRC file."""
    save_path = file_path.replace(BASE_PATH, BASE_SAVE_PATH).replace(".txt", ".trc")
    save_path = save_path.replace("\\", "/").replace("_skeletonopensim", "").replace("raw_txt", "raw_trc")
    
    save_dir = os.path.dirname(save_path)
    os.makedirs(save_dir, exist_ok=True)
    
    return save_path

def convert_to_trc(file_path):
    """Convert a single .txt file to TRC format."""
    lines = parse_file(file_path)
    if len(lines) < 2:
        return  # Skip empty or incomplete files
    
    num_frames = int(lines[-1].split(",")[0]) + 1
    headers = generate_headers(os.path.basename(file_path), num_frames)
    processed_data = process_lines(lines)
    
    save_path = construct_save_path(file_path)
    with open(save_path, "w") as fw:
        fw.write("\n".join(headers) + "\n")
        fw.write("\n".join(processed_data) + "\n")

def main():
    files = get_input_files(BASE_PATH)
    
    for file_path in files:
        convert_to_trc(file_path)
        print(f'Processed: {file_path}')

if __name__ == "__main__":
    main()