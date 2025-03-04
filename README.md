# LiDAR Processing Tools

This repository contains a set of tools for processing LiDAR data. The tools are written in Python and are designed to handle point cloud data (PCD) and packet capture (PCAP) files. Below are the details for each script, including their purpose and command-line arguments.

## Prerequisites

-List dependencies
--------------------
=>transformation matrix and merging
numpy,open3D,copy,argprase,time

=>stiching
numpy,open3D,copy,argprase,time

=>pcap_t0_pcd
os,argparse,client from Ouster.sdk,PointCloud and Encoding from pypcd4,numpy.



## Files and Usage
----------------------------------

####1.pcap_to_pcd conversion
**File:** `pcap_to_pcd_conversion.py`

**Description:** This script converts LiDAR packet capture (PCAP) files into PCD format for further processing.

**Command-Line Arguments:**
command:-python3 Filename.py /path/to/pcap/.pcap /path/to/json/.json pcap-to-pcd

Example:-python3 pcap_to_pcd_conversion.py /media/cvat-server/lidar_121.pcap /media/cvat-server/lidar_121_0.json pcap-to-pcd



####2.pcd allignment and merging
**File:** 'pcd_align_merge.py'
**Description:** pcd_align_merge.py aligns and merges two PCD files using point-to-plane ICP refinement. It supports outlier removal, voxel downsampling, and visualization for efficient LiDAR/3D scanning applications

**Command-Line Arguments:**
command:- python3 filename.py --source <source_dir>/<source_file> --target <target_dir>/<target_file> --output <output_dir>/<output_file> [--voxel_size <size>] [--threshold_factor <factor>] [--visualize_final] [--skip_outliers]

 flags:
<source_dir>, <target_dir>, <output_dir>: Base directories
<source_file>, <target_file>, <output_file>: Specific PCD filenames

Optional flags:

    --voxel_size <size>: Custom voxel size (default: 0.025)
    --threshold_factor <factor>: Custom threshold multiplier (default: 1.5)
    --visualize_final: Visualize the result
    --skip_outliers: Skip outlier removal
    
Example:
python3 pcd_align_merge.py --source /home/cvat-server/pcd_out_000006.pcd --target /home/cvat-server/pcd_out_000009.pcd --output ~/Brahmaiah/ICP/merged9_1.pcd --skip_outliers



####3.PCD's stitching
**File:** 'pcd_stitching.py'
**Description:**combining multiple point clouds from PCD (Point Cloud Data) files into a single, unified point cloud. This process involves aligning the point clouds using techniques like ICP (Iterative Closest Point) and merging them to create a seamless 3D representation. It is commonly used in LiDAR and 3D scanning applications to reconstruct complete environments or objects from multiple viewpoints

**Command-Line Arguments:**
command:- python3 Filename.py --source <source_pcd_path> --target <target_pcd_path> --output <output_pcd_path> [--voxel_size <size>] [--visualize_final]

Optional flags:
--voxel_size (optional) → Downsampling size for efficiency (e.g., 0.05)
--visualize_final (optional) → If included, it enables visualization of the final stitched point cloud

Example:
python3 snitch5.py --source /home/pcd_out_000021.pcd --target /home/pcd_out_000021.pcd --output merged_360_16.pcd \--voxel_size 0.025 --visualize_final
