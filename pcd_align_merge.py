import numpy as np
import open3d as o3d
import copy
import argparse
import time

def remove_outliers(pcd, nb_points=1000, std_ratio=1.0):
    """Remove statistical outliers from point cloud with even less aggressive parameters"""
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_points, std_ratio=std_ratio)
    return pcd.select_by_index(ind)

def estimate_normals(pcd, radius=0.1):
    """Estimate normals for point cloud"""
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100))
    return pcd

def global_registration(source, target, voxel_size):
    """Perform global registration using RANSAC for initial alignment"""
    source_with_normals = estimate_normals(source, radius=voxel_size * 5)
    target_with_normals = estimate_normals(target, radius=voxel_size * 5)
    
    distance_threshold = voxel_size * 1.5
    try:
        reg_p2p = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_with_normals, target_with_normals, 
            o3d.pipelines.registration.compute_fpfh_feature(source_with_normals, 
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)),
            o3d.pipelines.registration.compute_fpfh_feature(target_with_normals, 
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)),
            True, distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.999))  # Increased iterations
        return reg_p2p.transformation
    except Exception as e:
        print(f"Global registration failed: {str(e)}")
        return np.identity(4)

def icp_registration(source, target, threshold, initial_transform):
    """Perform point-to-plane ICP registration with outlier rejection"""
    source_normals = estimate_normals(source)
    target_normals = estimate_normals(target)
    
    try:
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_normals, target_normals, threshold, initial_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, 
                                                              relative_rmse=1e-6, 
                                                              max_iteration=2000))
        return reg_p2p.transformation
    except Exception as e:
        print(f"ICP registration failed: {str(e)}")
        return initial_transform

def apply_transformation(source, transformation):
    """Apply transformation to source point cloud"""
    source_transformed = copy.deepcopy(source)
    source_transformed.transform(transformation)
    return source_transformed

def merge_point_clouds(source, target, transformation):
    """Merge two point clouds after applying transformation"""
    source_transformed = apply_transformation(source, transformation)
    merged_pcd = source_transformed + target
    return merged_pcd

def main(source_path, target_path, output_path, voxel_size, threshold_factor, visualize_final=False, skip_outliers=False):
    # Set seeds for reproducibility
    np.random.seed(42)  # NumPy seed
    o3d.utility.random.seed(42)  # Open3D seed for RANSAC determinism
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)  # Reduce logging noise
    
    start_time = time.time()
    
    try:
        source_pcd = o3d.io.read_point_cloud(source_path)
        target_pcd = o3d.io.read_point_cloud(target_path)
    except Exception as e:
        print(f"Error loading PCD files: {str(e)}")
        return
    
    print(f"Source points: {len(source_pcd.points)}")
    print(f"Target points: {len(target_pcd.points)}")
    
    # Conditionally skip outlier removal
    if not skip_outliers:
        print("Removing outliers...")
        source_pcd = remove_outliers(source_pcd)
        target_pcd = remove_outliers(target_pcd)
        print(f"Source points after outlier removal: {len(source_pcd.points)}")
        print(f"Target points after outlier removal: {len(target_pcd.points)}")
    else:
        print("Skipping outlier removal...")
    
    # Downsample point clouds with smaller voxel_size for more detail
    distance_threshold = voxel_size * threshold_factor
    print(f"Downsampling with voxel_size={voxel_size}...")
    source_down = source_pcd.voxel_down_sample(voxel_size)
    target_down = target_pcd.voxel_down_sample(voxel_size)
    print(f"Downsampled source points: {len(source_down.points)}")
    print(f"Downsampled target points: {len(target_down.points)}")
    
    # Global registration for initial alignment
    print("Performing global registration for initial alignment...")
    initial_transform = global_registration(source_down, target_down, voxel_size)
    print("Initial transformation matrix:")
    print(initial_transform)
    
    # Perform ICP registration
    print(f"Running point-to-plane ICP with distance_threshold={distance_threshold}...")
    transformation = icp_registration(source_down, target_down, distance_threshold, initial_transform)
    
    print("Final transformation matrix:")
    print(transformation)
    
    # Evaluate alignment quality
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source_down, target_down, distance_threshold, transformation)
    print(f"Alignment quality - Fitness: {evaluation.fitness:.4f}, RMSE: {evaluation.inlier_rmse:.4f}")
    
    # Merge point clouds without downsampling for visualization
    merged_pcd = merge_point_clouds(source_pcd, target_pcd, transformation)
    print(f"Merged points before downsampling: {len(merged_pcd.points)}")
    
    # Downsample merged result for output file
    merged_pcd_output = merged_pcd.voxel_down_sample(voxel_size)
    print(f"Merged points after downsampling for output: {len(merged_pcd_output.points)}")
    
    # Save merged point cloud
    try:
        o3d.io.write_point_cloud(output_path, merged_pcd_output)
        print(f"Merged point cloud saved as: {output_path}")
    except Exception as e:
        print(f"Error saving output PCD: {str(e)}")
        return
    
    # Optionally visualize final merged result with color coding for clarity
    if visualize_final:
        print("Visualizing final merged point cloud with color coding for clarity...")
        transformed_source = apply_transformation(source_pcd, transformation)
        transformed_source.paint_uniform_color([1, 0.706, 0])  # Yellow
        target_pcd.paint_uniform_color([0, 0.651, 0.929])  # Blue
        o3d.visualization.draw_geometries([transformed_source, target_pcd])
    
    end_time = time.time()
    print(f"Total execution time: {end_time - start_time:.2f} seconds")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge two PCD files using advanced ICP alignment with improved clarity, outputting only transformation matrices and saving the merged PCD")
    parser.add_argument("--source", required=True, help="Path to source PCD file")
    parser.add_argument("--target", required=True, help="Path to target PCD file")
    parser.add_argument("--output", required=True, help="Path for output merged PCD file")
    parser.add_argument("--voxel_size", type=float, default=0.025, 
                        help="Voxel size for downsampling (default: 0.025)")
    parser.add_argument("--threshold_factor", type=float, default=1.5, 
                        help="Multiplier for distance threshold (default: 1.5)")
    parser.add_argument("--visualize_final", action="store_true", 
                        help="Enable visualization of the final merged point cloud with color coding (default: False)")
    parser.add_argument("--skip_outliers", action="store_true", 
                        help="Skip outlier removal to retain more points (default: False)")
    
    args = parser.parse_args()
    
    try:
        main(args.source, args.target, args.output, args.voxel_size, args.threshold_factor, 
             args.visualize_final, args.skip_outliers)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
