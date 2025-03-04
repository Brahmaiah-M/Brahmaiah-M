import numpy as np
import open3d as o3d
import copy
import argparse
import time

def remove_outliers(pcd, nb_points=500, std_ratio=1.0):
    """Remove statistical outliers from point cloud with even less aggressive parameters"""
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_points, std_ratio=std_ratio)
    return pcd.select_by_index(ind)

def apply_transformation(source, transformation):
    """Apply transformation to source point cloud"""
    source_transformed = copy.deepcopy(source)
    source_transformed.transform(transformation)
    return source_transformed

def stitch_point_clouds(source, target, transformation):
    """Stitch two point clouds using the provided transformation"""
    source_transformed = apply_transformation(source, transformation)
    stitched_pcd = source_transformed + target  # Combine without overlap assumption
    return stitched_pcd

def get_transformation_matrix_from_user():
    """Prompt user to input a 4x4 transformation matrix at runtime"""
    print("Enter the 4x4 transformation matrix as 16 floats (row-major order):")
    print("Example: 1 0 0 2 0 1 0 0 0 0 1 0 0 0 0 1 (for 2m X-translation)")
    print("Use this to position the source PCD relative to the target PCD.")
    while True:
        try:
            matrix_input = input("Enter 16 space-separated floats: ")
            matrix_values = [float(x) for x in matrix_input.split()]
            if len(matrix_values) != 16:
                print("Error: You must provide exactly 16 floats for a 4x4 matrix.")
                continue
            transformation_matrix = np.array(matrix_values).reshape(4, 4)
            print("Transformation matrix entered:")
            print(transformation_matrix)
            return transformation_matrix
        except ValueError:
            print("Error: Invalid input. Please enter 16 valid floats separated by spaces.")
        except Exception as e:
            print(f"Error: {str(e)}. Please try again.")

def main(source_path, target_path, output_path, voxel_size, visualize_final=False, skip_outliers=False):
    # Set seeds for reproducibility
    np.random.seed(42)
    o3d.utility.random.seed(42)
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    
    start_time = time.time()
    
    # Load point clouds
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
    
    # Downsample point clouds
    print(f"Downsampling with voxel_size={voxel_size}...")
    source_down = source_pcd.voxel_down_sample(voxel_size)
    target_down = target_pcd.voxel_down_sample(voxel_size)
    print(f"Downsampled source points: {len(source_down.points)}")
    print(f"Downsampled target points: {len(target_down.points)}")
    
    # Get the user-provided transformation matrix at runtime
    transformation = get_transformation_matrix_from_user()
    
    # Stitch point clouds using the user-provided transformation
    print("Stitching point clouds with user-provided transformation...")
    stitched_pcd = stitch_point_clouds(source_pcd, target_pcd, transformation)
    print(f"Stitched points before downsampling: {len(stitched_pcd.points)}")
    
    # Downsample stitched result for output file
    stitched_pcd_output = stitched_pcd.voxel_down_sample(voxel_size)
    print(f"Stitched points after downsampling for output: {len(stitched_pcd_output.points)}")
    
    # Save stitched point cloud
    try:
        o3d.io.write_point_cloud(output_path, stitched_pcd_output)
        print(f"Stitched point cloud saved as: {output_path}")
    except Exception as e:
        print(f"Error saving output PCD: {str(e)}")
        return
    
    # Optionally visualize final stitched result with color coding
    if visualize_final:
        print("Visualizing final stitched point cloud with color coding for clarity...")
        transformed_source = apply_transformation(source_pcd, transformation)
        transformed_source.paint_uniform_color([1, 0.706, 0])  # Yellow
        target_pcd.paint_uniform_color([0, 0.651, 0.929])  # Blue
        o3d.visualization.draw_geometries([transformed_source, target_pcd])
    
    end_time = time.time()
    print(f"Total execution time: {end_time - start_time:.2f} seconds")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stitch two PCD files using a user-provided transformation matrix")
    parser.add_argument("--source", required=True, help="Path to source PCD file")
    parser.add_argument("--target", required=True, help="Path to target PCD file")
    parser.add_argument("--output", required=True, help="Path for output stitched PCD file")
    parser.add_argument("--voxel_size", type=float, default=0.025, 
                        help="Voxel size for downsampling (default: 0.025)")
    parser.add_argument("--visualize_final", action="store_true", 
                        help="Enable visualization of the final stitched point cloud (default: False)")
    parser.add_argument("--skip_outliers", action="store_true", 
                        help="Skip outlier removal to retain more points (default: False)")
    
    args = parser.parse_args()
    
    try:
        main(args.source, args.target, args.output, args.voxel_size, 
             args.visualize_final, args.skip_outliers)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
