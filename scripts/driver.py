import argparse
import os
import shutil

import points
import icp
import triangulation
import verify

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", dest="verbose", action="store_true", help="verbose mode")
    parser.add_argument("-s", "--scan_directory", dest="main_directory", required=True, help="<scan_directory>")
    parser.add_argument("-o", "--output_filename", dest="out_filename", required=False, help="<output_filename>")

    # point cloud arguments
    parser.add_argument("-l", "--laser_threshold", dest="laser_threshold", required=False, help="[laser_threshold]")
    parser.add_argument("-w", "--window_len", dest="window_len", required=False, help="[window_len]")
    parser.add_argument("-p", "--pixel_skip", dest="pixel_skip", required=False, help="[pixel_skip]")

    # verification arguments
    parser.add_argument("-t", "--ground_truth_filename",  dest="truth_filename", required=False, help="[ground_truth_filename (perform verification)]")
    parser.add_argument("-k", "--num_kd_tree_neighbors", dest="num_kd_tree_neighbors", required=False, help="[num_kd_tree_neighbors]")
    parser.add_argument("-i", "--num_ipc_points", dest="num_ipc_points", required=False, help="[num_ipc_points]")

    args = parser.parse_args()
    main_directory            = args.main_directory

    TEMP_DIR                  = "temp"
    DEFAULT_OUT_FILENAME      = os.path.join(args.main_directory, ".obj")

    DEFAULT_LASER_THRESHOLD   = 100 # arbitrary
    DEFAULT_WINDOW_LEN        = 5   # MUST BE ODD!
    DEFAULT_PIXEL_SKIP        = 2

    DEFAULT_NUM_ICP_POINTS    = 200000
    DEFAULT_KD_TREE_NEIGHBORS = 10

    laser_threshold         = args.laser_threshold       if args.laser_threshold       else DEFAULT_LASER_THRESHOLD
    window_len              = args.window_len            if args.window_len            else DEFAULT_WINDOW_LEN
    pixel_skip              = args.pixel_skip            if args.pixel_skip            else DEFAULT_PIXEL_SKIP
    out_filename            = args.out_filename          if args.out_filename          else DEFAULT_OUT_FILENAME

    truth_filename          = args.truth_filename  
    num_ipc_points          = args.num_ipc_points        if args.num_ipc_points        else DEFAULT_NUM_ICP_POINTS
    num_kd_tree_neighbors   = args.num_kd_tree_neighbors if args.num_kd_tree_neighbors else DEFAULT_KD_TREE_NEIGHBORS
    
    verbose                 = args.verbose

    # parses directory and create point clouds
    # temporarily stores as pcd files under temp directory
    if os.path.exists(TEMP_DIR):
        shutil.rmtree(TEMP_DIR)
    os.makedirs(TEMP_DIR)
        
    scan_dirs = os.listdir(main_directory)
    for scan_dir in scan_dirs:
        out_filename = os.path.join(TEMP_DIR, scan_dir, ".pcd")
        run_points(scan_dir, out_filename, laser_threshold, window_len, pixel_skip, verbose)

    # perform icps on all pcd files under temp directory
    # source_pcd becomes the icp'ed point cloud
    pcd_files = os.listdir(TEMP_DIR)
    source_pcd = pcd_files[0]
    for i in range(1, len(pcd_files)):
        dest_pcd = pcd_files[i]
        (source, target, transformation) = run_icp(source_pcd, dest_pcd, verbose)
        output_registration_result(source, target, source_pcd, transformation)
    
    # perform triangulation
    run_triangluation(source_pcd, out_filename, verbose)

    # perform verification
    if truth_filename:
        run_verify(out_filename, truth_filename, num_ipc_points, num_kd_tree_neighbors, verbose)

if __name__ == "__main__":
    main()
