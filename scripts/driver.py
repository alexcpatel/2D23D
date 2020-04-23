import argparse
import os
import shutil

import points
import icp
import triangulation
import verify

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", dest="mode", required=True, help="<mode (f: full pipeline, p: generate pcds in temp folder, t: start with pcds in directory)>")
    parser.add_argument("-v", "--verbose", dest="verbose", action="store_true", help="verbose mode")
    parser.add_argument("-d", "--display", dest="display", action="store_true", help="display mode")
    parser.add_argument("-s", "--input_directory", dest="main_directory", required=True, help="<input_directory>")
    parser.add_argument("-o", "--output_filename", dest="out_filename", required=False, help="[output_filename (otherwise <scan_directory>.pcd)]")

    # point cloud arguments
    parser.add_argument("-l", "--laser_threshold", dest="laser_threshold", required=False, help="[laser_threshold]")
    parser.add_argument("-w", "--window_len", dest="window_len", required=False, help="[window_len]")
    parser.add_argument("-p", "--pixel_skip", dest="pixel_skip", required=False, help="[pixel_skip]")
    parser.add_argument("-f", "--image_skip", dest="image_skip", required=False, help="[image_skip]")

    # verification arguments
    parser.add_argument("-t", "--ground_truth_filename",  dest="truth_filename", required=False, help="[ground_truth_filename (perform verification)]")
    parser.add_argument("-k", "--num_kd_tree_neighbors", dest="num_kd_tree_neighbors", required=False, help="[num_kd_tree_neighbors]")
    parser.add_argument("-i", "--num_ipc_points", dest="num_ipc_points", required=False, help="[num_ipc_points]")

    args = parser.parse_args()
    main_directory            = args.main_directory

    TEMP_DIR                  = "temp"
    DEFAULT_OUT_FILENAME      = os.path.join(args.main_directory + ".obj")

    DEFAULT_LASER_THRESHOLD   = 95  # arbitrary, but found by testing
    DEFAULT_WINDOW_LEN        = 5   # MUST BE ODD!
    DEFAULT_PIXEL_SKIP        = 1
    DEFAULT_IMAGE_SKIP        = 1

    DEFAULT_NUM_ICP_POINTS    = 200000
    DEFAULT_KD_TREE_NEIGHBORS = 5

    mode                    = args.mode
    if mode != 'f' and mode != 'p' and mode != 't':
        print("Mode must be one of: [f, p, t]")
        exit(-1)

    laser_threshold         = int(args.laser_threshold)       if args.laser_threshold       else DEFAULT_LASER_THRESHOLD
    window_len              = int(args.window_len)            if args.window_len            else DEFAULT_WINDOW_LEN
    pixel_skip              = int(args.pixel_skip)            if args.pixel_skip            else DEFAULT_PIXEL_SKIP
    image_skip              = int(args.image_skip)            if args.image_skip            else DEFAULT_IMAGE_SKIP

    out_filename            = args.out_filename               if args.out_filename          else DEFAULT_OUT_FILENAME

    truth_filename          = args.truth_filename  
    num_ipc_points          = int(args.num_ipc_points)        if args.num_ipc_points        else DEFAULT_NUM_ICP_POINTS
    num_kd_tree_neighbors   = int(args.num_kd_tree_neighbors) if args.num_kd_tree_neighbors else DEFAULT_KD_TREE_NEIGHBORS
    
    verbose                 = args.verbose
    display                 = args.display

    # create temporary directory
    if os.path.exists(TEMP_DIR):
        shutil.rmtree(TEMP_DIR)
    os.makedirs(TEMP_DIR)

    # parses directory and create point clouds
    # temporarily stores as pcd files under temp directory
    if mode == 'f' or mode == 'p':
        scan_dirs = os.listdir(main_directory)
        for scan_dir in scan_dirs:
            pcd_out_filename = os.path.join(TEMP_DIR, os.path.basename(scan_dir) + ".pcd")
            points.run_points(os.path.join(main_directory, scan_dir), pcd_out_filename,
                              laser_threshold, window_len, pixel_skip, image_skip, verbose, display)

        if mode == 'p':
            print("All point clouds generated in %s" % TEMP_DIR)
            exit(0)

    # perform icps on all pcd files under temp directory
    pcd_dir = TEMP_DIR if mode == 'f' else main_directory
    pcd_files = os.listdir(pcd_dir)
    merge_pcd = os.path.join(TEMP_DIR, "__merge__.pcd")

    # copy pcd 0 into merge_pcd
    source = o3d.io.read_point_cloud(os.path.join(pcd_dir, pcd_files[0]))
    o3d.io.write_point_cloud(merge_pcd, source)

    # merge each other pcd with it
    for i in range(1, len(pcd_files)):
        source_pcd = os.path.join(pcd_dir, pcd_files[i])
        (source, target, transformation) = icp.run_icp(source_pcd, merge_pcd, verbose, display)
        icp.output_registration_result(source, target, merge_pcd, transformation, display)
    
    # perform triangulation
    triangulation.run_triangulation(merge_pcd, out_filename, verbose, display)

    # perform verification
    if truth_filename:
        verify.run_verify(out_filename, truth_filename, num_ipc_points, num_kd_tree_neighbors, verbose, display)

if __name__ == "__main__":
    main()
