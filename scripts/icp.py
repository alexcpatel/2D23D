import open3d as o3d
import numpy as np
import copy
import sys
import statistics

def preprocess_point_cloud(pcd, voxel_size):
    print("Downsampling with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print("Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print("Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size, source_pcd, dest_pcd, scale):
    print("Load two points clouds and display")
    source = o3d.io.read_point_cloud(source_pcd)
    target = o3d.io.read_point_cloud(dest_pcd)
    source.scale(scale)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print("Global registration on downsampled point clouds with voxel size is %.3f," % voxel_size)
    print("  and distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500)) #(max iterations, validation steps)
    return result

def execute_local_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_global):
    distance_threshold = voxel_size * 0.2
    print("Local Point-to-point ICP registration with distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_global.transformation,
        o3d.registration.TransformationEstimationPointToPoint())
    return result

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp], width=1280, height=720)

def output_registration_result(source, target, output_file, transformation):
    new_pcd = o3d.geometry.PointCloud()
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)    
    new_pcd += source_temp
    new_pcd += target_temp
    o3d.io.write_point_cloud(output_file, new_pcd)
    o3d.visualization.draw_geometries([new_pcd], width=1280, height=720)

def pcd_distance(source, target):
    return statistics.mean(source.compute_point_cloud_distance(target))

def scale_aligned_pcd(source, target, delta, max_iter):
    scale = 1
    # find initial direction
    direction = 1
    source_1 = copy.deepcopy(source)
    source_2 = copy.deepcopy(source)
    source_1.scale(scale=scale + direction * delta)
    source_2.scale(scale=scale - direction * delta)
    if(pcd_distance(source_1, target) > pcd_distance(source_2, target)):
        direction = -1
    
    source_copy = copy.deepcopy(source)
    prev_dist = pcd_distance(source_copy.scale(scale=scale), target)
    for i in range(max_iter):
        scale += direction * delta
        source_copy = copy.deepcopy(source)
        cur_dist = pcd_distance(source_copy.scale(scale=scale), target)
        # convergence
        if (cur_dist > prev_dist):
            scale -= direction * delta
            break
    
    print("Showing scaled results")
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.scale(scale=scale)
    o3d.visualization.draw_geometries([source_temp, target_temp], width=1280, height=720)
    
    source_copy = copy.deepcopy(source)
    source_copy.scale(scale=scale)
    return (scale, source_copy)


def run_icp(source_pcd, target_pcd): # takes pcd filenames as input
    voxel_size = 0.05  # 0.05 means 5cm for the dataset, note monkey is 2m wide in Blender
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
            prepare_dataset(voxel_size, source_pcd, dest_pcd, 1)

    result_global = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    draw_registration_result(source_down, target_down,
                             result_global.transformation)

    result_local = execute_local_registration(source, target, source_fpfh, target_fpfh,
                                     voxel_size, result_global)

    print("Transformation matrix: ")
    print(result_local.transformation)
    draw_registration_result(source, target, result_local.transformation)

    return (source, target, result_local.transformation)

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python icp.py <pcd_1> <pcd_2> <output_pcd>")
        exit(-1)
    
    source_pcd = sys.argv[1]
    dest_pcd = sys.argv[2]
    output_pcd = sys.argv[3]

    (source, target, transformation) = run_icp(source_pcd, dest_pcd)
    
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    print("Point cloud distance:")
    print(pcd_distance(source_temp, target_temp))
    # source_temp.transform(transformation)
    # (scale, scaled_source) = scale_aligned_pcd(source_temp, target_temp, 0.01, 10000)
    # print("Scale: " + str(scale))

    output_registration_result(source, target, output_pcd, transformation)

    # Scaling pipeline
    # if (scale > 1.05 or scale < 0.95): # iterate until proper scale
    #     while(abs(scale - 1) > 0.05):
    #         voxel_size = 0.05
    #         source, target, source_down, target_down, source_fpfh, target_fpfh = \
    #                 prepare_dataset(voxel_size, source_pcd, dest_pcd, scale)

    #         result_global = execute_global_registration(source_down, target_down,
    #                                                     source_fpfh, target_fpfh,
    #                                                     voxel_size)
    #         draw_registration_result(source_down, target_down,
    #                                 result_global.transformation)

    #         result_local = execute_local_registration(source, target, source_fpfh, target_fpfh,
    #                                         voxel_size, result_global)
    #         print("Transformation matrix: ")
    #         print(result_local.transformation)
    #         draw_registration_result(source, target, result_local.transformation)

    #         source_temp = copy.deepcopy(source)
    #         target_temp = copy.deepcopy(target)
    #         print("Point cloud distance:")
    #         print(pcd_distance(source_temp, target_temp))
    #         source_temp.transform(result_local.transformation)
    #         (scale, scaled_source) = scale_aligned_pcd(source_temp, target_temp, 0.01, 10000)
    #         print("Scale: " + str(scale))


    # print("Apply point-to-point ICP")
    # reg_p2p = o3d.registration.registration_icp(
    #     source, target, threshold, trans_init,
    #     o3d.registration.TransformationEstimationPointToPoint())
    # print(reg_p2p)
    # print("Transformation is:")
    # print(reg_p2p.transformation)
    # print("")
    # draw_registration_result(source, target, reg_p2p.transformation)

    # print("Apply point-to-plane ICP")
    # reg_p2l = o3d.registration.registration_icp(
    #     source, target, threshold, trans_init,
    #     o3d.registration.TransformationEstimationPointToPlane())
    # print(reg_p2l)
    # print("Transformation is:")
    # print(reg_p2l.transformation)
    # print("")
    # draw_registration_result(source, target, reg_p2l.transformation)