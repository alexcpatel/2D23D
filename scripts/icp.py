import open3d as o3d
import numpy as np
import copy
import sys

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size, source_pcd, dest_pcd):
    # print(":: Load two point clouds and disturb initial pose.")
    print(":: Load two points clouds and display")
    source = o3d.io.read_point_cloud(source_pcd)
    target = o3d.io.read_point_cloud(dest_pcd)
    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                          [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
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

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python icp.py <pcd_1> <pcd_2> <output_pcd>")
        exit(-1)
    
    source_pcd = sys.argv[1]
    dest_pcd = sys.argv[2]
    output_pcd = sys.argv[3]

    voxel_size = 0.05  # 0.05 means 5cm for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
            prepare_dataset(voxel_size, source_pcd, dest_pcd)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    draw_registration_result(source_down, target_down,
                             result_ransac.transformation)

    result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                     voxel_size)
    print(result_icp)
    draw_registration_result(source, target, result_icp.transformation)

    output_registration_result(source, target, output_pcd, result_icp.transformation)

    # source = o3d.io.read_point_cloud("cloud_bin_0.pcd")
    # target = o3d.io.read_point_cloud("cloud_bin_1.pcd")

    # o3d.visualization.draw_geometries([source])
    # o3d.visualization.draw_geometries([target])

    # threshold = 0.02
    # trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
    #                          [-0.139, 0.967, -0.215, 0.7],
    #                          [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    # trans_init = np.asarray([[0.5, 0.05, -0.5, 0.5],
    #                          [-0.5, 0.5, -0.215, 0.7],
    #                          [0.5, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    # trans_init = np.asarray([[-0.50983203,  0.85920657,  0.04284115,  0.51144388],
    #                             [-0.1963465,  -0.16470324,  0.96660276,  0.81431754],
    #                             [ 0.83756752,  0.48439334,  0.25267319, -1.39610046],
    #                             [ 0.0,          0.0,          0.0,         1.0        ]])
    # draw_registration_result(source, target, trans_init)
    # print("Initial alignment")
    # evaluation = o3d.registration.evaluate_registration(source, target,
    #                                                     threshold, trans_init)
    # print(evaluation)

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