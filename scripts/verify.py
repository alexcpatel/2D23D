import argparse
import open3d as o3d
import numpy as np
import copy
import icp

from sklearn.neighbors import KDTree 
from numpy import dot
from math import sqrt

# code adapted from generous Gwolyn Fischer on github to perform
# point-triangle distance queries
def point_triangle_distance(TRI, P):
    # function [dist,PP0] = pointTriangleDistance(TRI,P)
    # calculate distance between a point and a triangle in 3D
    # SYNTAX
    #   dist = pointTriangleDistance(TRI,P)
    #   [dist,PP0] = pointTriangleDistance(TRI,P)
    #
    # DESCRIPTION
    #   Calculate the distance of a given point P from a triangle TRI.
    #   Point P is a row vector of the form 1x3. The triangle is a matrix
    #   formed by three rows of points TRI = [P1;P2;P3] each of size 1x3.
    #   dist = pointTriangleDistance(TRI,P) returns the distance of the point P
    #   to the triangle TRI.
    #   [dist,PP0] = pointTriangleDistance(TRI,P) additionally returns the
    #   closest point PP0 to P on the triangle TRI.
    #
    # Author: Gwolyn Fischer
    # Release: 1.0
    # Release date: 09/02/02
    # Release: 1.1 Fixed Bug because of normalization
    # Release: 1.2 Fixed Bug because of typo in region 5 20101013
    # Release: 1.3 Fixed Bug because of typo in region 2 20101014

    # Possible extention could be a version tailored not to return the distance
    # and additionally the closest point, but instead return only the closest
    # point. Could lead to a small speed gain.

    # Example:
    # %% The Problem
    # P0 = [0.5 -0.3 0.5]
    #
    # P1 = [0 -1 0]
    # P2 = [1  0 0]
    # P3 = [0  0 0]
    #
    # vertices = [P1; P2; P3]
    # faces = [1 2 3]
    #
    # %% The Engine
    # [dist,PP0] = pointTriangleDistance([P1;P2;P3],P0)
    #
    # %% Visualization
    # [x,y,z] = sphere(20)
    # x = dist*x+P0(1)
    # y = dist*y+P0(2)
    # z = dist*z+P0(3)
    #
    # figure
    # hold all
    # patch('Vertices',vertices,'Faces',faces,'FaceColor','r','FaceAlpha',0.8)
    # plot3(P0(1),P0(2),P0(3),'b*')
    # plot3(PP0(1),PP0(2),PP0(3),'*g')
    # surf(x,y,z,'FaceColor','b','FaceAlpha',0.3)
    # view(3)

    # The algorithm is based on
    # "David Eberly, 'Distance Between Point and Triangle in 3D',
    # Geometric Tools, LLC, (1999)"
    # http:\\www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
    #
    #        ^t
    #  \     |
    #   \reg2|
    #    \   |
    #     \  |
    #      \ |
    #       \|
    #        *P2
    #        |\
    #        | \
    #  reg3  |  \ reg1
    #        |   \
    #        |reg0\
    #        |     \
    #        |      \ P1
    # -------*-------*------->s
    #        |P0      \
    #  reg4  | reg5    \ reg6
    # rewrite triangle in normal form
    B = TRI[0, :]
    E0 = TRI[1, :] - B
    # E0 = E0/sqrt(sum(E0.^2)); %normalize vector
    E1 = TRI[2, :] - B
    # E1 = E1/sqrt(sum(E1.^2)); %normalize vector
    D = B - P
    a = dot(E0, E0)
    b = dot(E0, E1)
    c = dot(E1, E1)
    d = dot(E0, D)
    e = dot(E1, D)
    f = dot(D, D)

    #print "{0} {1} {2} ".format(B,E1,E0)
    det = a * c - b * b
    s = b * e - c * d
    t = b * d - a * e

    # Terible tree of conditionals to determine in which region of the diagram
    # shown above the projection of the point into the triangle-plane lies.
    if (s + t) <= det:
        if s < 0.0:
            if t < 0.0:
                # region4
                if d < 0:
                    t = 0.0
                    if -d >= a:
                        s = 1.0
                        sqrdistance = a + 2.0 * d + f
                    else:
                        s = -d / a
                        sqrdistance = d * s + f
                else:
                    s = 0.0
                    if e >= 0.0:
                        t = 0.0
                        sqrdistance = f
                    else:
                        if -e >= c:
                            t = 1.0
                            sqrdistance = c + 2.0 * e + f
                        else:
                            t = -e / c
                            sqrdistance = e * t + f

                            # of region 4
            else:
                # region 3
                s = 0
                if e >= 0:
                    t = 0
                    sqrdistance = f
                else:
                    if -e >= c:
                        t = 1
                        sqrdistance = c + 2.0 * e + f
                    else:
                        t = -e / c
                        sqrdistance = e * t + f
                        # of region 3
        else:
            if t < 0:
                # region 5
                t = 0
                if d >= 0:
                    s = 0
                    sqrdistance = f
                else:
                    if -d >= a:
                        s = 1
                        sqrdistance = a + 2.0 * d + f;  # GF 20101013 fixed typo d*s ->2*d
                    else:
                        s = -d / a
                        sqrdistance = d * s + f
            else:
                # region 0
                invDet = 1.0 / det
                s = s * invDet
                t = t * invDet
                sqrdistance = s * (a * s + b * t + 2.0 * d) + t * (b * s + c * t + 2.0 * e) + f
    else:
        if s < 0.0:
            # region 2
            tmp0 = b + d
            tmp1 = c + e
            if tmp1 > tmp0:  # minimum on edge s+t=1
                numer = tmp1 - tmp0
                denom = a - 2.0 * b + c
                if numer >= denom:
                    s = 1.0
                    t = 0.0
                    sqrdistance = a + 2.0 * d + f;  # GF 20101014 fixed typo 2*b -> 2*d
                else:
                    s = numer / denom
                    t = 1 - s
                    sqrdistance = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f

            else:  # minimum on edge s=0
                s = 0.0
                if tmp1 <= 0.0:
                    t = 1
                    sqrdistance = c + 2.0 * e + f
                else:
                    if e >= 0.0:
                        t = 0.0
                        sqrdistance = f
                    else:
                        t = -e / c
                        sqrdistance = e * t + f
                        # of region 2
        else:
            if t < 0.0:
                # region6
                tmp0 = b + e
                tmp1 = a + d
                if tmp1 > tmp0:
                    numer = tmp1 - tmp0
                    denom = a - 2.0 * b + c
                    if numer >= denom:
                        t = 1.0
                        s = 0
                        sqrdistance = c + 2.0 * e + f
                    else:
                        t = numer / denom
                        s = 1 - t
                        sqrdistance = s * (a * s + b * t + 2.0 * d) + t * (b * s + c * t + 2.0 * e) + f

                else:
                    t = 0.0
                    if tmp1 <= 0.0:
                        s = 1
                        sqrdistance = a + 2.0 * d + f
                    else:
                        if d >= 0.0:
                            s = 0.0
                            sqrdistance = f
                        else:
                            s = -d / a
                            sqrdistance = d * s + f
            else:
                # region 1
                numer = c + e - b - d
                if numer <= 0:
                    s = 0.0
                    t = 1.0
                    sqrdistance = c + 2.0 * e + f
                else:
                    denom = a - 2.0 * b + c
                    if numer >= denom:
                        s = 1.0
                        t = 0.0
                        sqrdistance = a + 2.0 * d + f
                    else:
                        s = numer / denom
                        t = 1 - s
                        sqrdistance = s * (a * s + b * t + 2.0 * d) + t * (b * s + c * t + 2.0 * e) + f

    # account for numerical round-off error
    if sqrdistance < 0:
        sqrdistance = 0

    dist = sqrt(sqrdistance)

    PP0 = B + s * E0 + t * E1
    return dist, PP0

def distance_to_mesh(two_percent_dist, five_percent_distpoint, point, triangles):
    min_dist = -1
    for triangle in triangles:
        dist, _ = point_triangle_distance(triangle, point)
        if dist < two_percent_dist:
            return dist
        if min_dist < 0 or dist < min_dist:
            min_dist = dist
    return min_dist

def run_verify(scan_filename, truth_filename, num_ipc_points, num_kd_tree_neighbors, verbose, display):
    if verbose:
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    # load meshes
    voxel_size  = 0.05
    source_mesh = o3d.io.read_triangle_mesh(scan_filename)
    target_mesh = o3d.io.read_triangle_mesh(truth_filename)
    source_pcd  = source_mesh.sample_points_uniformly(
        number_of_points=num_ipc_points)
    target_pcd  = target_mesh.sample_points_uniformly(
        number_of_points=num_ipc_points)
    source_down, source_fpfh = icp.preprocess_point_cloud(
        source_pcd, voxel_size, verbose, display)
    target_down, target_fpfh = icp.preprocess_point_cloud(
        target_pcd, voxel_size, verbose, display)

    # find transformation between source and target
    result_global = icp.execute_global_registration(source_down, target_down,
        source_fpfh, target_fpfh, voxel_size, verbose)
    result_local  = icp.execute_local_registration(source_pcd, target_pcd,
        source_fpfh, target_fpfh, voxel_size, result_global, verbose)
    
    # preprocess mesh to correct it
    source_mesh.remove_degenerate_triangles()
    target_mesh.remove_degenerate_triangles()
    source_mesh.remove_duplicated_triangles()
    target_mesh.remove_duplicated_triangles()
    source_mesh.remove_duplicated_vertices()
    target_mesh.remove_duplicated_vertices()
    source_mesh.remove_non_manifold_edges()
    target_mesh.remove_non_manifold_edges()
    source_mesh.remove_unreferenced_vertices()
    target_mesh.remove_unreferenced_vertices()

    # transform source mesh to align with target
    source_mesh.transform(result_local.transformation)

    # visualize aligned meshes
    # o3d.visualization.draw_geometries([source_mesh, target_mesh])

    # extract points and triangles from source and target
    source_points           = np.asarray(source_mesh.vertices)
    target_points           = np.asarray(target_mesh.vertices)
    target_triangle_indices = np.asarray(target_mesh.triangles)
    target_triangles        = target_points[target_triangle_indices]

    # form kd-trees and query to precompute closest triangles
    target_centroids  = copy.deepcopy(target_triangles)
    target_centroids  = np.average(target_triangles, 1)
    target_p0s        = target_triangles[:,0,:]
    target_p1s        = target_triangles[:,1,:]
    target_p2s        = target_triangles[:,2,:]

    centroid_tree     = KDTree(target_centroids)
    _, centroid_ind   = centroid_tree.query(source_points, k=num_kd_tree_neighbors)
    p0_tree           = KDTree(target_p0s)
    _, p0_ind         = p0_tree.query(source_points, k=num_kd_tree_neighbors)
    p1_tree           = KDTree(target_p1s)
    _, p1_ind         = p1_tree.query(source_points, k=num_kd_tree_neighbors)
    p2_tree           = KDTree(target_p2s)
    _, p2_ind         = p2_tree.query(source_points, k=num_kd_tree_neighbors)

    nearest_ind       = np.append(np.append(centroid_ind, p0_ind, axis=1),
                        np.append(p1_ind, p2_ind, axis=1), axis=1)
    nearest_triangles = target_triangles[nearest_ind]

    # ACCURACY REQUIREMENT:
    # 100% of non-occluded points must be within 5% of the longest axis to 
    #   the ground truth model.
    # 90% of non-occluded points must be within 2% of the longest axis to
    #   the ground truth model.

    # determine longest axis cutoffs for ground truth mesh
    target_bbox       = target_mesh.get_axis_aligned_bounding_box()
    extent            = target_bbox.get_extent()
    longest_axis      = sqrt(extent[0]**2 + extent[1]**2 + extent[2]**2)
    two_percent_dist  = 0.02 * longest_axis
    five_percent_dist = 0.05 * longest_axis

    # get the distance from each vertex of the scan to ground truth
    num_points            = source_points.shape[0]
    num_two_percent_dist  = 0
    num_five_percent_dist = 0

    for i in range(num_points):
        if verbose and i % 10000 == 0:
            print("%7d / %7d    \tvertices processed..." % (i, num_points))
        dist = distance_to_mesh(two_percent_dist, five_percent_dist, source_points[i], nearest_triangles[i])
        if dist > two_percent_dist:
            num_two_percent_dist += 1
        if dist > five_percent_dist:
            num_five_percent_dist += 1

    print("num_points: %d, num_two_percent_dist: %d (%f%%), num_five_percent_dist: %d (%f%%)" \
          % (num_points,
             num_two_percent_dist,  (num_two_percent_dist  / num_points) * 100.0,
             num_five_percent_dist, (num_five_percent_dist / num_points) * 100.0))
    
    if num_five_percent_dist > 0:
        print("Scan does not meet accuracy requirement:\n",
              "\t100% of non-occluded points must be within 5% of the longest \
axis to the ground truth model.")
    elif num_two_percent_dist > 0.1 * num_points:
        print("Scan does not meet accuracy requirement:\n",
              "\t90% of non-occluded points must be within 2% of the longest \
axis to the ground truth model.")
    else:
        print("Scan meets accuracy requirements!")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--verbose", action="store_true", help="verbose mode")
    parser.add_argument("-s", "--scan_filename", dest="scan_filename", required=True, help="<scan_filename>")
    parser.add_argument("-t", "--truth_filename",  dest="truth_filename", required=True, help="<truth_filename>")
    parser.add_argument("-k", "--num_kd_tree_neighbors", dest="num_kd_tree_neighbors", required=False, help="[num_kd_tree_neighbors]")
    parser.add_argument("-i", "--num_ipc_points", dest="num_ipc_points", required=False, help="[num_ipc_points]")

    DEFAULT_NUM_ICP_POINTS    = 200000
    DEFAULT_KD_TREE_NEIGHBORS = 10

    args                  = parser.parse_args()
    scan_filename         = args.scan_filename
    truth_filename        = args.truth_filename
    num_ipc_points        = int(args.num_ipc_points) \
                            if args.num_ipc_points != None \
                            else DEFAULT_NUM_ICP_POINTS
    num_kd_tree_neighbors = int(args.num_kd_tree_neighbors) \
                            if args.num_kd_tree_neighbors != None \
                            else DEFAULT_KD_TREE_NEIGHBORS
    verbose               = args.verbose

    run_verify(scan_filename, truth_filename, num_ipc_points, num_kd_tree_neighbors, verbose, False)

if __name__ == "__main__":
    main()
