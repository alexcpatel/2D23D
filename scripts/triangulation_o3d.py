import argparse
import open3d as o3d
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", action="store_true", help="debugging mode")
    parser.add_argument(
        "-f", "--filename", dest="filename", required=True, help="<filename>.pcd"
    )

    args = parser.parse_args()
    filename = args.filename
    DEBUG = args.debug

    if DEBUG:
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    pcd = o3d.io.read_point_cloud(filename + ".pcd")

    # max_nn: max number of nearest neighbor
    # radius: radius of search
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # orient the normals inwards
    pcd.orient_normals_towards_camera_location([0, 0, 0.5])
    if DEBUG:
        print(pcd)
        o3d.visualization.draw_geometries([pcd])

    # poisson reconstruction
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=8, linear_fit=True
    )
    mesh.compute_triangle_normals()

    if DEBUG:
        o3d.visualization.draw_geometries([mesh])

    # store file
    o3d.io.write_triangle_mesh(filename + ".stl", mesh)


if __name__ == "__main__":
    main()
