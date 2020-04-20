import argparse
import open3d as o3d
import numpy as np

def run_triangluation(in_filename, out_filename, verbose):
    if verbose: o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    pcd = o3d.io.read_point_cloud(in_filename)

    # max_nn: max number of nearest neighbor
    # radius: radius of search
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # orient the normals inwards
    pcd.orient_normals_towards_camera_location([0, 0, 1])
    # reorient normals
    normals = []
    for normal in pcd.normals:
        normals.append(normal * -1)
    pcd.normals = o3d.utility.Vector3dVector(np.asarray(normals, dtype=np.float64))

    if verbose: o3d.visualization.draw_geometries([pcd])

    # poisson reconstruction
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=8, linear_fit=True)
    mesh.compute_triangle_normals()

    # preprocess mesh to correct it
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()

    # merge close vertices
    eps = np.mean(pcd.compute_nearest_neighbor_distance())
    mesh = mesh.merge_close_vertices(eps)

    if verbose: o3d.visualization.draw_geometries([mesh])

    # store file
    o3d.io.write_triangle_mesh(out_filename, mesh)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true", help="verbose mode")
    parser.add_argument("-f", "--filename", dest="filename", required=True, help="<filename>.pcd")

    args     = parser.parse_args()
    filename = args.filename
    verbose  = args.verbose

    run_triangluation(filename + ".pcd", filename + ".obj", verbose)

if __name__ == "__main__":
    main()
