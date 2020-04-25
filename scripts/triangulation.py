import argparse
import open3d as o3d
import numpy as np
import pyvista as pv
import time 

def run_triangulation(in_filename, out_filename, verbose, display):
    if verbose: 
        print("Perfoming screened poisson triangulation")
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

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

    # if display: 
    #     if verbose: print("Displaying input pcd file for triangulation...")
    #     o3d.visualization.draw_geometries([pcd])

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

    if display: o3d.visualization.draw_geometries([mesh])

    # store file
    o3d.io.write_triangle_mesh(out_filename, mesh)

def run_delaunay(in_filename, out_filename, verbose, display, alpha=0.1):
    if verbose:
        print("Running delaunay 3D triangulation")

    pcd = o3d.io.read_point_cloud(in_filename)
    cloud = pv.PolyData(np.asarray(pcd.points))

    if display:
        if verbose: print("Displaying point cloud")
        cloud.plot()
    
    if verbose:
        print("Performing delaunay algorithm")
        
    volume = cloud.delaunay_3d(alpha = alpha)
    shell = volume.extract_geometry()
     
    if display:
        if verbose: print("Displaying mesh ")
        shell.plot()

    if verbose:
        print("Saving output mesh")

    # First saving as STL file since pyvista OBJ file dealing is weird
    stl_name = out_filename.replace(".obj", ".stl")
    shell.save(stl_name)
    stl_mesh = o3d.io.read_triangle_mesh(stl_name)
    o3d.io.write_triangle_mesh(out_filename, stl_mesh)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true", help="verbose mode")
    parser.add_argument("-f", "--filename", dest="filename", required=True, help="<filename>")
    parser.add_argument("-o", "--outfilename", dest="out_filename", required=True, help="<filename>")
    parser.add_argument("-d", "--display", action="store_true", help="display point cloud and mesh")

    triangulation_parser = parser.add_mutually_exclusive_group()
    triangulation_parser.add_argument("-df", "--delaunay_fast", dest="delaunay_fast", action="store_true", required=False, help="fast delaunay triangulation")
    triangulation_parser.add_argument("-ds", "--delaunay_slow", dest="delaunay_slow", action="store_true", required=False, help="slow delaunay triangulation")

    args     = parser.parse_args()
    filename = args.filename
    verbose  = args.verbose
    display = args.display
    out_filename = args.out_filename

    delaunay_fast = args.delaunay_fast
    delaunay_slow = args.delaunay_slow

    if delaunay_fast:
        run_delaunay(filename, out_filename, verbose, display, alpha=0.1)
    elif delaunay_slow:
        run_delaunay(filename, out_filename, verbose, display, alpha=0.05)
    else:
        run_triangulation(filename, out_filename, verbose, display)

if __name__ == "__main__":
    main()
