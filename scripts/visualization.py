import open3d as o3d
import sys

if __name__ == "__main__":
    pcd_file = sys.argv[1]
    pcd = o3d.io.read_point_cloud(pcd_file)
    o3d.visualization.draw_geometries([pcd], width=1280, height=720)
