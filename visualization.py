import open3d as o3d

pcd = o3d.io.read_point_cloud("cube.pcd")
o3d.visualization.draw_geometries([pcd])
pcd = o3d.io.read_point_cloud("vase.pcd")
o3d.visualization.draw_geometries([pcd])
pcd = o3d.io.read_point_cloud("monkey.pcd")
o3d.visualization.draw_geometries([pcd])
