import open3d as o3d

pcd = o3d.io.read_point_cloud("test.pcd")

points = \
  [[ 0.556 ,-1.6  , -4.08 ],
[-1.356 , 3.902, -1.562],
[ 1.356, -3.902 , 1.562],
[-0.556 , 1.6,    4.08 ]]
lines = [ [0, 1], [1, 3], [2, 3], [0, 2]]
colors = [[1, 0, 0] for i in range(len(lines))]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(colors)

base_points = \
  [[ -10, -10, 0],
   [ -10,  10, 0],
   [  10, -10, 0],
   [  10,  10, 0]]
base_lines = [[0, 1], [1, 3], [2, 3], [0, 2]]
base_colors = [[0, 0, 1] for i in range(len(lines))]
base_line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(base_points),
    lines=o3d.utility.Vector2iVector(base_lines),
)
base_line_set.colors = o3d.utility.Vector3dVector(base_colors)

o3d.visualization.draw_geometries([pcd, line_set, base_line_set])
