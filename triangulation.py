# To install pyvista
# pip3 install pyvista
# or if you use Anaconda and other stuff https://pypi.org/project/pyvista/

import pyvista as pv
import open3d as o3d
import numpy as np

FILENAMES = ['cube', 'vase', 'monkey']

for filename in FILENAMES:
  pcd = o3d.io.read_point_cloud(filename + '.pcd')
  points = np.asarray(pcd.points)
  cloud = pv.PolyData(points)
  volume = cloud.delaunay_3d(tol=0.5)
  shell = volume.extract_geometry()
  shell.save(filename + '.stl')