# To install pyvista
# pip3 install pyvista
# or if you use Anaconda and other stuff https://pypi.org/project/pyvista/

import pyvista as pv
import numpy as np
from pyvista import examples

# Load data and generate numpy array
mesh = examples.load_airplane()
# Can try other example objects: load_globe(), load_sphere(), etc
points = mesh.points

# Convert to support VTK
cloud = pv.PolyData(points)
cloud.plot()

# To play around with parameters https://docs.pyvista.org/core/filters.html
# Generally alpha = 50-100 works pretty well
volume = cloud.delaunay_3d(alpha=50)
shell = volume.extract_geometry()

# this one gets color and features
#shell = volume.extract_surface()

shell.plot()