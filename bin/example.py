import matplotlib.pyplot as plt
import numpy as np

import sys

sys.path.insert(0, "C:\\dev\\hrbf_surface\\build\\Release_Cuda\\lib\\Debug")
import hrbf_surface_py

sphere = hrbf_surface_py.pcl_sphere(1.0, 10)
mesh = hrbf_surface_py.reconstruct(sphere, 0.1)


pts = sphere.positions
normals = sphere.normals

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot points
# ax.scatter(pts[:,0], pts[:,1], pts[:,2], color='b', s=20)

# Plot normals as arrows (quiver)
# ax.quiver(pts[:,0], pts[:,1], pts[:,2], 
#           normals[:,0], normals[:,1], normals[:,2], 
#           length=0.1, color='r')

ax.plot_trisurf(mesh.positions[:,0], mesh.positions[:,1], mesh.positions[:,2], triangles=mesh.faces, cmap='Spectral')

plt.show()

