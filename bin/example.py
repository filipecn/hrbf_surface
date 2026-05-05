import matplotlib.pyplot as plt
import numpy as np
import argparse
import sys

sys.path.insert(0, "C:\\dev\\hrbf_surface\\build\\Release_Cuda\\lib\\Debug")
import hrbf_surface_py

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--ply', default="")
    parser.add_argument('--sphere', action="store_true")
    parser.add_argument('--square', action="store_true")
    parser.add_argument('--debug', action="store_true")
    parser.add_argument('--partition-size', type=float, default=0.05)
    parser.add_argument('--voxel-size', type=float, default=0.1)
    parser.add_argument('--single-partition', type=int, default=-1)
    args = parser.parse_args()

    print(args)

    if args.sphere:
        pcl = hrbf_surface_py.sphere(1.0, 100)
    elif args.square:
        pcl = hrbf_surface_py.square(1.0, 100)
    else:
        pcl = hrbf_surface_py.load(args.ply)
        
    surface = hrbf_surface_py.reconstruct(pcl, args.partition_size)
    
    if args.single_partition >= 0:
        mesh = hrbf_surface_py.partition_mesh(surface, args.voxel_size, args.single_partition)
    else:
        mesh = hrbf_surface_py.mesh(surface, args.voxel_size)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if args.debug:
        # Plot points
        ax.scatter(pcl.positions[:,0], pcl.positions[:,1], pcl.positions[:,2], color='b', s=20)

        # Plot normals as arrows (quiver)
        ax.quiver(pcl.positions[:,0], pcl.positions[:,1], pcl.positions[:,2], 
                  pcl.normals[:,0], pcl.normals[:,1], pcl.normals[:,2], 
                  length=0.1, color='r')


    if len(mesh.positions):
        ax.plot_trisurf(mesh.positions[:,0], mesh.positions[:,1], mesh.positions[:,2], triangles=mesh.faces, cmap='Spectral')


    plt.show()