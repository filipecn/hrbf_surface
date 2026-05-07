import matplotlib.pyplot as plt
import numpy as np
import argparse
import sys

sys.path.insert(0, "C:\\dev\\hrbf_surface\\build\\Release_Cuda\\lib\\Debug")
sys.path.insert(0, "/home/filipecn/dev/hrbf_surface/build/Release_Cuda/lib")
import hrbf_surface_py

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ply", default="")
    parser.add_argument("--sphere", action="store_true")
    parser.add_argument("--square", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--partition-size", type=float, default=0.05)
    parser.add_argument("--partition-overlap", type=float, default=0.5)
    parser.add_argument("--voxel-size", type=float, default=0.1)
    parser.add_argument("--nth-partition", type=int, default=-1)
    parser.add_argument("--single-partition", action="store_true")
    args = parser.parse_args()

    print(args)

    if args.sphere:
        pcl = hrbf_surface_py.sphere(1.0, 15)
    elif args.square:
        pcl = hrbf_surface_py.square(1.0, 100)
    else:
        pcl = hrbf_surface_py.load_point_cloud(args.ply)

    # if args.single_partition:
    #     surface = hrbf_surface_py.reconstruct_single(pcl)
    # else:
    #     surface = hrbf_surface_py.reconstruct(pcl, args.partition_size)

    # if args.nth_partition >= 0:
    #     mesh = hrbf_surface_py.partition_mesh(surface, args.voxel_size, args.single_partition)
    # else:
    #     mesh = hrbf_surface_py.mesh(surface, args.voxel_size)

    single_partition = False
    if args.single_partition:
        single_partition = True

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    if args.debug:
        # Plot points
        ax.scatter(
            pcl.positions[:, 0],
            pcl.positions[:, 1],
            pcl.positions[:, 2],
            color="b",
            s=20,
        )

        # Plot normals as arrows (quiver)
        ax.quiver(
            pcl.positions[:, 0],
            pcl.positions[:, 1],
            pcl.positions[:, 2],
            pcl.normals[:, 0],
            pcl.normals[:, 1],
            pcl.normals[:, 2],
            length=0.1,
            color="r",
        )

    for i in range(0):
        mesh = hrbf_surface_py.reconstruct_surface(
            pcl,
            args.partition_size,
            args.partition_overlap,
            args.voxel_size,
            single_partition,
            i,
        )
        # Plot points
        ax.scatter(mesh.pv[:, 0], mesh.pv[:, 1], mesh.pv[:, 2], color="b", s=20)
        edges = [
            (0, 1),
            (1, 5),
            (5, 4),
            (4, 0),  # Bottom square
            (2, 3),
            (3, 7),
            (7, 6),
            (6, 2),  # Top square
            (0, 2),
            (1, 3),
            (5, 7),
            (4, 6),  # Vertical pillars
        ]

        for start_idx, end_idx in edges:
            # Get the x, y, z coordinates for the start and end points
            x = [mesh.bounds[start_idx, 0], mesh.bounds[end_idx, 0]]
            y = [mesh.bounds[start_idx, 1], mesh.bounds[end_idx, 1]]
            z = [mesh.bounds[start_idx, 2], mesh.bounds[end_idx, 2]]

            ax.plot3D(x, y, z, color="green", linewidth=2)
        # if len(mesh.positions):
        #     ax.plot_trisurf(mesh.positions[:,0], mesh.positions[:,1], mesh.positions[:,2], triangles=mesh.faces, cmap='Spectral')

    mesh = hrbf_surface_py.reconstruct_surface(
        pcl,
        args.partition_size,
        args.partition_overlap,
        args.voxel_size,
        single_partition,
        -1,
    )
    if len(mesh.positions):
        ax.plot_trisurf(
            mesh.positions[:, 0],
            mesh.positions[:, 1],
            mesh.positions[:, 2],
            triangles=mesh.faces,
            cmap="Spectral",
        )

    plt.show()
