"""
This script visualizes obstacles, lattice points, and edges in a 3D space using Matplotlib and Pandas. It reads data from CSV files, filters it based on specified region bounds, and plots the data in a 3D plot.
"""

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Define the region bounds
xmin, xmax = 0, 100
ymin, ymax = 0, 100
zmin, zmax = 0, 100

# Load the data
obstacles_df = pd.read_csv('../data/obstacles_output.csv')
lattice_points_df = pd.read_csv('../data/lattice_points_output.csv')
edges_df = pd.read_csv('../data/edges_output.csv')

# Function to create a 3D box
def create_box(min_x, min_y, min_z, max_x, max_y, max_z):
    """
    Creates a 3D box given the minimum and maximum coordinates along each axis.

    Parameters:
    min_x (float): Minimum x-coordinate of the box.
    min_y (float): Minimum y-coordinate of the box.
    min_z (float): Minimum z-coordinate of the box.
    max_x (float): Maximum x-coordinate of the box.
    max_y (float): Maximum y-coordinate of the box.
    max_z (float): Maximum z-coordinate of the box.

    Returns:
    list: A list of faces where each face is represented by a list of vertices.
    """
    vertices = [
        [min_x, min_y, min_z],
        [max_x, min_y, min_z],
        [max_x, max_y, min_z],
        [min_x, max_y, min_z],
        [min_x, min_y, max_z],
        [max_x, min_y, max_z],
        [max_x, max_y, max_z],
        [min_x, max_y, max_z]
    ]
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[4], vertices[7], vertices[3], vertices[0]]
    ]
    return faces

# Filter the data to include only points and edges within the specified region
filtered_obstacles_df = obstacles_df[
    (obstacles_df['minX'] >= xmin) & (obstacles_df['maxX'] <= xmax) &
    (obstacles_df['minY'] >= ymin) & (obstacles_df['maxY'] <= ymax) &
    (obstacles_df['minZ'] >= zmin) & (obstacles_df['maxZ'] <= zmax)
]

filtered_lattice_points_df = lattice_points_df[
    (lattice_points_df['x'] >= xmin) & (lattice_points_df['x'] <= xmax) &
    (lattice_points_df['y'] >= ymin) & (lattice_points_df['y'] <= ymax) &
    (lattice_points_df['z'] >= zmin) & (lattice_points_df['z'] <= zmax)
]

filtered_edges_df = edges_df[
    (edges_df['x1'] >= xmin) & (edges_df['x1'] <= xmax) &
    (edges_df['y1'] >= ymin) & (edges_df['y1'] <= ymax) &
    (edges_df['z1'] >= zmin) & (edges_df['z1'] <= zmax) &
    (edges_df['x2'] >= xmin) & (edges_df['x2'] <= xmax) &
    (edges_df['y2'] >= ymin) & (edges_df['y2'] <= ymax) &
    (edges_df['z2'] >= zmin) & (edges_df['z2'] <= zmax)
]

# Plotting the data
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot lattice points
ax.scatter(filtered_lattice_points_df['x'], filtered_lattice_points_df['y'], filtered_lattice_points_df['z'], c='b', marker='o')

# Plot obstacles
for index, row in filtered_obstacles_df.iterrows():
    faces = create_box(row['minX'], row['minY'], row['minZ'], row['maxX'], row['maxY'], row['maxZ'])
    poly3d = Poly3DCollection(faces, alpha=.25, linewidths=1, edgecolors='r')
    poly3d.set_facecolor('c')
    ax.add_collection3d(poly3d)

# Plot edges
for index, row in filtered_edges_df.iterrows():
    ax.plot([row['x1'], row['x2']], [row['y1'], row['y2']], [row['z1'], row['z2']], 'k-', linewidth=0.5)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim([xmin, xmax])
ax.set_ylim([ymin, ymax])
ax.set_zlim([zmin, zmax])

plt.show()
