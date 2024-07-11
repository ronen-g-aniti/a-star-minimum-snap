"""
This script visualizes the path found by the A* algorithm in 3D space. The obstacles, lattice points, edges, and path are plotted in 3D space.
The data is read from CSV files and filtered based on the bounding box of the path. The obstacles are shown as red boxes, lattice points as 
blue points, edges as green lines, and the path as a red line.
"""

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Load the data
obstacles_df = pd.read_csv('../data/obstacles_output.csv')
lattice_points_df = pd.read_csv('../data/lattice_points_output.csv')
edges_df = pd.read_csv('../data/edges_output.csv')
path_df = pd.read_csv('../data/path_output.csv')

# Function to create a 3D box
def create_box(min_x, min_y, min_z, max_x, max_y, max_z):
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

# Determine the bounding box for the path
if not path_df.empty:
    xmin, ymin, zmin = path_df[['x', 'y', 'z']].min()
    xmax, ymax, zmax = path_df[['x', 'y', 'z']].max()
else:
    xmin, ymin, zmin = 0, 0, 0
    xmax, ymax, zmax = 100, 100, 100

# Add a margin to the bounding box
margin = 10
xmin, ymin, zmin = xmin - margin, ymin - margin, zmin - margin
xmax, ymax, zmax = xmax + margin, ymax + margin, zmax + margin

# Plot the data
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot obstacles
for _, row in obstacles_df.iterrows():
    if row['minX'] >= xmin and row['maxX'] <= xmax and row['minY'] >= ymin and row['maxY'] <= ymax and row['minZ'] >= zmin and row['maxZ'] <= zmax:
        faces = create_box(row['minX'], row['minY'], row['minZ'], row['maxX'], row['maxY'], row['maxZ'])
        poly3d = Poly3DCollection(faces, alpha=0.5, edgecolors='r')
        ax.add_collection3d(poly3d)

# Plot lattice points
lattice_points_filtered = lattice_points_df[(lattice_points_df['x'] >= xmin) & (lattice_points_df['x'] <= xmax) &
                                            (lattice_points_df['y'] >= ymin) & (lattice_points_df['y'] <= ymax) &
                                            (lattice_points_df['z'] >= zmin) & (lattice_points_df['z'] <= zmax)]
ax.scatter(lattice_points_filtered['x'], lattice_points_filtered['y'], lattice_points_filtered['z'], c='b', marker='o', s=10, label='Lattice Points')

# Plot edges
edges_filtered = edges_df[(edges_df['x1'] >= xmin) & (edges_df['x1'] <= xmax) & (edges_df['x2'] >= xmin) & (edges_df['x2'] <= xmax) &
                          (edges_df['y1'] >= ymin) & (edges_df['y1'] <= ymax) & (edges_df['y2'] >= ymin) & (edges_df['y2'] <= ymax) &
                          (edges_df['z1'] >= zmin) & (edges_df['z1'] <= zmax) & (edges_df['z2'] >= zmin) & (edges_df['z2'] <= zmax)]
for _, row in edges_filtered.iterrows():
    x = [row['x1'], row['x2']]
    y = [row['y1'], row['y2']]
    z = [row['z1'], row['z2']]
    ax.plot(x, y, z, c='g', alpha=0.5)

# Plot path
if not path_df.empty:
    ax.plot(path_df['x'], path_df['y'], path_df['z'], c='r', linewidth=2, label='Path')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Path Visualization')

# Set limits
ax.set_xlim([xmin, xmax])
ax.set_ylim([ymin, ymax])
ax.set_zlim([zmin, zmax])

plt.legend()
plt.show()

