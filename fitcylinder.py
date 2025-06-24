#  This program is free software; you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation; version 2 or later of the License.      #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
#  GNU General Public License for more details.                          #

import pycc
import cccorelib
import math
import numpy as np
from scipy.linalg import svd

def create_cone(coordinates, cone_deflection):
    # Get CloudCompare instance
    cc = pycc.GetInstance()
    
    # Extract start and end points
    start_point = np.array(coordinates[0])
    end_point = np.array(coordinates[1])
    
    # Calculate direction vector and length
    direction = end_point - start_point
    length = np.linalg.norm(direction)
    direction_normalized = direction / length
    
    # Calculate cone radius at base (using 5 degree angle)
    angle_rad = math.radians(cone_deflection)
    base_radius = length * math.tan(angle_rad)
    
    # Create circle points for base (32 points)
    num_points = 32
    circle_points = []
    
    # Find perpendicular vectors
    if abs(direction_normalized[0]) < abs(direction_normalized[1]):
        perp1 = np.array([1.0, 0.0, 0.0])
    else:
        perp1 = np.array([0.0, 1.0, 0.0])
    
    perp1 = perp1 - np.dot(perp1, direction_normalized) * direction_normalized
    perp1 = perp1 / np.linalg.norm(perp1)
    perp2 = np.cross(direction_normalized, perp1)
    
    # Generate circle points
    for i in range(num_points):
        angle = (2 * math.pi * i) / num_points
        circle_point = end_point + base_radius * (
            math.cos(angle) * perp1 + math.sin(angle) * perp2
        )
        circle_points.append(circle_point)
    
    # Create vertices for the cone
    vertices = pycc.ccPointCloud([start_point[0]], [start_point[1]], [start_point[2]],)
    
    # Add circle points
    for point in circle_points:
        #cp = pycc.ccPointCloud([point[0]], [point[1]], [point[2]])
        vertices.addPoints([point[0]], [point[1]], [point[2]])
    
    # Create mesh
    mesh = pycc.ccMesh(vertices)
    for i in range(num_points):
        # Create triangles connecting apex to base
        mesh.addTriangle(0, i + 1, ((i + 1) % num_points) + 1)
    
    # Set appearance
    #mesh.setColor(pycc.Rgb(255, 0, 0))  # Red color
    #mesh.showColors(True)
    
    # Add to database and update display
    cc.addToDB(mesh)
    cc.updateUI()

def get_best_fit_line():
    # Get CloudCompare instance
    cc = pycc.GetInstance()
    
    # Get selected point cloud
    try:
        cloud = cc.getSelectedEntities()[0]
    except IndexError:
        print("No point cloud selected.")
        return

    # Get points from the cloud
    #points = cloud.getPoints()
    points = np.array(cloud.points())

    # Compute the best fit line using SVD
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    U, S, Vt = svd(centered_points)
    direction = Vt[0]
    t = np.array([-1, 1]) * np.max(np.abs(centered_points))
    line_points = np.outer(t, direction) + centroid
    return line_points


# Example usage:
degrees_deflection = 5  # Cone deflection angle in degrees

# Get the best fit line coordinates
coordinates = get_best_fit_line()

# Create the cone using the best fit line coordinates
create_cone(coordinates, degrees_deflection)