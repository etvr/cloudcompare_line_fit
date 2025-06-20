//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #

import cccorelib
import pycc
import numpy as np
from scipy.linalg import svd

cc = pycc.GetInstance()
try:
    cloud = cc.getSelectedEntities()[0]
except:
    print("No cloud selected!")

points = np.array(cloud.points())

#calculate the best fit line
centroid = np.mean(points, axis=0)
centered_points = points - centroid
U, S, Vt = svd(centered_points)
direction = Vt[0]
t = np.array([-1, 1]) * np.max(np.abs(centered_points))
line_points = np.outer(t, direction) + centroid

#convert verts to a ccPointcloud, formatted as seperate arrays of xyz [[x],[y],[z]]
vertices = pycc.ccPointCloud(line_points[:,0], line_points[:,1], line_points[:,2],)
line = pycc.ccPolyline(vertices)

line.setColor(pycc.Rgb(255, 0, 0)) # set the color to red
line.setWidth(9)
line.showColors(True)
line.setClosed(False)

#important else the line will have 0 vertices
line.addPointIndex(0, 2)
cc.addToDB(line)
cc.updateUI()
