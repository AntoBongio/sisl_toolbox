#!/usr/bin/python3



import sys
import time
import pandas as pd

dataPolygon = pd.read_csv('polygon.txt', sep='\s+', header=None)
dataPolygon = pd.DataFrame(dataPolygon)

xPolygon = dataPolygon[0]
yPolygon = dataPolygon[1]
zPolygon = dataPolygon[2]

dataRectangle = pd.read_csv('rectangle.txt', sep='\s+', header=None)
dataRectangle = pd.DataFrame(dataRectangle)

xRectangle = dataRectangle[0]
yRectangle = dataRectangle[1]
zRectangle = dataRectangle[2]

dataParallelLines = pd.read_csv('parallelStraightLines.txt', sep='\s+', header=None)
dataParallelLines = pd.DataFrame(dataParallelLines)

xTestLine = dataParallelLines[0]
yTestLine = dataParallelLines[1]
zTestLine = dataParallelLines[2]

dataIntersectionPoints = pd.read_csv('intersectionPoints.txt', sep='\s+', header=None)
dataIntersectionPoints = pd.DataFrame(dataIntersectionPoints)

xIntersectionPoints = dataIntersectionPoints[0]
yIntersectionPoints = dataIntersectionPoints[1]
zIntersectionPoints = dataIntersectionPoints[2]

import matplotlib.pyplot as plt
figure = plt.figure()
figure.set_figwidth(10)
figure.set_figheight(12)
ax = plt.axes(projection='3d')
plt.title('Path')
plt.xlabel("x")
plt.ylabel("y")

ax.plot3D(xPolygon, yPolygon, zPolygon, 'red', label='Polygon')
ax.plot3D(xRectangle, yRectangle, zRectangle, 'blue', label='Rectangle')
ax.plot3D(xTestLine, yTestLine, zTestLine, 'green', label='Test Line')

for i in range(len(xIntersectionPoints)):
    ax.scatter3D(xIntersectionPoints[i], yIntersectionPoints[i], zIntersectionPoints[i], 'yellow')

ax.legend()

plt.show()