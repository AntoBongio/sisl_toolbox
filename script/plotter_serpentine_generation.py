#!/usr/bin/python3



import sys
import time
import pandas as pd
data = pd.read_csv('path.txt', sep='\s+', header=None)
data = pd.DataFrame(data)

dataPolygon = pd.read_csv('polygon.txt', sep='\s+', header=None)
dataPolygon = pd.DataFrame(dataPolygon)

xPolygon = dataPolygon[1]
yPolygon = dataPolygon[0]
zPolygon = -dataPolygon[2]

dataRectangle = pd.read_csv('rectangle.txt', sep='\s+', header=None)
dataRectangle = pd.DataFrame(dataRectangle)

xRectangle = dataRectangle[1]
yRectangle = dataRectangle[0]
zRectangle = -dataRectangle[2]


points = pd.read_csv('intersectionPoints.txt', sep='\s+', header=None)
points = pd.DataFrame(points)
xPoints = points[1]
yPoints = points[0]
zPoints = -points[2]



"""
startPoint = pd.read_csv('startPoint.txt', sep='\s+', header=None)
startPoint = pd.DataFrame(startPoint)



import matplotlib.pyplot as plt
figure = plt.figure()
figure.set_figwidth(10)
figure.set_figheight(12)
ax = plt.axes(projection='3d',  adjustable='box')
plt.title('Path')
plt.xlabel("y")
plt.ylabel("x")

x = data[1]
y = data[0]
z = -data[2]

ax.plot3D(x, y, z, 'blue', label='Path')
#ax.scatter3D(x[0], y[0], z[0], 'yellow', label='Start point')
#ax.scatter3D(x.iloc[-1], y.iloc[-1], z.iloc[-1], 'red', label='Last point')
ax.plot3D(xPolygon, yPolygon, zPolygon, 'red', label='Polygon')
ax.plot3D(xRectangle, yRectangle, zRectangle, 'blue', label='Rectangle')

#ax.scatter3D(startPoint[1], startPoint[0], -startPoint[2], 'yellow', label='Start point')

for i in range(7):
    dataLine = pd.read_csv('line' + str(i) + '.txt', sep='\s+', header=None)
    dataLine = pd.DataFrame(dataLine)
    xLine = dataLine[1]
    yLine = dataLine[0]
    zLine = -dataLine[2]
    ax.plot3D(xLine, yLine, zLine, 'red', label='Line')

for i in range(len(xPoints)):
    ax.scatter3D(xPoints[i], yPoints[i], zPoints[i], 'blue', label=str(i))


ax.legend()

plt.show()
"""

import matplotlib.pyplot as plt
figure = plt.figure()
figure.set_figwidth(10)
figure.set_figheight(12)
ax = plt.axes(adjustable='box')
plt.title('Path')
plt.xlabel("y")
plt.ylabel("x")

x = data[1]
y = data[0]

ax.plot(x, y, 'blue', label='Path')
#ax.scatter(x[0], y[0], 'yellow', label='Start point')
#ax.scatter(x.iloc[-1], y.iloc[-1], 'red', label='Last point')
ax.plot(xPolygon, yPolygon, 'red', label='Polygon')
ax.plot(xRectangle, yRectangle, 'blue', label='Rectangle')

#ax.scatter3D(startPoint[1], startPoint[0], -startPoint[2], 'yellow', label='Start point')

for i in range(8):
    dataLine = pd.read_csv('line' + str(i) + '.txt', sep='\s+', header=None)
    dataLine = pd.DataFrame(dataLine)
    xLine = dataLine[1]
    yLine = dataLine[0]
    ax.plot(xLine, yLine, 'red', label='Line')

for i in range(len(xPoints)):
    ax.scatter(xPoints[i], yPoints[i], alpha=1.0, edgecolors='none', label=str(i))


ax.legend()

plt.xlim([-200, 200])
plt.ylim([-200, 200])
plt.show()