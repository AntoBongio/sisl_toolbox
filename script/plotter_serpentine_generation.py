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

dataIntersections = pd.read_csv('intersectionPoints.txt', sep='\s+', header=None)
dataIntersections = pd.DataFrame(dataIntersections)

xIntersections = dataIntersections[0]
yIntersections = dataIntersections[1]
zIntersections = dataIntersections[2]

dataSerpentine = pd.read_csv('serpentine.txt', sep='\s+', header=None)
dataSerpentine = pd.DataFrame(dataSerpentine)

xSerpentine = dataSerpentine[0]
ySerpentine = dataSerpentine[1]
zSerpentine = dataSerpentine[2]


import matplotlib.pyplot as plt
figure = plt.figure()
figure.set_figwidth(10)
figure.set_figheight(12)
ax = plt.axes(projection='3d')
plt.title('Path')
plt.xlabel("x")
plt.ylabel("y")
ax.set_xlim([-200, 200])
ax.set_ylim([-200, 200])
ax.set_zlim([-10, 10])


ax.plot3D(xPolygon, yPolygon, zPolygon, 'red', label='Polygon')
ax.plot3D(xRectangle, yRectangle, zRectangle, 'blue', label='Rectangle')
ax.plot3D(xSerpentine, ySerpentine, zSerpentine, 'black', label='Serpentine')
"""
for i in range(8):
    path = "line" + str(i) + ".txt"
    dataLine = pd.read_csv(path, sep='\s+', header=None)
    dataLine = pd.DataFrame(dataLine)

    xLine = dataLine[0]
    yLine = dataLine[1]
    zLine = dataLine[2]

    ax.plot([xLine[0], xLine[len(xLine) - 1]], [yLine[0], yLine[len(yLine) - 1]], [zLine[0], zLine[len(zLine) - 1]], 'green')
"""
for i in range(13):
    ax.scatter3D(xIntersections[i], yIntersections[i], zIntersections[i], 'purple', label=str(i))


"""
dataLine = pd.read_csv("/home/antonino/Desktop/sisl_toolbox/script/line45Greater.txt", sep='\s+', header=None)
dataLine = pd.DataFrame(dataLine)
xLine = dataLine[0]
yLine = dataLine[1]
zLine = dataLine[2]
ax.plot([xLine[0], xLine[len(xLine) - 1]], [yLine[0], yLine[len(yLine) - 1]], [zLine[0], zLine[len(zLine) - 1]],  label='line45')

dataLine = pd.read_csv("/home/antonino/Desktop/sisl_toolbox/script/line135Greater.txt", sep='\s+', header=None)
dataLine = pd.DataFrame(dataLine)
xLine = dataLine[0]
yLine = dataLine[1]
zLine = dataLine[2]
ax.plot([xLine[0], xLine[len(xLine) - 1]], [yLine[0], yLine[len(yLine) - 1]], [zLine[0], zLine[len(zLine) - 1]], label='line135')

dataLine = pd.read_csv("/home/antonino/Desktop/sisl_toolbox/script/line225Greater.txt", sep='\s+', header=None)
dataLine = pd.DataFrame(dataLine)
xLine = dataLine[0]
yLine = dataLine[1]
zLine = dataLine[2]
ax.plot([xLine[0], xLine[len(xLine) - 1]], [yLine[0], yLine[len(yLine) - 1]], [zLine[0], zLine[len(zLine) - 1]], label='line225')

dataLine = pd.read_csv("/home/antonino/Desktop/sisl_toolbox/script/line315Greater.txt", sep='\s+', header=None)
dataLine = pd.DataFrame(dataLine)
xLine = dataLine[0]
yLine = dataLine[1]
zLine = dataLine[2]
ax.plot([xLine[0], xLine[len(xLine) - 1]], [yLine[0], yLine[len(yLine) - 1]], [zLine[0], zLine[len(zLine) - 1]], label='line315')
"""
ax.legend()

plt.show()