#!/usr/bin/python3



import sys
import time
import pandas as pd
data = pd.read_csv('path.txt', sep='\s+', header=None)
data = pd.DataFrame(data)

if len(sys.argv) > 1:
    if sys.argv[1] == "findClosest" :
        findNearProblem = pd.read_csv('closestPoint.txt', sep='\s+', header=None)
        findNearProblem = pd.DataFrame(findNearProblem)
        findNearPoint = [findNearProblem[2][0], findNearProblem[1][0], -findNearProblem[3][0]]
        closestPoint = [findNearProblem[2][1], findNearProblem[1][1], -findNearProblem[3][1]]

    if sys.argv[1] == "movePoint" :
        movePointProblem = pd.read_csv('movePoint.txt', sep='\s+', header=None)
        movePointProblem = pd.DataFrame(movePointProblem)
        points = []
        for index in range(len(movePointProblem)):
            points.append([movePointProblem[2][index], movePointProblem[1][index], -movePointProblem[3][index], movePointProblem[0][index]])

    if sys.argv[1] == "extractSection" :
        dataSection = pd.read_csv('pathSection.txt', sep='\s+', header=None)
        dataSection = pd.DataFrame(dataSection)


import matplotlib.pyplot as plt
figure = plt.figure()
figure.set_figwidth(10)
figure.set_figheight(12)
ax = plt.axes(projection='3d')
plt.title('Path')
plt.xlabel("y")
plt.ylabel("x")

x = data[1]
y = data[0]
z = -data[2]
ax.plot3D(x, y, z, 'blue', label='Path')

ax.scatter3D(x[0], y[0], z[0], 'yellow', label='Start point')

ax.scatter3D(x.iloc[-1], y.iloc[-1], z.iloc[-1], 'red', label='Last point')

if len(sys.argv) > 1:
    if sys.argv[1] == "findClosest" :
        ax.scatter3D(findNearPoint[0], findNearPoint[1], findNearPoint[2], 'purple', label='Find near point')
        ax.scatter3D(closestPoint[0], closestPoint[1], closestPoint[2], 'green', label='Closest Point')
        ax.plot([findNearPoint[0], closestPoint[0]], [findNearPoint[1], closestPoint[1]], [findNearPoint[2], closestPoint[2]], c ='r')


    if sys.argv[1] == "movePoint" :
        for elem in points:
            ax.scatter3D(elem[0], elem[1], elem[2], 'black', label=str(elem[3]) )
            ax.legend()

    if sys.argv[1] == "extractSection" :
        xSection = dataSection[0]
        ySection = dataSection[1]
        zSection = dataSection[2]
        ax.plot3D(xSection, ySection, zSection, 'red', label='Path section')
        ax.scatter3D(xSection[0], ySection[0], zSection[0], 'red', label='Section start point')
        ax.scatter3D(xSection.iloc[-1], ySection.iloc[-1], zSection.iloc[-1], 'red', label='Section last point')

ax.legend()

plt.show()
