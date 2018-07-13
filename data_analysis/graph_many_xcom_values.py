#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import sys

# xCOM file format : xCOMMatrix {Row: Iteration, Col: xCOM for each Beta
xCOMFilename = sys.argv[1]

# Read xCOM file
xCOMMatrix = [x.split() for x in open(xCOMFilename).readlines()]

# Parse columns
xCOMMatrixLastRow = xCOMMatrix[-1]

#xCOM = list(map(float, xCOMMatrix))
xCOM = [[float(item) for item in itemList] for itemList in xCOMMatrix]
#xCOM = [float(item) for item in xCOMMatrixLastRow]
#print(xCOM)
xCOM = np.array(xCOM)
xCOM = np.absolute(xCOM)

#xCOMReal = [float(item[0]) for item in xCOMMatrix]
#xCOMPred = [float(item[1]) for item in xCOMMatrix]
#xCOMDiff = [float(item[2]) for item in xCOMMatrix]

print(len(xCOM))
x = list(range(1, len(xCOM) + 1))
#x = list([1])

xCOMAvg = [np.mean(item, axis=0) for item in xCOM];
std = [np.std(item, axis=0) for item in xCOM];
xCOMAvgPStd = np.add(xCOMAvg, std);
xCOMAvgMStd = np.subtract(xCOMAvg, std);

xCOMAvgP2Std = np.add(xCOMAvgPStd, std);
xCOMAvgM2Std = np.subtract(xCOMAvgMStd, std);

#xCOMAvg = np.mean(xCOM);
#std = np.std(xCOM);
#xCOMAvgPStd = xCOMAvg + std;
#xCOMAvgMStd = xCOMAvg - std;

# Plot xCOM Avg +/- std and Zero
fig = plt.figure()
fig.suptitle("X_CoM Differences for Many Betas during Learning")

ax = fig.add_subplot(111)
dotsize = 10
ax.scatter(x, xCOMAvgPStd, label='Avg+1*std', s=dotsize)
ax.scatter(x, xCOMAvg, label='Avg', s=dotsize)
ax.scatter(x, xCOMAvgMStd, label='Avg-1*std', s=dotsize)
ax.scatter(x, [0]*len(x), label='Zero', s=dotsize)
ax.scatter(x, [0.002]*len(x), label='2milli', s=dotsize)
#ax.scatter(x, xCOMAvgP2Std, label='Avg+2*std', s=dotsize)
#ax.scatter(x, xCOMAvgM2Std, label='Avg-2*std', s=dotsize)

ax.set_xlabel('Number of Poses')
ax.set_ylabel('X_CoM')

ax.legend();

plt.show()
