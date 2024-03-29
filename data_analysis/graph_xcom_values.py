#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import sys

# xCOM file format : xCOMMatrix {Row: Iteration, Col: xCOM for each Beta
xCOMFilename = sys.argv[1]

# Read xCOM file
xCOMMatrix = [x.split() for x in open(xCOMFilename).readlines()]

xCOM = [[float(item) for item in itemList] for itemList in xCOMMatrix]
xCOM = np.array(xCOM)
xCOMReal = xCOM[:, 0]
# Make xCOMReal a column vector
xCOMReal = xCOMReal[:, np.newaxis]
#xCOM = np.absolute(xCOM)
xCOM = np.delete(xCOM, 0, axis=1)

x = list(range(1, len(xCOM) + 1))

xCOMAvg = [np.mean(item, axis=0) for item in xCOM];
std = [np.std(item, axis=0) for item in xCOM];
xCOMAvgPStd = np.add(xCOMAvg, std);
xCOMAvgMStd = np.subtract(xCOMAvg, std);

xCOMAvgP2Std = np.add(xCOMAvgPStd, std);
xCOMAvgM2Std = np.subtract(xCOMAvgMStd, std);

# Plot xCOM Avg +/- std and Real
fig = plt.figure()
fig.suptitle("X_CoM Values for Many Betas during Testing")

ax = fig.add_subplot(111)
dotsize = 10
ax.scatter(x, xCOMAvgPStd, label='Avg+1*std', s=dotsize)
ax.scatter(x, xCOMAvgMStd, label='Avg-1*std', s=dotsize)
ax.scatter(x, xCOMAvg, label='Avg', s=dotsize)
ax.scatter(x, xCOMReal, label='Real', s=dotsize)
ax.plot(x, [0.002]*len(x), label='2milli')
ax.plot(x, [-0.002]*len(x), label='-2milli')
#ax.scatter(x, xCOMAvgP2Std, label='Avg+2*std', s=dotsize)
#ax.scatter(x, xCOMAvgM2Std, label='Avg-2*std', s=dotsize)

ax.set_xlabel('Number of Poses')
ax.set_ylabel('X_CoM')
ax.legend();

# Plot xCOM Diff Avg +/- std
xCOMDiff = xCOM - xCOMReal
xCOMDiff = np.absolute(xCOMDiff)
xCOMAvgDiff = [np.mean(item, axis=0) for item in xCOMDiff];
stdDiff = [np.std(item, axis=0) for item in xCOMDiff];
xCOMAvgDiffPStd = np.add(xCOMAvgDiff, stdDiff);
xCOMAvgDiffMStd = np.subtract(xCOMAvgDiff, stdDiff);

totalAvgDiff = np.mean(xCOMAvgDiff)
print(totalAvgDiff)

figDiff = plt.figure()
figDiff.suptitle("X_CoM Differences for Many Betas during Testing")

axDiff = figDiff.add_subplot(111)
dotsize = 10
axDiff.scatter(x, xCOMAvgDiffPStd, label='AvgDiff+1*std', s=dotsize)
axDiff.scatter(x, xCOMAvgDiffMStd, label='AvgDiff-1*std', s=dotsize)
axDiff.scatter(x, xCOMAvgDiff, label='AvgDiff', s=dotsize)
axDiff.scatter(x, [0]*len(x), label='Zero', s=1)
axDiff.plot(x, [0.002]*len(x), label='2milli')
axDiff.plot(x, [-0.002]*len(x), label='-2milli')
axDiff.plot(x, [totalAvgDiff]*len(x), label='Total Avg Diff')

axDiff.set_xlabel('Number of Poses')
axDiff.set_ylabel('X_CoM Diff')

axDiff.legend();
plt.show()
