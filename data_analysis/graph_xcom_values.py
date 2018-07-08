#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import sys

# xCOM file format : xCOMReal, xCOMPred, xCOMDiff
xCOMFilename = sys.argv[1]

# Read xCOM file
xCOMMatrix = [x.split() for x in open(xCOMFilename).readlines()]

# Parse columns
xCOMReal = [float(item[0]) for item in xCOMMatrix]
xCOMPred = [float(item[1]) for item in xCOMMatrix]
xCOMDiff = [float(item[2]) for item in xCOMMatrix]

x = list(range(1, len(xCOMReal) + 1))

# Plot xCOMReal and xCOM Pred
figxCOMRP = plt.figure()
figxCOMRP.suptitle("X_CoM Real & Pred")

axxCOMRP = figxCOMRP.add_subplot(111)
axxCOMRP.scatter(x, xCOMPred, label='Pred', s=2)
axxCOMRP.scatter(x, xCOMReal, label='Real', s=2)

axxCOMRP.set_xlabel('Number of Poses')
axxCOMRP.set_ylabel('X_CoM')

axxCOMRP.legend();

# Plot xCOMDiff
figxCOMDiff = plt.figure()
figxCOMDiff.suptitle("X_CoM (Real - Pred)")

axxCOMDiff = figxCOMDiff.add_subplot(111)
axxCOMDiff.scatter(x,xCOMDiff, label='Diff', s=2)
axxCOMDiff.scatter(x,[0]*len(x), label='Zero', s=2)

axxCOMDiff.set_xlabel('Number of Poses')
axxCOMDiff.set_ylabel('X_CoM Diff')

axxCOMDiff.legend();

plt.show()
