#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import sys

xCOMfilename = sys.argv[1]

# Read xCOM
with open(xCOMfilename) as xCOMF:
    xCOMData = xCOMF.read()

xCOMData = xCOMData.split('\n')

xCOM = xCOMData
# xCOM = [row.split(' ')[0] for row in xCOMData]
x = list(range(1, len(xCOM)))

xCOM.pop()

xCOM = [float(i) for i in xCOM]

# Plot xCOM
figxCOM = plt.figure()
figxCOM.suptitle("X_CoM")

axxCOM = figxCOM.add_subplot(111)
axxCOM.plot(x,xCOM)
axxCOM.plot(x,[0]*len(x))

axxCOM.set_xlabel('Number of Poses')
axxCOM.set_ylabel('X_CoM')

plt.show()
