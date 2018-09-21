#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from matplotlib import rc
import sys

plt.rcParams['axes.grid'] = True
plt.rc('xtick', labelsize=18)
plt.rc('ytick', labelsize=18)
rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
rc('text', usetex=True)

# xCOM file format : xCOMMatrix {Row: Iteration, Col: xCOM for each Beta
xCOMFilename = sys.argv[1]
totalMassFilename = sys.argv[2]
betaFilename = sys.argv[3]
betaIdealFilename = sys.argv[4]

# Read files
xCOMMatrix = [x.split() for x in open(xCOMFilename).readlines()]
totalMassMatrix = [x.split() for x in open(totalMassFilename).readlines()]
betaMatrix = [x.split() for x in open(betaFilename).readlines()]
betaIdealMatrix = [x.split() for x in open(betaIdealFilename).readlines()]

# Parse columns
xCOM = [[float(item) for item in itemList] for itemList in xCOMMatrix]
xCOM = np.array(xCOM)
xCOMReal = xCOM[:, 0]
# Make xCOMReal a column vector
xCOMReal = xCOMReal[:, np.newaxis]
#xCOM = np.absolute(xCOM)
xCOM = np.delete(xCOM, 0, axis=1)

# The number of x data points
x = list(range(1, len(xCOM) + 1))

totalMass = [[float(item) for item in itemList] for itemList in totalMassMatrix]
totalMass = np.array(totalMass)

beta = [[float(item) for item in itemList] for itemList in betaMatrix]
beta = np.array(beta)

betaIdeal = [[float(item) for item in itemList] for itemList in betaIdealMatrix]
betaIdeal = np.array(betaIdeal)

# Plot xCOM Avg +/- std and Real
# Take absolute value
xCOM = np.absolute(xCOM)
xCOMAvg = [np.mean(item, axis=0) for item in xCOM];
std = [np.std(item, axis=0) for item in xCOM];
xCOMAvgPStd = np.add(xCOMAvg, std);
xCOMAvgMStd = np.subtract(xCOMAvg, std);

figxCOM = plt.figure()
#figxCOM.suptitle("X_CoM for Many Betas during Learning")

axxCOM = figxCOM.add_subplot(111)
# TODO dont need avg in hardware plot
axxCOM.plot(x, xCOMAvg, label='Avg', linewidth=2)
axxCOM.plot(x, xCOMAvgPStd, label='Avg+1*std', linewidth=2)
axxCOM.set_ylim(0, 0.05)
#axxCOM.plot(x, xCOMAvg, label='Avg')
#axxCOM.plot(x, xCOMAvgMStd, label='Avg-1*std')
#axxCOM.plot(x, xCOMAvg, label='X_CoM')

#axxCOM.plot(x, [0.002]*len(x), label='2milli', linewidth=2)
#axxCOM.plot(x, [-0.002]*len(x), label='-2milli')

#TODO: Need to change from zero to real values
#axxCOM.scatter(x, [0]*len(x), label='Real', s=1)
#axxCOM.plot(x, [0]*len(x), label='Zero', linewidth=2)


axxCOM.set_xlabel('Number of Poses', fontsize = 18)
#axxCOM.set_ylabel('X_CoM')
axxCOM.set_ylabel('Error', fontsize = 18)
axxCOM.legend(fontsize = 18);

# # Plot xCOMDiff Avg +/- std and Zero
# xCOMDiff = xCOM - xCOMReal
# xCOMDiff = np.absolute(xCOMDiff)
#
# xCOMAvgDiff = [np.mean(item, axis=0) for item in xCOMDiff];
# stdDiff = [np.std(item, axis=0) for item in xCOMDiff];
# xCOMAvgDiffPStd = np.add(xCOMAvgDiff, stdDiff);
# xCOMAvgDiffMStd = np.subtract(xCOMAvgDiff, stdDiff);
#
# figxCOMDiff = plt.figure()
# figxCOMDiff.suptitle("X_CoM Differences for Many Betas during Learning")
#
# axxCOMDiff = figxCOMDiff.add_subplot(111)
# axxCOMDiff.plot(x, xCOMAvgDiffPStd, label='Avg+1*std')
# axxCOMDiff.plot(x, xCOMAvgDiffMStd, label='Avg-1*std')
# axxCOMDiff.plot(x, xCOMAvgDiff, label='Avg')
# axxCOMDiff.plot(x, [0.002]*len(x), label='2milli')
# axxCOMDiff.plot(x, [0]*len(x), label='Zero')
# axxCOMDiff.plot(x, [-0.002]*len(x), label='-2milli')
#
# axxCOMDiff.set_xlabel('Number of Poses')
# axxCOMDiff.set_ylabel('X_CoM Diff')
# axxCOMDiff.legend();

# Plot totalMass Avg +/- std and Zero
#tmAvg = [np.mean(item, axis=0) for item in totalMass];
#std = [np.std(item, axis=0) for item in totalMass];
#tmAvgPStd = np.add(tmAvg, std);
#tmAvgMStd = np.subtract(tmAvg, std);
#
#figtm = plt.figure()
##figtm.suptitle("Total Mass for Many Betas during Learning")
#
#axtm = figtm.add_subplot(111)
##TODO no need for avg and std for hardware plots
##axtm.plot(x, tmAvgPStd, label='Avg+1*std')
##axtm.plot(x, tmAvgMStd, label='Avg-1*std')
##axtm.plot(x, tmAvg, label='Avg')
#axtm.plot(x, tmAvg, label='Mass')
## TODO hardcoded
##axtm.plot(x, [162.158]*len(x), label='Actual')
#
#axtm.set_xlabel('Number of Poses')
#axtm.set_ylabel('Total Mass')
#axtm.legend();
#
#numBetasForPose = xCOM.shape[1]
#numBetaParams = beta.shape[1]
#
#beta = beta.reshape(len(xCOM), numBetasForPose, numBetaParams)
#betaAvg = np.mean(beta, axis=1)
#betaStd = np.std(beta, axis=1)
#betaAvgPStd = np.add(betaAvg, betaStd);
#betaAvgMStd = np.subtract(betaAvg, betaStd);
#
#dotsize = 10
##for i in range(0, int(numBetaParams/4)):
##for i in range(0, 6):
## TODO set a hard limit
#bodiesToGraph = [0, 3, 4, 5, 16]
#for i in bodiesToGraph:
#    # TODO Maybe read these values from a list (the body names)
#    fig = plt.figure()
#    fig.suptitle("Body " + str(i + 1))
#    for j in range(0, 4):
#        ax = fig.add_subplot(220 + j + 1)
#        betaBodyAvg = betaAvg[:, i * 4 + j]
#        betaBodyStd = betaStd[:, i * 4 + j]
#        betaBodyAvgPStd = betaAvgPStd[:, i * 4 + j]
#        betaBodyAvgMStd = betaAvgMStd[:, i * 4 + j]
#
#        # TODO dont need avg and real for hardware
#        #ax.plot(x, betaBodyAvgPStd, label='Avg+1*std')
#        #ax.plot(x, betaBodyAvgMStd, label='Avg-1*std')
#        ax.plot(x, betaBodyAvg, label='Avg')
#        #ax.plot(x, [betaIdeal[0][i * 4 + j]]*len(x), label='Real')
#
#        ax.set_xlabel('Number of Poses')
#        if (j == 0):
#            ax.set_ylabel('Mass (?)')
#        if (j == 1):
#            ax.set_ylabel('x_com(?)')
#        if (j == 2):
#            ax.set_ylabel('y_com(?)')
#        if (j == 3):
#            ax.set_ylabel('z_com(?)')

plt.show()
