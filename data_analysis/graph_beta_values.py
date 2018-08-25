#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import sys

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
xCOM = np.delete(xCOM, 0, axis=1)
xCOMNotAbs = xCOM
xCOM = np.absolute(xCOM)

# The number of x data points
x = list(range(1, len(xCOM) + 1))

totalMass = [[float(item) for item in itemList] for itemList in totalMassMatrix]
totalMass = np.array(totalMass)

beta = [[float(item) for item in itemList] for itemList in betaMatrix]
beta = np.array(beta)

betaIdeal = [[float(item) for item in itemList] for itemList in betaIdealMatrix]
betaIdeal = np.array(betaIdeal)

# Plot xCOM Avg +/- std and max and Real

xCOMAvg = [np.mean(item, axis=0) for item in xCOM];
std = [np.std(item, axis=0) for item in xCOM];
xCOMAvgPStd = np.add(xCOMAvg, std);
xCOMAvgMStd = np.subtract(xCOMAvg, std);
xCOMMax = [np.max(item, axis=0) for item in xCOM];

figxCOM = plt.figure()
figxCOM.suptitle("X_CoM for Many Betas during Learning")

axxCOM = figxCOM.add_subplot(111)
axxCOM.plot(x, xCOMMax, label='Max')
axxCOM.plot(x, xCOMAvgPStd, label='Avg+1*std')
#axxCOM.plot(x, xCOMAvgMStd, label='Avg-1*std')
axxCOM.plot(x, xCOMAvg, label='Avg')
#TODO: Need to change from zero to real values
# I mean with the right balancing they are p much zero so its fine
axxCOM.scatter(x, [0]*len(x), label='Real', s=1)

axxCOM.plot(x, [0.002]*len(x), label='2milli')
#axxCOM.plot(x, [-0.002]*len(x), label='-2milli')

axxCOM.set_xlabel('Number of Poses')
axxCOM.set_ylabel('X_CoM')
axxCOM.legend();

# Plot xCOMDiff Avg +/- std and Zero
# make xCOM the unabsolute value stored before for difference calculation
xCOM = xCOMNotAbs
xCOMDiff = xCOM - xCOMReal
xCOMDiff = np.absolute(xCOMDiff)

xCOMAvgDiff = [np.mean(item, axis=0) for item in xCOMDiff];
stdDiff = [np.std(item, axis=0) for item in xCOMDiff];
xCOMAvgDiffPStd = np.add(xCOMAvgDiff, stdDiff);
xCOMAvgDiffMStd = np.subtract(xCOMAvgDiff, stdDiff);
xCOMMaxDiff = [np.max(item, axis=0) for item in xCOMDiff]

figxCOMDiff = plt.figure()
figxCOMDiff.suptitle("X_CoM Differences for Many Betas during Learning")

axxCOMDiff = figxCOMDiff.add_subplot(111)
axxCOMDiff.plot(x, xCOMMaxDiff, label='Max')
axxCOMDiff.plot(x, xCOMAvgDiffPStd, label='Avg+1*std')
#axxCOMDiff.plot(x, xCOMAvgDiffMStd, label='Avg-1*std')
axxCOMDiff.plot(x, xCOMAvgDiff, label='Avg')
axxCOMDiff.plot(x, [0.002]*len(x), label='2milli')
axxCOMDiff.plot(x, [0]*len(x), label='Zero')
#axxCOMDiff.plot(x, [-0.002]*len(x), label='-2milli')

axxCOMDiff.set_xlabel('Number of Poses')
axxCOMDiff.set_ylabel('X_CoM Diff')
axxCOMDiff.legend();

# Plot totalMass Avg +/- std and Zero
tmAvg = [np.mean(item, axis=0) for item in totalMass];
std = [np.std(item, axis=0) for item in totalMass];
tmAvgPStd = np.add(tmAvg, std);
tmAvgMStd = np.subtract(tmAvg, std);

figtm = plt.figure()
figtm.suptitle("Total Mass for Many Betas during Learning")

axtm = figtm.add_subplot(111)
axtm.plot(x, tmAvgPStd, label='Avg+1*std')
axtm.plot(x, tmAvgMStd, label='Avg-1*std')
axtm.plot(x, tmAvg, label='Avg')
# TODO hardcoded
axtm.plot(x, [162.158]*len(x), label='Actual')

axtm.set_xlabel('Number of Poses')
axtm.set_ylabel('Total Mass')
axtm.legend();

numBetasForPose = xCOM.shape[1]
numBetaParams = beta.shape[1]

beta = beta.reshape(len(xCOM), numBetasForPose, numBetaParams)
betaAvg = np.mean(beta, axis=1)
betaStd = np.std(beta, axis=1)
betaAvgPStd = np.add(betaAvg, betaStd);
betaAvgMStd = np.subtract(betaAvg, betaStd);

dotsize = 10
#for i in range(0, int(numBetaParams/4)):
# TODO set a hard limit
for i in range(0, 2):
    # TODO Maybe read these values from a list (the body names)
    fig = plt.figure()
    fig.suptitle("Body " + str(i + 1))
    for j in range(0, 4):
        ax = fig.add_subplot(220 + j + 1)
        betaBodyAvg = betaAvg[:, i * 4 + j]
        betaBodyStd = betaStd[:, i * 4 + j]
        betaBodyAvgPStd = betaAvgPStd[:, i * 4 + j]
        betaBodyAvgMStd = betaAvgMStd[:, i * 4 + j]

        ax.plot(x, betaBodyAvgPStd, label='Avg+1*std')
        ax.plot(x, betaBodyAvgMStd, label='Avg-1*std')
        ax.plot(x, betaBodyAvg, label='Avg')
        ax.plot(x, [betaIdeal[0][i * 4 + j]]*len(x), label='Real')

        ax.set_xlabel('Number of Poses')
        if (j == 0):
            ax.set_ylabel('Mass (?)')
        if (j == 1):
            ax.set_ylabel('x_com(?)')
        if (j == 2):
            ax.set_ylabel('y_com(?)')
        if (j == 3):
            ax.set_ylabel('z_com(?)')

plt.show()
