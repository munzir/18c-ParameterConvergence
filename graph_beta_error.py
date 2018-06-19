#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import sys

filename = sys.argv[1]
betafilename = sys.argv[2]

betaVectors = np.loadtxt(betafilename)
betaVectors = [x.split() for x in open(betafilename).readlines()]
#print(betaVectors)

m1 = [float(item[0]) for item in betaVectors]
xm1 = [float(item[1]) for item in betaVectors]
ym1 = [float(item[2]) for item in betaVectors]
zm1 = [float(item[3]) for item in betaVectors]

m2 = [float(item[4]) for item in betaVectors]
m3 = [float(item[8]) for item in betaVectors]

m4 = [float(item[12]) for item in betaVectors]
xm4 = [float(item[13]) for item in betaVectors]
ym4 = [float(item[14]) for item in betaVectors]
zm4 = [float(item[15]) for item in betaVectors]

m5 = [float(item[16]) for item in betaVectors]
xm5 = [float(item[17]) for item in betaVectors]
ym5 = [float(item[18]) for item in betaVectors]
zm5 = [float(item[19]) for item in betaVectors]

m6 = [float(item[20]) for item in betaVectors]

m7 = [float(item[24]) for item in betaVectors]
xm7 = [float(item[25]) for item in betaVectors]
ym7 = [float(item[26]) for item in betaVectors]
zm7 = [float(item[27]) for item in betaVectors]

m8 = [float(item[28]) for item in betaVectors]
m9 = [float(item[32]) for item in betaVectors]
m10 = [float(item[36]) for item in betaVectors]
m11 = [float(item[40]) for item in betaVectors]
m12 = [float(item[44]) for item in betaVectors]

m13 = [float(item[48]) for item in betaVectors]
xm13 = [float(item[49]) for item in betaVectors]
ym13 = [float(item[50]) for item in betaVectors]
zm13 = [float(item[51]) for item in betaVectors]

m14 = [float(item[52]) for item in betaVectors]
m15 = [float(item[56]) for item in betaVectors]
m16 = [float(item[60]) for item in betaVectors]
m17 = [float(item[64]) for item in betaVectors]
m18= [float(item[68]) for item in betaVectors]
m19 = [float(item[72]) for item in betaVectors]
m20 = [float(item[76]) for item in betaVectors]
m21 = [float(item[80]) for item in betaVectors]
m22 = [float(item[84]) for item in betaVectors]
m23 = [float(item[88]) for item in betaVectors]
m24 = [float(item[92]) for item in betaVectors]

with open(filename) as f:
    data = f.read()

data = data.split('\n')

y = [row.split(' ')[0] for row in data]
x = list(range(1, len(y)))

y.pop()
y = [float(i) for i in y]

#ax1.set_title("Plot title...")
#ax1.set_xlabel('your x label..')
#ax1.set_ylabel('your y label...')

fig = plt.figure()
fig.suptitle("X_CoM")

ax = fig.add_subplot(111)
ax.plot(x,y)
ax.plot(x,[0]*len(x))

ax.set_xlabel('Number of Poses')
ax.set_ylabel('X_CoM')

fig1 = plt.figure()
fig1.suptitle("Base")
ax11 = fig1.add_subplot(221)
ax12 = fig1.add_subplot(222)
ax13 = fig1.add_subplot(223)
ax14 = fig1.add_subplot(224)

ax11.plot(x,m1)
ax11.plot(x,[75.767]*len(x))
ax12.plot(x,xm1)
ax12.plot(x,[-0.0475817]*len(x))
ax13.plot(x,ym1)
ax13.plot(x,[0.146988]*len(x))
ax14.plot(x,zm1)
ax14.plot(x,[5.81292]*len(x))

fig4 = plt.figure()
fig4.suptitle("Spine")
ax41 = fig4.add_subplot(221)
ax42 = fig4.add_subplot(222)
ax43 = fig4.add_subplot(223)
ax44 = fig4.add_subplot(224)

ax41.plot(x,m4)
ax41.plot(x,[14.006]*len(x))
ax42.plot(x,xm4)
ax42.plot(x,[-0.717415]*len(x))
ax43.plot(x,ym4)
ax43.plot(x,[0.00483207]*len(x))
ax44.plot(x,zm4)
ax44.plot(x,[1.12785]*len(x))

fig5 = plt.figure()
fig5.suptitle("Bracket")
ax51 = fig5.add_subplot(221)
ax52 = fig5.add_subplot(222)
ax53 = fig5.add_subplot(223)
ax54 = fig5.add_subplot(224)

ax51.plot(x,m5)
ax51.plot(x,[6.533]*len(x))
ax52.plot(x,xm5)
ax52.plot(x,[0]*len(x))
ax53.plot(x,ym5)
ax53.plot(x,[0.499415]*len(x))
ax54.plot(x,zm5)
ax54.plot(x,[-0.0390151]*len(x))

fig7 = plt.figure()
fig7.suptitle("L1")
ax71 = fig7.add_subplot(221)
ax72 = fig7.add_subplot(222)
ax73 = fig7.add_subplot(223)
ax74 = fig7.add_subplot(224)

ax71.plot(x,m7)
ax71.plot(x,[7.35]*len(x))
ax72.plot(x,xm7)
ax72.plot(x,[0.159495]*len(x))
ax73.plot(x,ym7)
ax73.plot(x,[-0.507885]*len(x))
ax74.plot(x,zm7)
ax74.plot(x,[0]*len(x))

fig13 = plt.figure()
fig13.suptitle("lGripper")
ax131 = fig13.add_subplot(221)
ax132 = fig13.add_subplot(222)
ax133 = fig13.add_subplot(223)
ax134 = fig13.add_subplot(224)

ax131.plot(x,m13)
ax131.plot(x,[2.3838]*len(x))
ax132.plot(x,xm13)
ax132.plot(x,[0]*len(x))
ax133.plot(x,ym13)
ax133.plot(x,[0]*len(x))
ax134.plot(x,zm13)
ax134.plot(x,[0.251491]*len(x))


#ax1.plot(x, y, c='r', label='Error')

#leg = ax1.legend()
#ax1.yaxis.set_major_locator(loc)
plt.show()
