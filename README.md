# 18c: Parameter Convergence
This project attempts to converge to a set of parameters i.e. beta (mass & CoM for each robot body) given a set of input poses by generating a phi matrix and learning on the input data.

## converge\_beta.cpp
Converges to a set of parameters for the robot and outputs:
- phi matrix
- parameter vectors (beta) every update
- x center of mass estimation as it updates

### Dependencies
- DART (at least version 6) [Dart Homepage](https://dartsim.github.io)

### Build and Run
1: Enter the converge\_beta directory

2: Build the project

    mkdir build
    cd build
    cmake ..
    make

3: Run the project

    ./converge_beta.cpp

## graph\_beta\_values.py
A script that graphs the generated beta values compared to the ideal values
Takes in the output files from `converge_beta` in the following order.

1: xCoM file

2: total mass file

3: beta values file

### Dependencies
- Numpy [Numpy Installation](https://www.scipy.org/scipylib/download.html)
- Matplotlib [Matplotlib
  Installation](https://matplotlib.org/users/installing.html)

### Run
1: Run the script

    python3 <Path of graph_beta_values> <Path of xCoM file> <Path of total mass file> <Path of beta file>

e.g.

    python3 ../../graph_beta_values.py xCOMValues.txt totalMassValues.txt betaVectors.txt &

Tip: Append `&` at the end of the command to run it in the background.
