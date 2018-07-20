# 18c: Parameter Convergence
This project attempts to converge to a set of parameters i.e. beta (mass & CoM for each robot body) given a set of input poses by generating a phi matrix and learning on the input data.

## train\_beta.cpp
Converges to a set of parameters for the robot and outputs:
- phi matrix
- parameter vectors (for each initial beta) every update
- x center of mass estimation for each initial beta as it updates
- filtered poses (the poses actually used to learn)

### Dependencies
- DART (at least version 6) [Dart Homepage](https://dartsim.github.io)

### Build and Run
1: Enter the param\_converge\train\_beta directory

2: Build the project

    mkdir build
    cd build
    cmake ..
    make

3: Run the project

    ./train_beta

## filter\_training\_set.cpp
Same procedure and output as train\_beta, but selectively filters the best poses to train
on. Includes a convergence condition as well.

### Dependencies
Same as train\_beta

### Build and Run
Same as train\_beta

## test\_beta.cpp
Tests beta values with the ideal beta value comparing the xcom values outputting:
- xcom for the ideal beta
- xcom for each converged beta

### Dependencies
Same as train\_beta

### Build and Run
Same as train\_beta

## graph\_beta\_values.py
A script that graphs the average and average +/- standard deviation based on the beta values as well as the real values based on the ideal beta. It also graphs the average and average +/- standard deviation of the differences of the xcom values.
Takes in the output files from `train_beta` in the following order.
\*Note: the ideal beta value file is not outputted by `train_beta`

1: xcom file

2: total mass file

3: beta values file

4. ideal beta values file

### Dependencies
- Numpy [Numpy Installation](https://www.scipy.org/scipylib/download.html)
- Matplotlib [Matplotlib
  Installation](https://matplotlib.org/users/installing.html)

### Run
1: Run the script

    python3 <Path of graph_beta_values> <Path of xcom file> <Path of total mass file> <Path of beta file> <Path of ideal beta file>

e.g.

    python3 ../../../data_analysis/graph_beta_values.py xCOMValues.txt totalMassValues.txt betaVectors.txt idealBetaVector.txt &

## graph\_xcom\_values.py
A script that graphs the average and average +/- standard deviation xcom values based on the beta values as well as the real values based on the ideal beta. It also graphs the average and average +/- standard deviation of the differences for all the xcom values compared to the ideal xcom value.
Takes in xcom file with format of [ideal xcom | predicted xcom].
Same as outputted by test\_beta

### Dependencies
Same as `graph_beta_values.py`

### Run
1: Run the script

    python3 <Path of graph_xcom_values> <Path of xcom file>

e.g.

    python3 ../../../data_analysis/graph_xcom_values.py xCOMValues.txt &

Tip: Append `&` at the end of the command to run it in the background.
