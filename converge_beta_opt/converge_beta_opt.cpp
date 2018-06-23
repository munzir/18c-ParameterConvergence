// Author: Akash Patel (apatel435@gatech.edu)

// genPhiMatrixAsFile
// Purpose: Determine phi vectors for each input pose
//   This phi will be used for finding beta (weights of actual robot) via gradient descent
//
// Input: Ideal beta={mi, MXi, MYi, ...}, krang urdf model, perturbation value,
//   potentially unbalanced data points (q/poses) as a file,
// Output: Phi matrix as a file

// convergeToBeta
// Purpose: Converge to an optimal beta vector
//   This beta is the parameter vector of Krang
//
// Input: Perturbed beta={mi, MXi, MYi, ...}, phi matrix,
// Output: Converged beta vector as a file
//
//
// Overall Input: Poses in {heading, qBase, etc.} format
// Overall Output: Converged beta value
// Intermediary Input/Output Flow:
// Input Pose File -> Dart Poses -> Opt Dart Poses -> Phi Matrix
// Phi Matrix -> Converged Beta
//

////////////////////////////////////////////////////////////////////////////////
//           Use your search feature to find hardcoded inputs to this script
//           Search for "INPUT on below"
////////////////////////////////////////////////////////////////////////////////

// TODO: Need to combine methods together so I am not reading/writing files more
// than once and not creating multiple copies of the same data
// TODO: Perform C++ warning checks
// TODO: Check for memory leaks (valgrind)
// TODO: Optimize script for speed

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

#define MAXBUFSIZE ((int) 1e6)

int genPhiMatrixAsFile() {

    // Put a hard stop on reading poses just in case
    // INPUT on below line (Hard stop to number of pose readings)
    //int controlPoseNums = 300;
    int controlPoseNums = 10000;
    // Get `complete` convergence with 600ish (1000)
    // INPUT on below line (lines to skip so an even distribution of samples can
    // be taken) Dependent on file lines
    //int linesToSkip = 1000/controlPoseNums;

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers (the pose params)
    ifstream infile;
    // INPUT on below line (input pose file)
    //infile.open("../balancedPoses_0-001T.txt");
    infile.open("../randomOptPoses10000.txt");
    cout << "Reading input poses ...\n";
    //int lineNumber = 0;
    while(! infile.eof() && rows <= controlPoseNums) {
        //if (lineNumber == linesToSkip) {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];
        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
        //lineNumber = 0;
        //}
        //lineNumber++;
    }

    infile.close();
    cout << "|-> Done\n";
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    int numInputPoses = rows;
    int numParams = cols;

    Eigen::MatrixXd allInitPoseParamsFromFile(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            allInitPoseParamsFromFile(i,j) = buff[cols*i+j];

    Eigen::MatrixXd allInitPoseParams(cols, rows);
    allInitPoseParams = allInitPoseParamsFromFile.transpose();

    // Perturbation Value
    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(1, -17);

    // Instantiate "ideal" robot and n other robots
    cout << "Creating ideal beta vector ...\n";
    dart::utils::DartLoader loader;
    // INPUT on below line (absolute path of the Krang URDF file)
    dart::dynamics::SkeletonPtr idealRobot = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    // Create ideal beta
    // Wait do we get beta from the ideal robot or give it to the ideal robot?
    // Beta Definition/Format
    // mi, mxi, myi, mzi for each body

    int bodyParams = 4;
    int numBodies = idealRobot->getNumBodyNodes();
    dart::dynamics::BodyNodePtr bodyi;
    string namei;
    double mi;
    double xMi;
    double yMi;
    double zMi;

    Eigen::MatrixXd betaParams(1, numBodies*bodyParams);
    int numPertRobots = numBodies*bodyParams;

    for (int i = 0; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);
        namei = bodyi->getName();
        mi = bodyi->getMass();
        xMi = mi * bodyi->getLocalCOM()(0);
        yMi = mi * bodyi->getLocalCOM()(1);
        zMi = mi * bodyi->getLocalCOM()(2);

        betaParams(0, i * bodyParams + 0) = mi;
        betaParams(0, i * bodyParams + 1) = xMi;
        betaParams(0, i * bodyParams + 2) = yMi;
        betaParams(0, i * bodyParams + 3) = zMi;
    }
    cout << "|-> Done\n";
    cout << "Creating robot array ...\n";
    // TODO: Need to create an array of pertRobots in a fast time
    // Create array of robots out of pose loop for efficiency
    // then change appropriate values (betaParams(i)) for each robot when
    // going through all the robots
    dart::dynamics::SkeletonPtr pertRobotArray[sizeof(SkeletonPtr) * numPertRobots];
    for (int i = 0; i < numPertRobots; i++) {

        // TODO: Segfaulting right here
        // Trying to create an array of idealRobots by calling parseSkeleton
        // only once since it is time expenseive
        //memcpy(pertRobotArray[i], idealRobot, sizeof(SkeletonPtr));

        pertRobotArray[i] = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    }

    cout << "|-> Done\n";
    //Create file to print the actual xCOM of the poses for the ideal robot
    ofstream realxCOMFile;
    realxCOMFile.open("realxCOM.txt");

    // Find phiMatrix
    cout << "Calculating Phi Matrix ...\n";
    Eigen::MatrixXd phiMatrix(numInputPoses, numPertRobots);
    double phi = 0;

    // Find the pose in DART formats
    for (int pose = 0; pose < numInputPoses; pose++) {

        Eigen::MatrixXd optDartPoseParams(25, 1);
        optDartPoseParams = allInitPoseParams.col(pose);

        // Set position of ideal robot to the pose in DART format
        idealRobot->setPositions(optDartPoseParams);

        // Get x center of mass
        double xCOMIdealRobot = idealRobot->getCOM()(0);
        realxCOMFile << xCOMIdealRobot << "\n";

        for (int pertRobotNum = 0; pertRobotNum < numPertRobots; pertRobotNum++) {

            if (pertRobotNum % bodyParams == 0) {
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMass(betaParams(0, pertRobotNum) + perturbedValue);
            }
            else if (pertRobotNum % bodyParams == 1) {
                Eigen::Vector3d bodyMCOM(betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1), betaParams(0, pertRobotNum + 2));
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyMCOM/(betaParams(0,pertRobotNum - 1)));
            } else if (pertRobotNum % bodyParams == 2) {
                Eigen::Vector3d bodyMCOM(betaParams(0, pertRobotNum - 1), betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1));
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyMCOM/(betaParams(0,pertRobotNum - 2)));
            } else {
                Eigen::Vector3d bodyMCOM(betaParams(0, pertRobotNum - 2), betaParams(0, pertRobotNum - 1), betaParams(0, pertRobotNum) + perturbedValue);
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyMCOM/(betaParams(0,pertRobotNum - 3)));
            }

            // Set perturbed robot position to pose
            pertRobotArray[pertRobotNum]->setPositions(optDartPoseParams);

            // Get the center of mass of the perturbedRobot
            double xCOMpertRobot = pertRobotArray[pertRobotNum]->getCOM()(0);

            // Calculate phi for betai and pose
            phi = (xCOMpertRobot - xCOMIdealRobot)/perturbedValue;

            // Add phi to phiMatrix and then print it looks cleaner
            phiMatrix(pose, pertRobotNum) = phi;
        }
    }

    // Close the real xCOM file
    realxCOMFile.close();

    // Open output file to write phi matrix
    ofstream phiFile;
    phiFile.open("phiMatrix.txt");

    // phi should be the same no matter estimation/perturbation
    phiFile << phiMatrix;
    phiFile.close();
    cout << "|-> Done\n";

    return 0;
}

int convergeToBeta() {

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // I feel like there is a better way to read the file (if it fits it ships?)
    // Read numbers (the pose params)
    ifstream infile;
    // infile.open("../defaultInit.txt");
    // INPUT on below line (input phi matrix file)
    infile.open("phiMatrix.txt");
    while(! infile.eof()) {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();
    //rows--;
    int numInputPoses = rows;
    int numBetaParams = cols;

    // Populate matrix with numbers.
    // every column is a paramCoeff, the rows are the phi vectors for each data
    // point
    Eigen::MatrixXd phiMatrix(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            phiMatrix(i, j) = buff[cols*i+j];

    // make idealRobot a copy of krang model
    dart::utils::DartLoader loader;
    // INPUT on below line (absolute path of the Krang URDF model
    dart::dynamics::SkeletonPtr idealRobot = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    // Beta Definition/Format
    // mi, mxi, myi, mzi for each body
    int bodyParams = 4;
    int numBodies = idealRobot->getNumBodyNodes();
    dart::dynamics::BodyNodePtr bodyi;
    string namei;
    double mi;
    double xMi;
    double yMi;
    double zMi;

    //TODO: Need to add perturbation to the ideal beta value read
    //I think should be read as input but for now let's manually create the
    //perturbation from the ideal beta
    // INPUT on below lines (need to create a prior beta value aka betaHat)
    double deviation = 0.00;
    double offset = 0.00;
    Eigen::MatrixXd nonIdealBetaParams(1, numBodies*bodyParams);

    ofstream krangSpecsFile;
    krangSpecsFile.open("krangSpecs.txt");

    for (int i = 0; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);
        namei = bodyi->getName();
        mi = bodyi->getMass();
        xMi = mi * bodyi->getLocalCOM()(0);
        yMi = mi * bodyi->getLocalCOM()(1);
        zMi = mi * bodyi->getLocalCOM()(2);

        krangSpecsFile << namei << " " << mi << " " << xMi << " " << yMi << " " << zMi << "\n";

        nonIdealBetaParams(0, i * bodyParams + 0) = mi + deviation * mi + offset;
        nonIdealBetaParams(0, i * bodyParams + 1) = xMi + deviation * xMi + offset;
        nonIdealBetaParams(0, i * bodyParams + 2) = yMi + deviation * yMi + offset;
        nonIdealBetaParams(0, i * bodyParams + 3) = zMi + deviation * zMi + offset;

    }

    krangSpecsFile.close();

    // Initialize constants/hyperparameters
    // Eons (how many times to learn on same dataset)
    // INPUT on below line (eons)
    int eons = 1;

    // Learning Rate
    // INPUT on below line (learning rate)
    double n = 1000.0;
    // not converging fast enough with 1900, 1700, 1500, 1300 and values below
    // 500
    //Best so far with u = 0 n = 1100.0/900.0 would need to do comparisons of
    //the average of the last 50 values for better decision making
    //stick with 1000 for now
    //0.1

    // Regularizes the importance of the masses with respect to the moments
    // INPUT on below line (mass coefficient for regularization
    double u = 0.0;
    //0.1

    // Mass Indicator Matrix
    Eigen::MatrixXd massIndicatorMatrix(1, numBetaParams);
    for (int i = 0; i < numBetaParams; i++) {
        if (i % 4 == 0) {
            massIndicatorMatrix(0, i) = 1;
        } else {
            massIndicatorMatrix(0, i) = 0;
        }
    }

    // Open output file to write beta vector
    ofstream betaFile;
    betaFile.open("betaVectors.txt");
    // Open output file to write xCOM errors
    ofstream xCOMValuesFile;
    xCOMValuesFile.open("xCOMValues.txt");
    // Open output file to write total mass values
    ofstream totalMassFile;
    totalMassFile.open("totalMassValues.txt");

    // Update beta params while looping through all the input phi vectors of
    // their respective poses
    Eigen::MatrixXd currBeta = nonIdealBetaParams;
    Eigen::MatrixXd nextBeta(1, numBetaParams);

    Eigen::MatrixXd phiVec(1, numBetaParams);
    Eigen::MatrixXd xCOM(1, 1);
    Eigen::MatrixXd xCOMValue(1, 1);
    Eigen::MatrixXd delta(1, numBetaParams);

    Eigen::MatrixXd totalMass(1, 1);
    //totalMass << idealRobot->getMass();

    // 1 means it has converged to a solution with a suitable error
    int hasConverged = 0;
    // INPUT on below line (satisfactory error value)
    double suitableError = 0;

    // Write the prior beta vector
    cout << "Converging to a Beta Vector ...\n";
    betaFile << currBeta << "\n";
    // Loop through eons
    for (int k = 0; k < eons; k++) {
    // Loop through the phi matrix
    for (int pose = 0; pose < numInputPoses && hasConverged == 0; pose++) {
        phiVec = phiMatrix.row(pose);
        // Not sure how to get total mass do i use perturbed robot or ideal
        // robot ?
        xCOM = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - totalMass));

        // Should/can also write the error to a file for analysis
        xCOMValue = phiVec * currBeta.transpose();
        xCOMValuesFile << xCOMValue << "\n";

        totalMass = massIndicatorMatrix * currBeta.transpose();
        totalMassFile << totalMass << "\n";

        // Use absolute value of error to see if solution is suitable or not
        //if (abs(xCOMValue(0, 0)) <= suitableError) {
        //    hasConverged = 1;
        //}

        delta = phiVec + (u * massIndicatorMatrix);
        // Update currBeta parameter vector
        // currBeta = currBeta - n * delta;
        currBeta = currBeta - (n * (xCOM * delta));

        // Write the updated beta vector
        betaFile << currBeta << "\n";

    }
    }

    // Write the xCOM error of the last beta and the last pose (so the last pose
    // is used twice with two different betas)
    // Same with the total mass

    xCOMValue = phiVec * currBeta.transpose();
    xCOMValuesFile << xCOMValue << "\n";

    totalMass = massIndicatorMatrix * currBeta.transpose();
    totalMassFile << totalMass << "\n";

    betaFile.close();
    cout << "|-> Done\n";
    xCOMValuesFile.close();

    return 0;
}

int main() {
    genPhiMatrixAsFile();
    convergeToBeta();
}


