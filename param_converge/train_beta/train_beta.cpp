// Author: Akash Patel (apatel435@gatech.edu)

// trainBeta
// Purpose: Converge to an optimal beta vector
//   This beta is the parameter vector of Krang
//
// Input: Perturbed beta={mi, MXi, MYi, ...}, phi matrix,
// Output: Converged beta vector as a file
//
// Overall Input: Poses in dart format
// Overall Output: Converged beta value

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>

#include "../gen_phi_matrix.hpp"
#include "../file_ops.hpp"

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// Function Prototypes
// // Converge to Beta
Eigen::MatrixXd trainBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, int bodyParams, string fullRobotPath, double maxDeviation, double maxOffset, int eons, double learningRate, double massRegularization, double suitableError, int suitableNumPoses, double mag, int exp);

// // Change robot's beta values (parameters)
SkeletonPtr setParameters(SkeletonPtr robot, Eigen::MatrixXd beta, int bodyParams);

// // Random Value
double fRand(double fMin, double fMax);

// // Absolute Value Average
double absAverage(Eigen::MatrixXd vector, int index, int total);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (Random Seed)
    srand(0);

    // INPUT on below line (input poses filename)
    //string inputPosesFilename = "../custom2comfullbalancenotolunsafe.txt";
    string inputPosesFilename = "../randomOptPoses10000.txt";

    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(10, -8);

    // INPUT on below line (number of parameters for each body)
    int bodyParams = 4;

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf";

    // INPUT on below lines (need to create a prior beta value aka betaHat)
    double maxDeviation = 0.50;
    double maxOffset = 0.50;

    // Eons (how many times to learn on same dataset)
    // INPUT on below line (eons)
    int eons = 1;

    //Best so far with u = 0 n = 300 would need to do comparisons of
    // INPUT on below line (learning rate)
    double learningRate = 300;
    // optimal value between 200 & 300
    // Determined by using a threshold and whichever learning rate gives us
    // least number of filtered poses
    //double learningRate = 0.05;

    // Regularizes the importance of the masses with respect to the moments
    // INPUT on below line (mass coefficient for regularization)
    double massRegularization = 0.0;

    // INPUT on below line (satisfactory error value and poses averaged over)
    double suitableError = 0.002;
    int suitableNumPoses = 100;

    // INPUT on below line (threshold to filter out)
    // threshold = mag * 10^exp
    double mag = 2;
    int exp = -3;

    string inputName = extractFilename(inputPosesFilename);

    cout << "Reading Input Poses ...\n";
    Eigen::MatrixXd inputPoses = readInputFileAsMatrix(inputPosesFilename);
    cout << "|-> Done\n";

    Eigen::MatrixXd phiMatrix = genPhiMatrix(inputPoses, bodyParams, fullRobotPath, perturbedValue);

    cout << "Converging to Beta ...\n";
    Eigen::MatrixXd betaVectors = trainBeta(inputName, inputPoses, phiMatrix, bodyParams, fullRobotPath, maxDeviation, maxOffset, eons, learningRate, massRegularization, suitableError, suitableNumPoses, mag, exp);
    cout << "|-> Done\n";


    // Write betaVectors to file
    string outfilename;
    string ext = ".txt";
    string outputBaseName = "betaVectors";

    outfilename = outputBaseName + inputName + to_string(mag) + "*10e" + to_string(exp) + "filter" + ext;

    cout << "Beta Vectors written to " << outfilename << endl;

    //cout << "Writing beta vectors to " << outfilename << " ...\n";

    //ofstream outfile;
    //outfile.open(outfilename);
    //outfile << betaVectors;
    //outfile.close();

    //cout << "|-> Done\n";
}

// // Converge to Beta
// TODO: Add total mass constraint
Eigen::MatrixXd trainBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, int bodyParams, string fullRobotPath, double maxDeviation, double maxOffset, int eons, double n, double u, double suitableError, int suitableNumPoses, double mag, int exp) {
    int numInputPoses = phiMatrix.rows();
    int numBetaParams = phiMatrix.cols();

    // Make idealRobot a copy of krang model
    DartLoader loader;
    SkeletonPtr idealRobot = loader.parseSkeleton(fullRobotPath);

    // Beta Definition/Format
    // mi, mxi, myi, mzi for each body
    int numBodies = idealRobot->getNumBodyNodes();
    BodyNodePtr bodyi;
    string namei;
    double mi;
    double mxi;
    double myi;
    double mzi;

    //TODO: Need to add perturbation to the ideal beta value read
    //I think should be read as input but for now let's manually create the
    //perturbation from the ideal beta as random
    // Random value betwen +/- deviation and add it to ideal value
    // as well as an offset since some values are zero.
    double deviation;
    double offset;

    Eigen::MatrixXd nonIdealBetaParams(1, numBodies*bodyParams);

    ofstream krangSpecsFile;
    krangSpecsFile.open("krangSpecs.txt");

    for (int i = 0; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);
        namei = bodyi->getName();
        mi = bodyi->getMass();
        mxi = mi * bodyi->getLocalCOM()(0);
        myi = mi * bodyi->getLocalCOM()(1);
        mzi = mi * bodyi->getLocalCOM()(2);

        krangSpecsFile << namei << " " << mi << " " << mxi << " " << myi << " " << mzi << "\n";

        nonIdealBetaParams(0, i * bodyParams + 0) = mi;

        deviation = fRand(-maxDeviation, maxDeviation);
        offset = fRand(-maxOffset, maxOffset);
        nonIdealBetaParams(0, i * bodyParams + 1) = mxi + deviation * mxi + offset;

        deviation = fRand(-maxDeviation, maxDeviation);
        offset = fRand(-maxOffset, maxOffset);
        nonIdealBetaParams(0, i * bodyParams + 2) = myi + deviation * myi + offset;

        deviation = fRand(-maxDeviation, maxDeviation);
        offset = fRand(-maxOffset, maxOffset);
        nonIdealBetaParams(0, i * bodyParams + 3) = mzi + deviation * mzi + offset;

    }

    krangSpecsFile.close();

    // Mass Indicator Matrix
    Eigen::MatrixXd massIndicatorMatrix(1, numBetaParams);
    for (int i = 0; i < numBetaParams; i++) {
        if (i % 4 == 0) {
            massIndicatorMatrix(0, i) = 1;
        } else {
            massIndicatorMatrix(0, i) = 0;
        }
    }

    // Start with adding initial beta to betaVectors
    Eigen::MatrixXd betaVectors = nonIdealBetaParams;
    Eigen::MatrixXd currBeta = nonIdealBetaParams;

    // TODO: Not assigning values inside the matrix properly for xCOM and
    // totalMass potential fix in test_beta
    Eigen::MatrixXd phiVec(1, numBetaParams);
    Eigen::MatrixXd delta(1, numBetaParams);

    Eigen::MatrixXd xCOM(1, 1);
    Eigen::MatrixXd xCOMValues((eons*numInputPoses)+1,1);
    double xCOMValue;

    Eigen::MatrixXd totalMassValues((eons*numInputPoses)+1,1);
    double totalMass;
    Eigen::MatrixXd idealTotalMass(1, 1);
    idealTotalMass << idealRobot->getMass();

    SkeletonPtr currRobot = idealRobot->clone();

    // Base outout filename
    string ext = ".txt";
    string outBaseFilename = inputName + to_string(mag) + "*10e" + to_string(exp) + "filter" + ext;
    // Open output file to write beta vectors
    ofstream betaVectorsFile;
    string betaVectorsFilename = "betaVectors" + outBaseFilename;
    betaVectorsFile.open(betaVectorsFilename);
    //Initial beta vector
    betaVectorsFile << currBeta << endl;
    // Open output file to write poses used
    ofstream filteredPosesFile;
    string filteredPosesFilename = "filteredPoses" + outBaseFilename;
    filteredPosesFile.open(filteredPosesFilename);
    // Open output file to write xCOM values
    ofstream xCOMValuesFile;
    string xCOMValuesFilename = "xCOMValues" + outBaseFilename;
    xCOMValuesFile.open(xCOMValuesFilename);
    // Open output file to write total mass values
    ofstream totalMassFile;
    string totalMassFilename = "totalMassValues" + outBaseFilename;
    totalMassFile.open(totalMassFilename);

    // 1 means it has converged to a solution with a suitable error
    int hasConverged = 0;
    // Threshold that signifies if new data is not learning much
    // TODO: This favors learning on parameters that heavily change xcom (i
    // think it should be fine tho: the lower we go the more we compensate for
    // it)
    double threshold = mag * std::pow(10, exp);

    // Loop through eons
    for (int k = 0; k < eons; k++) {
    // Loop through the phi matrix to calculate the beta vectors
    for (int pose = 0; pose < numInputPoses && hasConverged == 0; pose++) {
        phiVec = phiMatrix.row(pose);

        xCOMValue = (phiVec * currBeta.transpose())(0, 0);
        // currRobot = setParameters(idealRobot, currBeta, bodyParams);
        // xCOMValue = currRobot->getCOM()(0);

        // Filter out poses that do not change the xCOM value much
        if (abs(xCOMValue) >= threshold) {

            // Write this pose
            filteredPosesFile << inputPoses.row(pose) << endl;

            xCOM = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - idealTotalMass));
            // Append the next xCOM
            //xCOMValues(k*pose, 0) = xCOMValue;
            xCOMValuesFile << xCOMValue << endl;

            totalMass = (massIndicatorMatrix * currBeta.transpose())(0, 0);
            totalMassValues(k*pose, 0) = totalMass;
            totalMassFile << totalMass << endl;

            // Use absolute value of average error to see if solution is suitable or not
            // TODO: Need to properly create xCOMValues
            //if (pose >= suitableNumPoses && absAverage(xCOMValues, pose, suitableNumPoses) <= suitableError) {
            //    hasConverged = 1;
            //}

            delta = phiVec + (u * massIndicatorMatrix);
            // Update currBeta parameter vector
            //currBeta = currBeta - n * delta;
            //currBeta = currBeta - n * xcom;
            //cout << xCOM << endl;
            currBeta = currBeta - (n * (xCOM * delta));

            betaVectorsFile << currBeta << endl;

            // Append the updated beta vector
            //Eigen::MatrixXd betaTmp(betaVectors.rows()+currBeta.rows(), currBeta.cols());
            //betaTmp << betaVectors,
                       //currBeta;
            //betaVectors = betaTmp;
        }
    }
    }

    betaVectorsFile.close();
    filteredPosesFile.close();

    // Write the xCOM value of the last beta and the last pose
    // Same with the total mass
    xCOMValue = (phiVec * currBeta.transpose())(0, 0);
    // currRobot = setParameters(idealRobot, currBeta, bodyParams);
    // xCOMValue = currRobot->getCOM()(0);
    // xCOMValues(eons*numInputPoses, 0) = xCOMValue;
    xCOMValuesFile << xCOMValue << endl;

    // Open output file to write xCOM values
    // ofstream xCOMValuesFile;
    // xCOMValuesFile.open("xCOMValues.txt");
    // xCOMValuesFile << xCOMValues;
    xCOMValuesFile.close();

    totalMass = (massIndicatorMatrix * currBeta.transpose())(0, 0);
    // totalMassValues(eons*numInputPoses, 0) = totalMass;
    totalMassFile << totalMass << endl;

    // Open output file to write total mass values
    // ofstream totalMassFile;
    // totalMassFile.open("totalMassValues.txt");
    // totalMassFile << totalMassValues;
    totalMassFile.close();

    return betaVectors;
}

// // Change robot's beta values (parameters)
SkeletonPtr setParameters(SkeletonPtr robot, Eigen::MatrixXd betaParams, int bodyParams) {
    Eigen::Vector3d bodyMCOM;
    double mi;
    int numBodies = betaParams.cols()/bodyParams;
    for (int i = 0; i < numBodies; i++) {
        mi = betaParams(0, i * bodyParams);
        bodyMCOM(0) = betaParams(0, i * bodyParams + 1);
        bodyMCOM(1) = betaParams(0, i * bodyParams + 2);
        bodyMCOM(2) = betaParams(0, i * bodyParams + 3);

        robot->getBodyNode(i)->setMass(mi);
        robot->getBodyNode(i)->setLocalCOM(bodyMCOM/mi);
    }
    return robot;
}

// // Random Value
double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

// // Absolute Value Average Value
// TODO
double absAverage(Eigen::MatrixXd vector, int index, int total) {
    return 1;
}