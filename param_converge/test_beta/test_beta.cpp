// Author: Akash Patel (apatel435@gatech.edu)

// testBeta
// Purpose: test beta vector by analyzing the xCOM value
//   This beta is the parameter vector of Krang
//
// Input: beta={mi, MXi, MYi, ...}, phi matrix,
// Output: xCOM values at each pose as a file
//
// Overall Input: beta vector, Poses in dart format
// Overall Output: xCOM values
// Intermediary Input/Output Flow:
// Input Pose File -> Phi Matrix -> xCOM Values

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>

#include "../gen_phi_matrix.hpp"

#include "../../../18h-Util/convert_pose_formats.hpp"
#include "../../../18h-Util/file_ops.hpp"

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// Function Prototypes
// // Test Beta
Eigen::MatrixXd testBeta(Eigen::MatrixXd betaVectors, Eigen::MatrixXd phiMatrix, Eigen::MatrixXd inputPoses, int numBetas, int bodyParams, string fullRobotPath);

// // Read ideal beta params from robot
Eigen::MatrixXd readIdealBeta(int bodyParams, string fullRobotPath);

// // Set Parameters
SkeletonPtr setParameters(SkeletonPtr robot, Eigen::MatrixXd betaParams, int bodyParams);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (input poses filename)
    string inputPosesFilename = "../random10000anglebalance0.001000tolsafe.txt";
    //string inputPosesFilename = "../rand";
    //string inputPosesFilename = "../hardware-balanced-posesmunzir.txt";

    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(10, -8);

    // INPUT on below line (input beta vector file)
    string inputBetaFilename = "../betaVectors20000.txt";
    //string inputBetaFilename = "../betaVectors1initialBetahardwaretrain-balanced-posesmunzirdart.txt";
    //string inputBetaFilename = "../betaVectors1initialBetahardware-balanced-posesmunzir.txt";
    //string inputBetaFilename = "../betaVectors1initialBetahardware-balanced-posesmunzir2.000000*10e-3filter.txt";

    // INPUT on below line (test with ideal beta)
    //bool testIdeal = true;
    bool testIdeal = false;

    // INPUT on below line (how many betas are converging)
    int numBetas = 500;
    //int numBetas = 1;

    // INPUT on below line (parameters for each body)
    int bodyParams = 4;

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/Krang.urdf";

    // INPUT on below line (output filename)
    string outputBaseName = "testxCOMValues";

    Eigen::MatrixXd inputPoses;
    Eigen::MatrixXd betaVectors;

    try {
        cout << "Reading input poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    try {
        cout << "Reading converged beta ...\n";
        betaVectors = readInputFileAsMatrix(inputBetaFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    if (testIdeal) {
        cout << "Generating ideal beta vector ...\n";
        betaVectors = readIdealBeta(bodyParams, fullRobotPath);
        cout << "|-> Done\n";
    }

    Eigen::MatrixXd phiMatrix = genPhiMatrix(inputPoses, bodyParams, fullRobotPath, perturbedValue);

    cout << "Testing Beta ...\n";
    Eigen::MatrixXd testXCOMValues = testBeta(betaVectors, phiMatrix, inputPoses, numBetas, bodyParams, fullRobotPath);
    cout << "|-> Done\n";

    // Write test xCOM values to file
    string outfilename;
    string inputPosesName = extractFilename(inputPosesFilename);
    string inputBetaName = extractFilename(inputBetaFilename);
    string ext = ".txt";

    outfilename = outputBaseName + inputBetaName + inputPosesName + ext;

    cout << "Writing tested xCOM values to " << outfilename << " ...\n";

    ofstream outfile;
    outfile.open(outfilename);
    outfile << testXCOMValues;
    outfile.close();

    cout << "|-> Done\n";
}

// // Test Beta
Eigen::MatrixXd testBeta(Eigen::MatrixXd betaVectors, Eigen::MatrixXd phiMatrix, Eigen::MatrixXd inputPoses, int numBetas, int bodyParams, string fullRobotPath) {
    int numInputPoses = phiMatrix.rows();
    int numBetaParams = phiMatrix.cols();

    // Make idealRobot a copy of krang model
    DartLoader loader;
    SkeletonPtr idealRobot = loader.parseSkeleton(fullRobotPath);

    // If just wondering (this is the last value)
    //Eigen::MatrixXd beta = betaVectors.block(betaVectors.rows() - numBetas, 0, numBetas, betaVectors.cols());
    Eigen::MatrixXd beta;
    Eigen::MatrixXd phiVec;
    Eigen::MatrixXd xCOMPred(numInputPoses*betaVectors.rows(), numBetas);
    Eigen::MatrixXd xCOMReal(numInputPoses*betaVectors.rows(), 1);

    SkeletonPtr currRobot = idealRobot->clone();
    // TODO: what to do with negative mass values (for now xCOM Real is not
    // actually xCOM Real)
    //currRobot = setParameters(currRobot, beta, bodyParams);

    // Loop through the phi matrix to test the beta
    for (int betaVectorNum = 0; betaVectorNum < betaVectors.rows(); betaVectorNum += numBetas) {
        beta = betaVectors.block(betaVectorNum, 0, numBetas, betaVectors.cols());
        for (int pose = 0; pose < numInputPoses; pose++) {
            phiVec = phiMatrix.row(pose);

            // Add the xCOM to the matrix
            xCOMPred.row(betaVectorNum * numInputPoses + pose) = phiVec * beta.transpose();
            //currRobot->setPositions(inputPoses.row(pose));
            currRobot->setPositions(munzirToDart(inputPoses.row(pose).transpose()));
            xCOMReal(betaVectorNum * numInputPoses + pose, 0) = currRobot->getCOM()(0);

        }
    }

    Eigen::MatrixXd allXCOM(xCOMReal.rows(), xCOMPred.cols() + 1);
    allXCOM << xCOMReal, xCOMPred;

    return allXCOM;
}

// // Read ideal beta from robot
Eigen::MatrixXd readIdealBeta(int bodyParams, string fullRobotPath) {
    // Instantiate ideal robot
    DartLoader loader;
    SkeletonPtr idealRobot = loader.parseSkeleton(fullRobotPath);

    // Create ideal beta
    // Beta Definition/Format
    // mi, mxi, myi, mzi for each body

    int numBodies = idealRobot->getNumBodyNodes();
    BodyNodePtr bodyi;
    double mi;
    double xi;
    double yi;
    double zi;

    Eigen::MatrixXd betaParams(1, numBodies*bodyParams);

    for (int i = 0; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);
        mi = bodyi->getMass();
        xi = bodyi->getLocalCOM()(0);
        yi = bodyi->getLocalCOM()(1);
        zi = bodyi->getLocalCOM()(2);

        betaParams(0, i * bodyParams + 0) = mi;
        betaParams(0, i * bodyParams + 1) = mi*xi;
        betaParams(0, i * bodyParams + 2) = mi*yi;
        betaParams(0, i * bodyParams + 3) = mi*zi;

    }
    return betaParams;
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
