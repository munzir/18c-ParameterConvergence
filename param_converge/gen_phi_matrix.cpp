// Author: Akash Patel (apatel435@gatech.edu)

// genPhiMatrixAsFile
// Purpose: Determine phi vectors for each input pose
//      This phi contains the weights of each parameter at every pose
//
// Input: Ideal beta={mi, MXi, MYi, MZi, ...}, krang urdf model, perturbation value, data points (q/poses) as a file,
// Output: Phi matrix as a file

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include "gen_phi_matrix.hpp"

#include "../../18h-Util/convert_pose_formats.hpp"

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// // Generate Phi Matrix
Eigen::MatrixXd genPhiMatrix(Eigen::MatrixXd inputPoses, int bodyParams, string fullRobotPath, double perturbedValue) {
    int numInputPoses = inputPoses.rows();
    int numParams = inputPoses.cols();

    // Instantiate ideal robot
    cout << "Creating ideal beta vector, robot array, and perturbing robots ...\n";
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
    int numPertRobots = numBodies*bodyParams;

    // Forward perturbed robot array
    SkeletonPtr forwPertRobotArray[numPertRobots];
    for (int i = 0; i < numPertRobots; i++) {
        forwPertRobotArray[i] = idealRobot->clone();
    }

    // Backward perturbed robot array
    SkeletonPtr backPertRobotArray[numPertRobots];
    for (int i = 0; i < numPertRobots; i++) {
        backPertRobotArray[i] = idealRobot->clone();
    }

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

        // Note that beta has mass times CoM but the perturbation is only in CoM
        // so the phi should divide the derivative by mi to make it consistent with our choice of beta
        forwPertRobotArray[i * bodyParams + 0]->getBodyNode(i)->setMass(mi + perturbedValue);
        forwPertRobotArray[i * bodyParams + 1]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi + perturbedValue, yi, zi));
        forwPertRobotArray[i * bodyParams + 2]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi + perturbedValue, zi));
        forwPertRobotArray[i * bodyParams + 3]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi, zi + perturbedValue));

        backPertRobotArray[i * bodyParams + 0]->getBodyNode(i)->setMass(mi - perturbedValue);
        backPertRobotArray[i * bodyParams + 1]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi - perturbedValue, yi, zi));
        backPertRobotArray[i * bodyParams + 2]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi - perturbedValue, zi));
        backPertRobotArray[i * bodyParams + 3]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi, zi - perturbedValue));

    }

    cout << "|-> Done\n";

    cout << "Generating Phi Matrix ...\n";

    // Find phiMatrix
    Eigen::MatrixXd phiMatrix(numInputPoses, numPertRobots);
    Eigen::VectorXd phi(bodyParams);
    double xMCOMIdealRobot;
    double xMCOMForwPertRobot;
    double xMCOMBackPertRobot;

    // Loop through all the input poses
    for (int pose = 0; pose < numInputPoses; pose++) {

        // Set position of ideal robot to the pose in DART format
        //idealRobot->setPositions(inputPoses.row(pose));
        idealRobot->setPositions(munzirToDart(inputPoses.row(pose)));

        // Get x center of mass
        xMCOMIdealRobot = idealRobot->getMass()*idealRobot->getCOM()(0);

        for (int b = 0; b < numBodies; b++) {

            for(int i=0; i<bodyParams; i++){

                // Set perturbed robot position to pose
                //forwPertRobotArray[b*bodyParams + i]->setPositions(inputPoses.row(pose));
                forwPertRobotArray[b*bodyParams + i]->setPositions(munzirToDart(inputPoses.row(pose)));
                //backPertRobotArray[b*bodyParams + i]->setPositions(inputPoses.row(pose));
                backPertRobotArray[b*bodyParams + i]->setPositions(munzirToDart(inputPoses.row(pose)));

                // Get the center of mass of the perturbedRobot
                xMCOMForwPertRobot = forwPertRobotArray[b*bodyParams + i]->getMass()*forwPertRobotArray[b*bodyParams + i]->getCOM()(0);
                xMCOMBackPertRobot = backPertRobotArray[b*bodyParams + i]->getMass()*backPertRobotArray[b*bodyParams + i]->getCOM()(0);

                // Calculate phi for betai and pose
                phi(i) = (xMCOMForwPertRobot - xMCOMBackPertRobot)/(2*perturbedValue);
                // phi(i) = (xCOMForwPertRobot - xCOMIdealRobot)/(perturbedValue);
            }

            // rhs = a*m + b*m*x + c*m*y + d*m*z
            // d_m = a + b*x + c*y + d*z
            // [d_x, d_y, d_z] = [b*m, c*m, d*m]
            // [b, c, d] = [d_x, d_y, d_z]/m
            // a = d_m - b*x - c*y - d*z

            bodyi = idealRobot->getBodyNode(b);
            mi = bodyi->getMass();
            phi.tail(3) = phi.tail(3)/mi;
            phi(0) = phi(0) - phi.tail(3).transpose()*bodyi->getLocalCOM();

            // Add phi to phiMatrix and then print it looks cleaner
            phiMatrix.block(pose, b*bodyParams, 1, bodyParams) = phi.transpose();
        }

    }

    phiMatrix = phiMatrix/idealRobot->getMass();

    cout << "|-> Done\n";

    // Open output file to write phi matrix
    ofstream phiFile;
    phiFile.open("phiMatrix.txt");

    // phi should be the same no matter estimation/perturbation
    phiFile << phiMatrix;
    phiFile.close();

    return phiMatrix;
}
