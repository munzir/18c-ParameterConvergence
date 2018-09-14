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

#include "../../../18h-Util/convert_pose_formats.hpp"
#include "../../../18h-Util/file_ops.hpp"
#include "../../../18h-Util/random.hpp"

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// // Structs
struct trainBetaRetVal {
    Eigen::MatrixXd retBetaVectors;
    Eigen::MatrixXd retXCOMVectors;
    Eigen::MatrixXd retTotalMassVectors;
};

// Function Prototypes
// // Converge to Beta
trainBetaRetVal trainBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, Eigen::MatrixXd priorBetas, int bodyParams, string fullRobotPath, double learningRate, double massRegularization);

// // Create a prior beta
Eigen::MatrixXd createPriorBeta(string fullRobotPath, int bodyParams, double minXCOMError, double maxDeviation, double maxOffset, Eigen::MatrixXd initialPosePhiVec, int numRandomBetas);

// // Change robot's beta values (parameters)
SkeletonPtr setParameters(SkeletonPtr robot, Eigen::MatrixXd beta, int bodyParams);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (Random Seed)
    double startTime = time(0);
    srand(startTime);
    //srand(0);

    // INPUT on below line (number of random initial betas)
    //int numRandomBetas = 500;
    int numRandomBetas = 1;

    // INPUT on below line (input poses filename)
    //string inputPosesFilename = "../filteredPoses500initialBetarandom22106fullbalance0.001000tolsafe2.000000*10e-3filter.txt";
    //string inputPosesFilename = "../filPoses.txt";
    //string inputPosesFilename = "../hardwaretrain-balanced-posesmunzirdart.txt";
    //string inputPosesFilename = "../hardware-balanced-posesmunzir.txt";
    //string inputPosesFilename = "../random20000anglebalance0.001000tolsafe.txt";
    string inputPosesFilename = "../hardware236.txt";

    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(10, -8);

    // INPUT on below line (number of parameters for each body)
    int bodyParams = 4;

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/Krang.urdf";

    // INPUT on below lines (need to create a prior beta value)
    //double minXCOMError = 0.02;
    //double maxDeviation = 0.50;
    //double maxOffset = 0.50;
    double minXCOMError = 0.00;
    double maxDeviation = 0.00;
    double maxOffset = 0.00;

    //Best so far with u = 0 n = 300 would need to do comparisons of
    // INPUT on below line (learning rate)
    double learningRate = 300;
    // optimal value between 200 & 300
    // Determined by using a threshold and whichever learning rate gives us
    // least number of filtered poses
    //double learningRate = 0.05;

    // Regularizes the importance of the masses with respect to the moments
    // INPUT on below line (mass coefficient for regularization)
    double massRegularization = 0.00;

    string inputName = extractFilename(inputPosesFilename);

    Eigen::MatrixXd inputPoses;
    try {
        cout << "Reading Input Poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    Eigen::MatrixXd phiMatrix = genPhiMatrix(inputPoses, bodyParams, fullRobotPath, perturbedValue);

    // TODO: Maybe add the greatest xcom to check for initial error? instead of
    // basing it off the first one
    Eigen::MatrixXd initialPosePhiVec = phiMatrix.row(0);
    Eigen::MatrixXd priorBetas = createPriorBeta(fullRobotPath, bodyParams, minXCOMError, maxDeviation, maxOffset, initialPosePhiVec, numRandomBetas);

    cout << "Converging to Beta ...\n";
    trainBetaRetVal retVal = trainBeta(inputName, inputPoses, phiMatrix, priorBetas, bodyParams, fullRobotPath, learningRate, massRegularization);
    cout << "\n|-> Done\n";

    Eigen::MatrixXd betaVectors = retVal.retBetaVectors;
    Eigen::MatrixXd xCOMVectors = retVal.retXCOMVectors;
    Eigen::MatrixXd totalMassVectors = retVal.retTotalMassVectors;


    // Base outout filename
    string ext = ".txt";
    //string outBaseFilename = to_string(numRandomBetas) + "initialBeta" + to_string(startTime) + inputName + ext;
    string outBaseFilename = to_string(numRandomBetas) + "initialBeta" + inputName + ext;

    cout << "Writing beta vectors to " << outBaseFilename << " ...\n";

    // Open output file to write beta vectors
    ofstream betaVectorsFile;
    string betaVectorsFilename = "betaVectors" + outBaseFilename;
    betaVectorsFile.open(betaVectorsFilename);
    // Open output file to write xCOM values
    ofstream xCOMAllVectorsFile;
    string xCOMAllVectorsFilename = "xCOMAllVectors" + outBaseFilename;
    xCOMAllVectorsFile.open(xCOMAllVectorsFilename);
    // Open output file to write total mass values
    ofstream totalMassFile;
    string totalMassFilename = "totalMassValues" + outBaseFilename;
    totalMassFile.open(totalMassFilename);

    betaVectorsFile << betaVectors << endl;
    xCOMAllVectorsFile << xCOMVectors << endl;
    totalMassFile << totalMassVectors << endl;

    betaVectorsFile.close();
    xCOMAllVectorsFile.close();
    totalMassFile.close();

    cout << "|-> Done\n";

    return 0;
}

// // Converge to Beta
// TODO: Add total mass constraint
trainBetaRetVal trainBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, Eigen::MatrixXd priorBetas, int bodyParams, string fullRobotPath, double n, double u) {
    int numInputPoses = phiMatrix.rows();
    int numBetas = priorBetas.rows();
    int numBetaParams = priorBetas.cols();

    // Make idealRobot a copy of krang model
    DartLoader loader;
    SkeletonPtr idealRobot = loader.parseSkeleton(fullRobotPath);

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
    Eigen::MatrixXd betaVectors((numInputPoses + 1) * numBetas, numBetaParams);
    betaVectors.block(0, 0, numBetas, numBetaParams) = priorBetas;
    Eigen::MatrixXd currBeta = betaVectors.block(0, 0, numBetas, numBetaParams);

    Eigen::MatrixXd phiVec(1, numBetaParams);
    Eigen::MatrixXd delta(1, numBetaParams);

    Eigen::MatrixXd xCOMVector(1, currBeta.rows());
    Eigen::MatrixXd xCOMVectorWithReal(1, 1 + xCOMVector.cols());
    Eigen::MatrixXd xCOMAllVectors((numInputPoses + 1), 1 + xCOMVector.cols());
    double xCOMValue;

    Eigen::MatrixXd totalMassVector(1, currBeta.rows());
    Eigen::MatrixXd totalMassAllVectors((numInputPoses + 1), priorBetas.rows());
    Eigen::MatrixXd idealTotalMass(1, 1);
    idealTotalMass << idealRobot->getMass();
    Eigen::MatrixXd idealTotalMassVector = idealTotalMass * Eigen::MatrixXd::Ones(1, currBeta.rows());

    SkeletonPtr currRobot = idealRobot->clone();

    // Loop through the phi matrix to calculate the beta vectors
    cout << "Pose: 0" << "/" << numInputPoses;
    int pose = 0;
    for (; pose < numInputPoses; pose++) {
        cout << "\rPose: " << pose + 1 << "/" << numInputPoses << " \t ";

        phiVec = phiMatrix.row(pose);

        xCOMVector = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - idealTotalMassVector));

        // Append the next xCOM
        // TODO: Error setting pose
        //currRobot->setPositions(inputPoses.row(pose));
        currRobot->setPositions(munzirToDart(inputPoses.row(pose)));
        xCOMVectorWithReal(0, 0) = currRobot->getCOM()(0);
        for (int i = 1; i < xCOMVectorWithReal.cols(); i++) {
            xCOMVectorWithReal(0, i) = xCOMVector(0, i - 1);
        }
        xCOMAllVectors.row(pose) = xCOMVectorWithReal;

        totalMassVector = (massIndicatorMatrix * currBeta.transpose());
        totalMassAllVectors.row(pose) = totalMassVector;

        delta = phiVec + (u * massIndicatorMatrix);

        // Calculating with a 2D currBeta
        // Is there a way to create currBeta.rows() for a single vector
        // (delta)
        // A workaround
        for (int i = 0; i < currBeta.rows(); i++) {
            currBeta.row(i) = currBeta.row(i) - (n * (xCOMVector.col(i) * delta));
        }

        // Plus one to pose because we still have our initial betas
        //betaVectors.block((pose + 1) * numBetas - 1, 0, numBetas, numBetaParams) = currBeta;
        betaVectors.block((pose + 1) * numBetas, 0, numBetas, numBetaParams) = currBeta;

    }

    // Write the xCOM value of the last beta and the last pose
    // Same with the total mass
    xCOMVector = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - idealTotalMassVector));
    xCOMVectorWithReal(0, 0) = currRobot->getCOM()(0);
    for (int i = 1; i < xCOMVectorWithReal.cols(); i++) {
        xCOMVectorWithReal(0, i) = xCOMVector(0, i - 1);
    }
    xCOMAllVectors.row(pose) = xCOMVectorWithReal;

    totalMassVector = (massIndicatorMatrix * currBeta.transpose());
    totalMassAllVectors.row(pose) = totalMassVector;

    trainBetaRetVal retVal = {.retBetaVectors = betaVectors, .retXCOMVectors = xCOMAllVectors, .retTotalMassVectors = totalMassAllVectors};

    return retVal;
}

// // Create a prior beta
// TODO: Need to create betas with only one value perturbed such that
// minXCOMError is achieved
Eigen::MatrixXd createPriorBeta(string fullRobotPath, int bodyParams, double minXCOMError, double maxDeviation, double maxOffset, Eigen::MatrixXd initialPosePhiVec, int numRandomBetas) {
    // Make idealRobot a copy of krang model
    DartLoader loader;
    SkeletonPtr robot = loader.parseSkeleton(fullRobotPath);

    // Beta Definition/Format
    // mi, mxi, myi, mzi for each body
    int numBodies = robot->getNumBodyNodes();
    BodyNodePtr bodyi;
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

    Eigen::MatrixXd priorBetas(numRandomBetas, numBodies*bodyParams);
    double xCOM;

    for (int currRandomBeta = 0; currRandomBeta < numRandomBetas; currRandomBeta++) {
        // Reset priorBeta and xCOM
        Eigen::MatrixXd priorBeta(1, numBodies*bodyParams);
        xCOM = 0;
        while (abs(xCOM) <= minXCOMError) {
            for (int i = 0; i < numBodies; i++) {
                bodyi = robot->getBodyNode(i);
                mi = bodyi->getMass();
                mxi = mi * bodyi->getLocalCOM()(0);
                myi = mi * bodyi->getLocalCOM()(1);
                mzi = mi * bodyi->getLocalCOM()(2);

                priorBeta(0, i * bodyParams + 0) = mi;

                deviation = fRand(-maxDeviation, maxDeviation);
                offset = fRand(-maxOffset, maxOffset);
                priorBeta(0, i * bodyParams + 1) = mxi + deviation * mxi + offset;

                deviation = fRand(-maxDeviation, maxDeviation);
                offset = fRand(-maxOffset, maxOffset);
                priorBeta(0, i * bodyParams + 2) = myi + deviation * myi + offset;

                deviation = fRand(-maxDeviation, maxDeviation);
                offset = fRand(-maxOffset, maxOffset);
                priorBeta(0, i * bodyParams + 3) = mzi + deviation * mzi + offset;
            }
            xCOM = (initialPosePhiVec * priorBeta.transpose())(0, 0);
        }
        priorBetas.row(currRandomBeta) = priorBeta;
    }
    return priorBetas;
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
