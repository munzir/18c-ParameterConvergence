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

// // Structs
struct trainBetaRetVal {
    Eigen::MatrixXd retFilteredPoses;
    Eigen::MatrixXd retBetaVectors;
    Eigen::MatrixXd retXCOMVectors;
    Eigen::MatrixXd retTotalMassVectors;
};

// Function Prototypes
// // Converge to Beta
trainBetaRetVal trainBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, Eigen::MatrixXd priorBetas, int bodyParams, string fullRobotPath, double learningRate, double massRegularization, double threshold, int convergedPoses);

// // Create a prior beta
Eigen::MatrixXd createPriorBeta(string fullRobotPath, int bodyParams, double minXCOMError, double maxDeviation, double maxOffset, Eigen::MatrixXd initialPosePhiVec, int numRandomBetas);

// // Change robot's beta values (parameters)
SkeletonPtr setParameters(SkeletonPtr robot, Eigen::MatrixXd beta, int bodyParams);

// // Check for convergence
bool xtolReached(double threshold, Eigen::MatrixXd xCOMAllVectors, int pose, int convergedPoses);

// // Random Value
double fRand(double fMin, double fMax);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (Random Seed)
    srand(time(0));
    //srand(0);

    // INPUT on below line (input poses filename)
    string inputPosesFilename = "../random22106fullbalance0.001000tolsafe.txt";

    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(10, -8);

    // INPUT on below line (number of parameters for each body)
    int bodyParams = 4;

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/Krang.urdf";

    // INPUT on below line (number of random initial betas)
    int numRandomBetas = 500;

    // INPUT on below lines (need to create a prior beta value)
    double minXCOMError = 0.02;
    double maxDeviation = 0.50;
    double maxOffset = 0.50;

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

    // INPUT on below line (threshold to filter out)
    // threshold = mag * 10^exp
    double mag = 2;
    int exp = -3;
    double threshold = mag * pow(10, exp);

    // INPUT on below line (how many poses to check for convergence)
    int convergedPoses = 10;

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

    // TODO: Maybe add the greatest xcom to check for initial error?
    Eigen::MatrixXd initialPosePhiVec = phiMatrix.row(0);
    Eigen::MatrixXd priorBetas = createPriorBeta(fullRobotPath, bodyParams, minXCOMError, maxDeviation, maxOffset, initialPosePhiVec, numRandomBetas);

    cout << "Converging to Beta ...\n";
    trainBetaRetVal retVal = trainBeta(inputName, inputPoses, phiMatrix, priorBetas, bodyParams, fullRobotPath, learningRate, massRegularization, threshold, convergedPoses);
    cout << "\n|-> Done\n";

    Eigen::MatrixXd filteredPoses = retVal.retFilteredPoses;
    Eigen::MatrixXd betaVectors = retVal.retBetaVectors;
    Eigen::MatrixXd xCOMVectors = retVal.retXCOMVectors;
    Eigen::MatrixXd totalMassVectors = retVal.retTotalMassVectors;


    // Base outout filename
    string ext = ".txt";
    string outBaseFilename = to_string(numRandomBetas) + "initialBeta" + inputName + to_string(mag) + "*10e" + to_string(exp) + "filter" + ext;

    cout << "Writing beta vectors to " << outBaseFilename << " ...\n";

    // Open output file to write poses used
    ofstream filteredPosesFile;
    string filteredPosesFilename = "filteredPoses" + outBaseFilename;
    filteredPosesFile.open(filteredPosesFilename);
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

    filteredPosesFile << filteredPoses << endl;
    betaVectorsFile << betaVectors << endl;
    xCOMAllVectorsFile << xCOMVectors << endl;
    totalMassFile << totalMassVectors << endl;

    betaVectorsFile.close();
    filteredPosesFile.close();
    xCOMAllVectorsFile.close();
    totalMassFile.close();

    cout << "|-> Done\n";

    return 0;
}

// // Converge to Beta
// TODO: Add total mass constraint
trainBetaRetVal trainBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, Eigen::MatrixXd priorBetas, int bodyParams, string fullRobotPath, double n, double u, double threshold, int convergedPoses) {
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

    Eigen::MatrixXd filteredPoses(numInputPoses, inputPoses.cols());

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
    while (pose < numInputPoses) {
        // Max xCOM so biggest learning is always done first
        phiVec = phiMatrix.row(0);
        double maxXCOMValue = 0;
        int maxPose = 0;

        // Loop through remaining poses to determine the one that provides
        // greatest learning
        int i = 0;
        while (i < phiMatrix.rows()) {
            cout << "\rPose: " << pose + 1 << "/" << numInputPoses << " Finding Max: " << i << "/" << phiMatrix.rows() << " \t " << endl;
            phiVec = phiMatrix.row(i);
            xCOMValue = ((phiVec * currBeta.transpose()).cwiseAbs()).sum();
            if (xCOMValue >= maxXCOMValue) {
                maxXCOMValue = xCOMValue;
                maxPose = i;
            }
            i++;
        }
        phiVec = phiMatrix.row(maxPose);

        // Write this pose
        filteredPoses.row(pose) = inputPoses.row(maxPose);

        xCOMVector = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - idealTotalMassVector));

        // Append the next xCOM
        currRobot->setPositions(inputPoses.row(maxPose));
        xCOMVectorWithReal(0, 0) = currRobot->getCOM()(0);
        for (int i = 0; i < xCOMVectorWithReal.cols(); i++) {
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


        // Determine the min value of all the xCOMs of each currBeta
        if (!xtolReached(threshold, xCOMAllVectors, pose, convergedPoses)) {
            // Remove the maxPose from phiMatrix and inputPoses, since it is
            // already used (don't want to double count a single pose)
            Eigen::MatrixXd phiMatrixTmp(phiMatrix.rows() - 1, phiMatrix.cols());
            Eigen::MatrixXd inputPosesTmp(inputPoses.rows() - 1, inputPoses.cols());

            if (maxPose == 0) {
                phiMatrixTmp = phiMatrix.bottomRows(phiMatrix.rows() - 1);
                inputPosesTmp = inputPoses.bottomRows(inputPoses.rows() - 1);
            } else {
                phiMatrixTmp.topRows(maxPose) = phiMatrix.topRows(maxPose);
                phiMatrixTmp.bottomRows(phiMatrixTmp.rows() - maxPose) = phiMatrix.bottomRows(phiMatrix.rows() - 1 - maxPose);
                inputPosesTmp.topRows(maxPose) = inputPoses.topRows(maxPose);
                inputPosesTmp.bottomRows(inputPosesTmp.rows() - maxPose) = inputPoses.bottomRows(inputPoses.rows() - 1 - maxPose);
            }
            phiMatrix = phiMatrixTmp;
            inputPoses = inputPosesTmp;
            pose++;
        } else {
            pose++;
            break;
        }
    }
    cout << "\rPose: " << pose << "/" << numInputPoses << " Finding Max: " << phiMatrix.rows() << "/" << phiMatrix.rows() << " \t ";

    Eigen::MatrixXd filteredPosesFin = filteredPoses.topRows(pose);

    int betaVectorsTopRows = (pose + 1) * numBetas;
    Eigen::MatrixXd betaVectorsFin = betaVectors.topRows(betaVectorsTopRows);

    // Write the xCOM value of the last beta and the last pose
    // Same with the total mass
    xCOMVector = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - idealTotalMassVector));
    xCOMVectorWithReal(0, 0) = currRobot->getCOM()(0);
    for (int i = 0; i < xCOMVectorWithReal.cols(); i++) {
        xCOMVectorWithReal(0, i) = xCOMVector(0, i - 1);
    }
    xCOMAllVectors.row(pose) = xCOMVector;
    Eigen::MatrixXd xCOMAllVectorsFin = xCOMAllVectors.topRows(pose + 1);

    totalMassVector = (massIndicatorMatrix * currBeta.transpose());
    totalMassAllVectors.row(pose) = totalMassVector;
    Eigen::MatrixXd totalMassAllVectorsFin = totalMassAllVectors.topRows(pose + 1);

    trainBetaRetVal retVal = {.retFilteredPoses = filteredPosesFin, .retBetaVectors = betaVectorsFin, .retXCOMVectors = xCOMAllVectorsFin, .retTotalMassVectors = totalMassAllVectorsFin};

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
        while (abs(xCOM) < minXCOMError) {
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

// // Check for convergence
bool xtolReached(double threshold, Eigen::MatrixXd xCOMAllVectors, int pose, int convergedPoses) {

    if (pose + 1 < convergedPoses) {
        return false;
    }

    for (int i = 0; i < convergedPoses; i++) {
        Eigen::MatrixXd xCOMVector = xCOMAllVectors.row(pose + 1 - convergedPoses + i);
        double minxCOM = xCOMVector.minCoeff();
        double maxxCOM = xCOMVector.maxCoeff();
        double absMaxxCOM = maxxCOM;
        if (abs(minxCOM) > maxxCOM) {
            absMaxxCOM = abs(minxCOM);
        }
        if (absMaxxCOM >= threshold) {
            return false;
        }
    }

    return true;

}

// // Random Value
double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
