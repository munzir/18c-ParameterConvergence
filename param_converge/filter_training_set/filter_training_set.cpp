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
Eigen::MatrixXd trainBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, Eigen::MatrixXd priorBetas, int bodyParams, string fullRobotPath, int eons, double learningRate, double massRegularization, double mag, int exp, int numRandomBetas);

// // Create a prior beta
Eigen::MatrixXd createPriorBeta(string fullRobotPath, int bodyParams, double minXCOMError, double maxDeviation, double maxOffset, Eigen::MatrixXd initialPosePhiVec, int numRandomBetas);

// // Change robot's beta values (parameters)
SkeletonPtr setParameters(SkeletonPtr robot, Eigen::MatrixXd beta, int bodyParams);

// // Random Value
double fRand(double fMin, double fMax);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (Random Seed)
    //srand(time(0));
    srand(0);

    // INPUT on below line (input poses filename)
    string inputPosesFilename = "../random6003fullbalance0.001000tolsafe.txt";

    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(10, -8);

    // INPUT on below line (number of parameters for each body)
    int bodyParams = 4;

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/Krang.urdf";

    // INPUT on below lines (need to create a prior beta value)
    double minXCOMError = 0.02;
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
    double massRegularization = 0.00;

    // INPUT on below line (threshold to filter out)
    // threshold = mag * 10^exp
    double mag = 2;
    int exp = -3;

    // INPUT on below line (number of random initial betas)
    int numRandomBetas = 500;

    string inputName = extractFilename(inputPosesFilename);

    Eigen::MatrixXd inputPoses;
    try {
        cout << "Reading Input Poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
    }

    Eigen::MatrixXd phiMatrix = genPhiMatrix(inputPoses, bodyParams, fullRobotPath, perturbedValue);

    Eigen::MatrixXd initialPosePhiVec = phiMatrix.row(0);
    Eigen::MatrixXd priorBetas = createPriorBeta(fullRobotPath, bodyParams, minXCOMError, maxDeviation, maxOffset, initialPosePhiVec, numRandomBetas);

    cout << "Converging to Beta ...\n";
    Eigen::MatrixXd betaVectors = trainBeta(inputName, inputPoses, phiMatrix, priorBetas, bodyParams, fullRobotPath, eons, learningRate, massRegularization, mag, exp, numRandomBetas);
    cout << "\n|-> Done\n";


    // Write betaVectors to file
    string outfilename;
    string ext = ".txt";
    string outputBaseName = "betaVectors";

    outfilename = outputBaseName + to_string(numRandomBetas) + "initialBeta" + inputName + to_string(mag) + "*10e" + to_string(exp) + "filter" + ext;

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
Eigen::MatrixXd trainBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, Eigen::MatrixXd priorBetas, int bodyParams, string fullRobotPath, int eons, double n, double u, double mag, int exp, int numRandomBetas) {
    int numInputPoses = phiMatrix.rows();
    int numBetaParams = phiMatrix.cols();

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
    Eigen::MatrixXd betaVectors = priorBetas.row(0);
    Eigen::MatrixXd currBeta = priorBetas;

    // TODO: Not assigning values inside the matrix properly for xCOM and
    // totalMass potential fix in test_beta
    Eigen::MatrixXd phiVec(1, numBetaParams);
    Eigen::MatrixXd delta(1, numBetaParams);

    Eigen::MatrixXd xCOM(1, 1);
    Eigen::MatrixXd xCOMValueVector(1, priorBetas.rows());
    Eigen::MatrixXd xCOMValues((eons*numInputPoses)+1,1);
    double xCOMValue;
    Eigen::MatrixXd xCOMVector(1, currBeta.rows());

    Eigen::MatrixXd totalMassValues((eons*numInputPoses)+1,1);
    double totalMass;
    Eigen::MatrixXd totalMassVector(1, currBeta.rows());
    Eigen::MatrixXd idealTotalMass(1, 1);
    idealTotalMass << idealRobot->getMass();
    Eigen::MatrixXd idealTotalMassVector = idealTotalMass * Eigen::MatrixXd::Ones(1, currBeta.rows());

    SkeletonPtr currRobot = idealRobot->clone();

    // Base outout filename
    string ext = ".txt";
    string outBaseFilename = to_string(numRandomBetas) + "initialBeta" + inputName + to_string(mag) + "*10e" + to_string(exp) + "filter" + ext;
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

    // Threshold that signifies if new data is not learning much
    // TODO: This favors learning on parameters that heavily change xcom (i
    // think it should be fine tho: the lower we go the more we compensate for
    // it)
    double threshold = mag * std::pow(10, exp);

    int finPose = 0;
    // Loop through eons
    for (int k = 0; k < eons; k++) {
    // Loop through the phi matrix to calculate the beta vectors
    cout << "Pose: 0" << "/" << numInputPoses;
    // TODO: Change stopping condition to the filtering if statement threshold
    // TODO: Put absMaxxCOM calculation after learning in the loop and do an
    // initial maxXCOMValue calculation before loop starts in order to not
    // repeat the while loop condition once
    double absMaxxCOM = threshold;
    while (abs(absMaxxCOM) >= threshold) {
    //for (int pose = 0; pose < numInputPoses; pose++) {
        // Max xCOM so biggest learning is always done first
        phiVec = phiMatrix.row(0);
        //double maxXCOMValue = (phiVec * currBeta.transpose())(0, 0);
        double maxXCOMValue = ((phiVec * currBeta.transpose()).cwiseAbs()).sum();
        int maxPose = 0;

        // Loop through remaining poses to determine the one that provides
        // greatest learning
        int i = 0;
        while (i < phiMatrix.rows()) {
            cout << "\rPose: " << finPose + 1 << "/" << numInputPoses << " Finding Max: " << i << "/" << phiMatrix.rows() << " \t ";
            phiVec = phiMatrix.row(i);
            xCOMValue = ((phiVec * currBeta.transpose()).cwiseAbs()).sum();
            if (xCOMValue > maxXCOMValue) {
                maxXCOMValue = xCOMValue;
                maxPose = i;
            }
            i++;
        }
        phiVec = phiMatrix.row(maxPose);

        // Determine the min value of all the xCOMs of each currBeta
        xCOMValueVector = phiVec * currBeta.transpose();
        double minxCOM = xCOMValueVector.minCoeff();
        double maxxCOM = xCOMValueVector.maxCoeff();
        absMaxxCOM = maxxCOM;
        if (abs(minxCOM) > maxxCOM) {
            absMaxxCOM = minxCOM;
        }
        // currRobot = setParameters(idealRobot, currBeta, bodyParams);
        // xCOMValue = currRobot->getCOM()(0);

        // Filter out poses that do not change the xCOM value much
        if (abs(absMaxxCOM) >= threshold) {

            // Write this pose
            filteredPosesFile << inputPoses.row(maxPose) << endl;

            // Just calculating for the first beta
            //xCOM = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - idealTotalMass));

            xCOMVector = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - idealTotalMassVector));

            // Append the next xCOM
            //xCOMValues(k*pose, 0) = xCOMValue;
            //xCOMValuesFile << xCOMValue << endl;
            // For multiple betas
            xCOMValuesFile << xCOMVector << endl;

            //totalMass = (massIndicatorMatrix * currBeta.transpose())(0, 0);
            //totalMassValues(k*finPose, 0) = totalMass;
            //totalMassFile << totalMass << endl;
            // For mutilple betas
            totalMassVector = (massIndicatorMatrix * currBeta.transpose());
            totalMassFile << totalMassVector << endl;

            delta = phiVec + (u * massIndicatorMatrix);
            // Update currBeta parameter vector
            //currBeta = currBeta - n * delta;
            //currBeta = currBeta - n * xcom;
            //cout << xCOM << endl;

            // Just calculating for one beta
            //currBeta = currBeta - (n * (xCOM * delta));

            // Calculating with a 2D currBeta
            // Is there a way to create currBeta.rows() for a single vector
            // (delta)
            // A workaround
            for (int i = 0; i < currBeta.rows(); i++) {
                currBeta.row(i) = currBeta.row(i) - (n * (xCOMVector.col(i) * delta));
            }

            betaVectorsFile << currBeta << endl;

            // Append the updated beta vector
            //Eigen::MatrixXd betaTmp(betaVectors.rows()+currBeta.rows(), currBeta.cols());
            //betaTmp << betaVectors,
                       //currBeta;
            //betaVectors = betaTmp;

            // Remove the maxPose from phiMatrix and inputPoses, since it is
            // already used (don't want to double count a single pose)
            // TODO
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
        }
        finPose++;
    }
    }
    cout << "\rPose: " << finPose << "/" << numInputPoses;

    betaVectorsFile.close();
    filteredPosesFile.close();

    // Write the xCOM value of the last beta and the last pose
    // Same with the total mass
    //xCOMValue = (phiVec * currBeta.transpose())(0, 0);
    // For multiple betas
    xCOMVector = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - idealTotalMassVector));
    xCOMValuesFile << xCOMVector << endl;
    // currRobot = setParameters(idealRobot, currBeta, bodyParams);
    // xCOMValue = currRobot->getCOM()(0);
    // xCOMValues(eons*numInputPoses, 0) = xCOMValue;
    //xCOMValuesFile << xCOMValue << endl;

    // Open output file to write xCOM values
    // ofstream xCOMValuesFile;
    // xCOMValuesFile.open("xCOMValues.txt");
    // xCOMValuesFile << xCOMValues;
    xCOMValuesFile.close();

    // totalMass = (massIndicatorMatrix * currBeta.transpose())(0, 0);
    // totalMassValues(eons*numInputPoses, 0) = totalMass;
    // totalMassFile << totalMass << endl;
    // For multiple betas
    totalMassVector = (massIndicatorMatrix * currBeta.transpose());
    totalMassFile << totalMassVector << endl;

    // Open output file to write total mass values
    // ofstream totalMassFile;
    // totalMassFile.open("totalMassValues.txt");
    // totalMassFile << totalMassValues;
    totalMassFile.close();

    return betaVectors;
}

// // Create a prior beta
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

// // Random Value
double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
