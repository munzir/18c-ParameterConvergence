// Author: Akash Patel (apatel435@gatech.edu)

// genPhiMatrixAsFile
// Purpose: Determine phi vectors for each input pose
//   This phi will be used for finding beta (weights of actual robot) via gradient descent
//
// Input: Ideal beta={mi, MXi, MYi, MZi, ...}, krang urdf model, perturbation value, data points (q/poses) as a file,
// Output: Phi matrix as a file

// convergeToBeta
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

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// Defines
#define MAXBUFSIZE ((int) 1e6)

// Function Prototypes
// // Generate Phi Matrix
Eigen::MatrixXd genPhiMatrix(Eigen::MatrixXd inputPoses, string fullRobotPath, double perturbedValue);

// // Converge to Beta
Eigen::MatrixXd convergeToBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, string fullRobotPath, double maxDeviation, double maxOffset, int eons, double learningRate, double massRegularization, double suitableError, int suitableNumPoses, double mag, int exp);

// // Change robot's beta values (parameters)
SkeletonPtr setParameters(SkeletonPtr robot, Eigen::MatrixXd beta, int bodyParams);

// // Random Value
double fRand(double fMin, double fMax);

// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename);

// // Extract filename
string extractFilename(string filename);

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
    double perturbedValue = std::pow(10, -3);

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf";

    // INPUT on below lines (need to create a prior beta value aka betaHat)
    double maxDeviation = 0.00;
    double maxOffset = 0.00;

    // Eons (how many times to learn on same dataset)
    // INPUT on below line (eons)
    int eons = 1;

    // learning rate not converging fast enough with 1900, 1700, 1500, 1300 and values below 500
    //Best so far with u = 0 n = 1100.0/900.0 would need to do comparisons of
    // INPUT on below line (learning rate)
    double learningRate = 1000;
    //double learningRate = 0.05;

    // Regularizes the importance of the masses with respect to the moments
    // INPUT on below line (mass coefficient for regularization)
    double massRegularization = 0.0;

    // INPUT on below line (satisfactory error value and poses averaged over)
    double suitableError = 0.002;
    int suitableNumPoses = 100;

    // INPUT on below line (threshold to filter out)
    // threshold = mag * 10^exp
    double mag = 0;
    int exp = -3;

    string inputName = extractFilename(inputPosesFilename);

    cout << "Reading Input Poses ...\n";
    Eigen::MatrixXd inputPoses = readInputFileAsMatrix(inputPosesFilename);
    cout << "|-> Done\n";

    Eigen::MatrixXd phiMatrix = genPhiMatrix(inputPoses, fullRobotPath, perturbedValue);

    cout << "Converging to Beta ...\n";
    Eigen::MatrixXd betaVectors = convergeToBeta(inputName, inputPoses, phiMatrix, fullRobotPath, maxDeviation, maxOffset, eons, learningRate, massRegularization, suitableError, suitableNumPoses, mag, exp);
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

// // Generate Phi Matrix
Eigen::MatrixXd genPhiMatrix(Eigen::MatrixXd inputPoses, string fullRobotPath, double perturbedValue) {
    int numInputPoses = inputPoses.rows();
    int numParams = inputPoses.cols();

    // Instantiate ideal robot
    cout << "Creating ideal beta vector, robot array, and perturbing robots ...\n";
    DartLoader loader;
    SkeletonPtr idealRobot = loader.parseSkeleton(fullRobotPath);

    // Create ideal beta
    // Beta Definition/Format
    // mi, mxi, myi, mzi for each body

    int bodyParams = 4;
    int numBodies = idealRobot->getNumBodyNodes();
    BodyNodePtr bodyi;
    string namei;
    double mi;
    double mxi;
    double myi;
    double mzi;

    Eigen::MatrixXd betaParams(1, numBodies*bodyParams);
    int numPertRobots = numBodies*bodyParams;

    for (int i = 0; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);
        namei = bodyi->getName();
        mi = bodyi->getMass();
        mxi = mi * bodyi->getLocalCOM()(0);
        myi = mi * bodyi->getLocalCOM()(1);
        mzi = mi * bodyi->getLocalCOM()(2);

        betaParams(0, i * bodyParams + 0) = mi;
        betaParams(0, i * bodyParams + 1) = mxi;
        betaParams(0, i * bodyParams + 2) = myi;
        betaParams(0, i * bodyParams + 3) = mzi;
    }

    SkeletonPtr pertRobotArray[numPertRobots];
    for (int i = 0; i < numPertRobots; i++) {
        pertRobotArray[i] = idealRobot->clone();
    }

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
    }
    cout << "|-> Done\n";

    cout << "Generating Phi Matrix ...\n";

    // Find phiMatrix
    Eigen::MatrixXd phiMatrix(numInputPoses, numPertRobots);
    double phi;
    Eigen::MatrixXd realxCOMVector(numInputPoses, 1);
    double xCOMIdealRobot;
    double xCOMPertRobot;
    double phiMass;
    double phiXM;
    double phiYM;
    double phiZM;
    double xcom;
    double ycom;
    double zcom;

    // Loop through all the input poses
    for (int pose = 0; pose < numInputPoses; pose++) {

        // Set position of ideal robot to the pose in DART format
        idealRobot->setPositions(inputPoses.row(pose));

        // Get x center of mass
        xCOMIdealRobot = idealRobot->getCOM()(0);
        realxCOMVector(pose, 0) = xCOMIdealRobot;

        for (int pertRobotNum = 0; pertRobotNum < numPertRobots; pertRobotNum++) {

            // Set perturbed robot position to pose
            pertRobotArray[pertRobotNum]->setPositions(inputPoses.row(pose));

            // Get the center of mass of the perturbedRobot
            xCOMPertRobot = pertRobotArray[pertRobotNum]->getCOM()(0);

            // Calculate phi for betai and pose

            // rhs = a*m + b*m*x + c*m*y + d*m*z
            // dm = a + bx + cy + dz
            // dx = bm
            // dy = cm
            // dz = dz
            // a = dm - bx - cy - dz
            // [b, c, d] = [dx, dy, dz]/m
            // phi = [a, b, c, d]

            // TODO: How to calculate phiMatrix
            // Mass
            if (pertRobotNum % bodyParams == 0) {
                phiXM = (xCOMPertRobot - xCOMIdealRobot)/(perturbedValue * pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum/bodyParams)->getMass());
                phiYM = (xCOMPertRobot - xCOMIdealRobot)/(perturbedValue * pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum/bodyParams)->getMass());
                phiZM = (xCOMPertRobot - xCOMIdealRobot)/(perturbedValue * pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum/bodyParams)->getMass());
                xcom = pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum/bodyParams)->getLocalCOM()(0);
                ycom = pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum/bodyParams)->getLocalCOM()(1);
                zcom = pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum/bodyParams)->getLocalCOM()(2);

                phiMass = (xCOMPertRobot - xCOMIdealRobot)/perturbedValue - phiXM*xcom - phiYM*ycom - phiZM*zcom;
                phi = phiMass;

            }
            // mxi
            else if (pertRobotNum % bodyParams == 1) {
                phiXM = (xCOMPertRobot - xCOMIdealRobot)/(perturbedValue * pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum/bodyParams)->getMass());
                phi = phiXM;
            }
            // myi
            else if (pertRobotNum % bodyParams == 2) {
                phiYM = (xCOMPertRobot - xCOMIdealRobot)/(perturbedValue * pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum/bodyParams)->getMass());
                phi = phiYM;
            }
            // mzi
            else {
                phiZM = (xCOMPertRobot - xCOMIdealRobot)/(perturbedValue * pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum/bodyParams)->getMass());
                phi = phiZM;
            }

            phi = (xCOMPertRobot - xCOMIdealRobot)/(perturbedValue);

            // Add phi to phiMatrix and then print it looks cleaner
            phiMatrix(pose, pertRobotNum) = phi;
        }
    }

    cout << "|-> Done\n";

    //Create file to print the actual xCOM of the poses for the ideal robot
    ofstream realxCOMFile;
    realxCOMFile.open("realxCOM.txt");
    realxCOMFile << realxCOMVector;
    realxCOMFile.close();

    // Open output file to write phi matrix
    ofstream phiFile;
    phiFile.open("phiMatrix.txt");

    // phi should be the same no matter estimation/perturbation
    phiFile << phiMatrix;
    phiFile.close();

    return phiMatrix;
}

// // Converge to Beta
// TODO: Add total mass constraint
Eigen::MatrixXd convergeToBeta(string inputName, Eigen::MatrixXd inputPoses, Eigen::MatrixXd phiMatrix, string fullRobotPath, double maxDeviation, double maxOffset, int eons, double n, double u, double suitableError, int suitableNumPoses, double mag, int exp) {
    int numInputPoses = phiMatrix.rows();
    int numBetaParams = phiMatrix.cols();

    // Make idealRobot a copy of krang model
    DartLoader loader;
    SkeletonPtr idealRobot = loader.parseSkeleton(fullRobotPath);

    // Beta Definition/Format
    // mi, mxi, myi, mzi for each body
    int bodyParams = 4;
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
    Eigen::MatrixXd xCOM(1, 1);
    Eigen::MatrixXd xCOMValues((eons*numInputPoses)+1,1);
    double xCOMValue;
    Eigen::MatrixXd delta(1, numBetaParams);

    Eigen::MatrixXd totalMassValues((eons*numInputPoses)+1,1);
    double totalMass;
    Eigen::MatrixXd idealTotalMass(1, 1);
    idealTotalMass << idealRobot->getMass();

    SkeletonPtr currRobot;

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
        if (abs(xCOMValue) > threshold) {

            // Write this pose
            filteredPosesFile << inputPoses.row(pose) << endl;

            xCOM = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - idealTotalMass));
            // Append the next xCOM
            //xCOMValues(k*pose, 0) = xCOMValue;
            xCOMValuesFile << xCOMValue << endl;

            totalMass = (massIndicatorMatrix * currBeta.transpose())(0, 0);
            //totalMassValues(k*pose, 0) = totalMass;
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

// // Create Perturbed parameters
// TODO

// // Change robot's beta values (parameters)
SkeletonPtr setParameters(SkeletonPtr robot, Eigen::MatrixXd betaParams, int bodyParams) {
    Eigen::Vector3d bodyMCOM;

    for (int i = 0; i < betaParams.cols()/bodyParams; i++) {
        robot->getBodyNode(i)->setMass(betaParams(0, i*bodyParams));
        bodyMCOM(0) = betaParams(0, i*bodyParams + 1);
        bodyMCOM(1) = betaParams(0, i*bodyParams + 2);
        bodyMCOM(2) = betaParams(0, i*bodyParams + 3);
        robot->getBodyNode(i/bodyParams)->setLocalCOM(bodyMCOM/(betaParams(0, i*bodyParams)));
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

// // Read file as Matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename) {
    // Read numbers (the pose params)
    ifstream infile;
    infile.open(inputPosesFilename);

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

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
    rows--;

    // Populate matrix with numbers.
    Eigen::MatrixXd outputMatrix(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            outputMatrix(i,j) = buff[cols*i+j];

    return outputMatrix;
}

// // Extract Filename
string extractFilename(string filename) {
    // Remove directory if present.
    // Do this before extension removal incase directory has a period character.
    const size_t last_slash_idx = filename.find_last_of("\\/");
    if (std::string::npos != last_slash_idx) {
        filename.erase(0, last_slash_idx + 1);
    }
    // Remove extension if present.
    const size_t period_idx = filename.rfind('.');
    if (std::string::npos != period_idx) {
        filename.erase(period_idx);
    }

    return filename;
}
