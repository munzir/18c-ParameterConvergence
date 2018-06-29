// Author: Akash Patel (apatel435@gatech.edu)

// genPhiMatrixAsFile
// Purpose: Determine phi vectors for each input pose
//      This phi contains the weights of each parameter at every pose
//
// Input: Ideal beta={mi, MXi, MYi, MZi, ...}, krang urdf model, perturbation value, data points (q/poses) as a file,
// Output: Phi matrix as a file

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

// // Test Beta
Eigen::MatrixXd testBeta(Eigen::MatrixXd beta, Eigen::MatrixXd phiMatrix, string fullRobotPath);

// // Change robot's beta values (parameters)
SkeletonPtr setParameters(SkeletonPtr robot, Eigen::MatrixXd beta, int bodyParams);

// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename);

// // Extract filename
string extractFilename(string filename);

// // Return a copy of the passed in robot
SkeletonPtr copyRobot(SkeletonPtr robot);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (input poses filename)
    //string inputPosesFilename = "../random500fullbalance0.001000tolunsafe.txt";
    string inputPosesFilename = "../randomOptPoses10000.txt";
    //string inputPosesFilename = "../custom2comfullbalancenotolunsafe.txt";

    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(10, -10);

    // INPUT on below line (input beta vector file)
    //string inputBetaFilename = "../betaVectorscustom2comfullbalancenotolunsafe-3filter.txt";
    //string inputBetaFilename = "../betaVectorsrandomOptPoses100000.000000*10e-3filter.txt";
    //string inputBetaFilename = "../betaVectorscustom2comfullbalancenotolunsafe1*10e-3filter.txt";
    string inputBetaFilename = "../betaVectorsIdeal.txt";

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf";

    // INPUT on below line (output filename)
    string outputBaseName = "testxCOMValues";

    cout << "Reading input poses ...\n";
    Eigen::MatrixXd inputPoses = readInputFileAsMatrix(inputPosesFilename);
    cout << "|-> Done\n";

    Eigen::MatrixXd phiMatrix = genPhiMatrix(inputPoses, fullRobotPath, perturbedValue);

    cout << "Reading converged beta ...\n";
    Eigen::MatrixXd betaVectors = readInputFileAsMatrix(inputBetaFilename);
    Eigen::MatrixXd beta = betaVectors.row(betaVectors.rows() - 1);
    cout << "|-> Done\n";

    cout << "Testing Beta ...\n";
    Eigen::MatrixXd testXCOMValues = testBeta(beta, phiMatrix, fullRobotPath);
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

// // Generate Phi Matrix
// TODO: Make perturbedRobotArray generation faster
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

    // TODO: Need to create an array of pertRobots in a fast time
    // Create array of robots out of pose loop for efficiency
    // then change appropriate values (betaParams(i)) for each robot when
    // going through all the robots
    SkeletonPtr pertRobotArray[numPertRobots];
    for (int i = 0; i < numPertRobots; i++) {
        // TODO: Segfaulting right here
        // Trying to create an array of idealRobots by calling parseSkeleton
        // only once since it is time expenseive
        //memcpy(pertRobotArray[i], idealRobot, sizeof(SkeletonPtr));

        pertRobotArray[i] = loader.parseSkeleton(fullRobotPath);
        //pertRobotArray[i] = copyRobot(idealRobot);
    }

    for (int pertRobotNum = 0; pertRobotNum < numPertRobots; pertRobotNum++) {
        // TODO: Can i make this another method
        // Input: Eigen::MatrixXd parameters Output: SkeletonPtr robot with new parameters
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

    // Loop through all the input poses
    for (int pose = 0; pose < numInputPoses; pose++) {

        // Set position of ideal robot to the pose in DART format
        idealRobot->setPositions(inputPoses.row(pose));

        // Get x center of mass
        xCOMIdealRobot = idealRobot->getCOM()(0);
        realxCOMVector(pose, 0);

        for (int pertRobotNum = 0; pertRobotNum < numPertRobots; pertRobotNum++) {

            // Set perturbed robot position to pose
            pertRobotArray[pertRobotNum]->setPositions(inputPoses.row(pose));

            // Get the center of mass of the perturbedRobot
            xCOMPertRobot = pertRobotArray[pertRobotNum]->getCOM()(0);

            // Calculate phi for betai and pose
            phi = (xCOMPertRobot - xCOMIdealRobot)/perturbedValue;

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

// // Test Beta
Eigen::MatrixXd testBeta(Eigen::MatrixXd beta, Eigen::MatrixXd phiMatrix, string fullRobotPath) {
    int numInputPoses = phiMatrix.rows();
    int numBetaParams = phiMatrix.cols();

    // Make idealRobot a copy of krang model
    DartLoader loader;
    SkeletonPtr idealRobot = loader.parseSkeleton(fullRobotPath);

    Eigen::MatrixXd phiVec(1, numBetaParams);
    Eigen::MatrixXd xCOM(1, 1);
    Eigen::MatrixXd xCOMValues(numInputPoses+1,1);
    double xCOMValue;

    SkeletonPtr currRobot;

    // Loop through the phi matrix to test the beta
    for (int pose = 0; pose < numInputPoses; pose++) {
        phiVec = phiMatrix.row(pose);

        // Add the xCOM to the matrix
        xCOMValue = (phiVec * beta.transpose())(0, 0);
        // currRobot = setParameters(idealRobot, beta, bodyParams);
        // xCOMValue = currRobot->getCOM()(0);

        // Append the next xCOM
        xCOMValues(pose, 0) = xCOMValue;

    }

    // Add the xCOM value of the last beta and the last pose
    // Same with the total mass
    xCOMValue = (phiVec * beta.transpose())(0, 0);
    // currRobot = setParameters(idealRobot, beta, bodyParams);
    // xCOMValue = currRobot->getCOM()(0);
    xCOMValues(numInputPoses, 0) = xCOMValue;

    return xCOMValues;
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

// // Return a copy of the passed in robot
// TODO
SkeletonPtr copyRobot(SkeletonPtr robot) {
    return robot;
}
