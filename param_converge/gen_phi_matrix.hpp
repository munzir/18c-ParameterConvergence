// Author: Akash Patel (apatel435@gatech.edu)

// genPhiMatrixAsFile
// Purpose: Determine phi vectors for each input pose
//      This phi contains the weights of each parameter at every pose
//
// Input: Ideal beta={mi, MXi, MYi, MZi, ...}, krang urdf model, perturbation value, data points (q/poses) as a file,
// Output: Phi matrix as a file

// Includes
//#include <dart/dart.hpp>

// Namespaces
using namespace std;
//using namespace dart::dynamics;

// Function Prototypes
// // Generate Phi Matrix
Eigen::MatrixXd genPhiMatrix(Eigen::MatrixXd inputPoses, int bodyParams, string fullRobotPath, double perturbedValue);
