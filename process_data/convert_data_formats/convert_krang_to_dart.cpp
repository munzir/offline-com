// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To convert from recorded format of offline-com (krang) to Munzir's
// set of coordinates

// Krang Coordinate Format
// qBase, qWaist, qTorso,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// Munizr Coordinate Format
// heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// DART Coordinate format
// axis-angle1, axis-angle2, axis-angle3, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

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
Eigen::MatrixXd convertPoses(string convert, Eigen::MatrixXd inputPosesFilename, string fullRobotPath);

// // Conversions
Eigen::MatrixXd krangToMunzir(Eigen::RowVectorXd krangPose);
Eigen::MatrixXd munzirToDart(Eigen::RowVectorXd munzirPose);
Eigen::MatrixXd dartToMunzir(Eigen::RowVectorXd dartPose, SkeletonPtr robot);

// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename);

// // Extract filename
string extractFilename(string filename);

// Main Function
int main() {
    // TODO: Command line flags and arguments

    //INPUT on below line (output format)
    //string convert = "krang2dart";
    //string convert = "krang2munzir";
    string convert = "munzir2dart";
    // Options: dart2munzir, munzir2dart, krang2munzir

    //INPUT on below line (input file to convert)
    //string inputPosesFilename = "../../output-balanced-poses.txt";
    string inputPosesFilename = "output-balanced-posesmunzir.txt";

    //INPUT on below line (full robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/KrangNoKinect.urdf";

    // Read input file
    Eigen::MatrixXd inputPoses;
    try {
        cout << "Reading input poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    // Generate output
    cout << "Converting Poses ...\n";
    Eigen::MatrixXd outputPoses = convertPoses(convert, inputPoses, fullRobotPath);
    cout << "|-> Done\n";

    // Name output file
    string outfilename;
    string inputName = extractFilename(inputPosesFilename);
    string outputFormat;
    if (convert == "dart2munzir" || convert == "krang2munzir") {
        outputFormat = "munzir";
    } else {
        outputFormat = "dart";
    }
    string ext = ".txt";
    outfilename = inputName + outputFormat + ext;

    // Writing to file
    cout << "Writing Poses to " << outfilename << " ...\n";
    ofstream outfile;
    outfile.open(outfilename);
    outfile << outputPoses;
    outfile.close();
    cout << "|-> Done\n";

    return 0;
}

// Functions
Eigen::MatrixXd convertPoses(string convert, Eigen::MatrixXd inputPoses, string fullRobotPath) {
    int numInputPoses = inputPoses.rows();
    int outputCols;

    // Robot used to find transform matrix
    DartLoader loader;
    SkeletonPtr robot;

    if (convert == "krang2munzir") {
        outputCols = inputPoses.cols() + 6;
    } else if (convert == "dart2munzir") {
        robot = loader.parseSkeleton(fullRobotPath);
        outputCols = inputPoses.cols() - 1;
    } else {
        outputCols = inputPoses.cols() + 1;
    }

    Eigen::MatrixXd outputPoses(numInputPoses, outputCols);

    // Tranpose big matrix first
    // Cols are now poses, and rows are now params
    inputPoses.transposeInPlace();

    Eigen::MatrixXd convertedPose;

    int poseCounter = 0;
    cout << "Pose: " << poseCounter;

    while (poseCounter < numInputPoses) {

        if (convert == "krang2munzir") {
            convertedPose = krangToMunzir(inputPoses.col(poseCounter));
        } else if (convert == "dart2munzir") {
            convertedPose = dartToMunzir(inputPoses.col(poseCounter), robot);
        } else {
            convertedPose = munzirToDart(inputPoses.col(poseCounter));
        }

        convertedPose.transposeInPlace();
        outputPoses.row(poseCounter) = convertedPose;

        cout << "\rPose: " << ++poseCounter;

    }

    cout << endl;
    return outputPoses;
}

Eigen::MatrixXd krangToMunzir(Eigen::RowVectorXd krangPose) {
    // Find the pose in munzir format
    Eigen::Matrix<double, 16, 1> unchangedValues;
    unchangedValues << krangPose.segment(1,16).transpose();
    double qBase = krangPose.col(0)(0, 0);

    double heading = 0;
    double x = 0;
    double y = 0;
    double z = 0;
    double qLWheel = 0;
    double qRWheel = 0;

    // Now compile this data into munzir pose
    Eigen::Matrix<double, 23, 1> munzirPose;
    munzirPose << heading, qBase, x, y, z, qLWheel, qRWheel, unchangedValues;

    return munzirPose;
}

Eigen::MatrixXd munzirToDart(Eigen::RowVectorXd munzirPose) {
    // Convert input

    // Find the pose in DART formats
    double headingInit = munzirPose(0);
    double qBaseInit = munzirPose(1);
    Eigen::Matrix<double, 21, 1> unchangedValues;
    unchangedValues << munzirPose.segment(2,21).transpose();

    // Calculating the axis angle representation of orientation from headingInit and qBaseInit:
    // RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
    Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

    // Now compile this data into dartPoseParams
    Eigen::Matrix<double, 24, 1> dartPose;
    dartPose << aa.angle()*aa.axis(), unchangedValues;

    return dartPose;
}

Eigen::MatrixXd dartToMunzir(Eigen::RowVectorXd dartPose, SkeletonPtr robot) {
    // Find the pose in munzir format
    Eigen::Matrix<double, 21, 1> unchangedValues;
    unchangedValues << dartPose.segment(3,21).transpose();

    // Calculating the headingInit and qBase Init from the axis angle representation of orientation:
    robot->setPositions(dartPose);
    Eigen::MatrixXd baseTf = robot->getBodyNode(0)->getTransform().matrix();
    double headingInit = atan2(baseTf(0, 0), -baseTf(1, 0));
    double qBaseInit = atan2(baseTf(0,1)*cos(headingInit) + baseTf(1,1)*sin(headingInit), baseTf(2,1));
    // I thought the below would work but its not qbase is off by 1.57 rads (90
    // degs) the above expression for qBaseInit fixes that
    //double qBaseInit = atan2(baseTf(2,1), baseTf(2,2));

    // Now compile this data into munzir pose
    Eigen::Matrix<double, 23, 1> munzirPose;
    munzirPose << headingInit, qBaseInit, unchangedValues;

    return munzirPose;
}

// // Read file as Matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename) {
    // Read numbers (the pose params)
    ifstream infile;
    infile.open(inputPosesFilename);

    if (!infile.is_open()) {
        throw runtime_error(inputPosesFilename + " can not be read, potentially does not exit!");
    }

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
