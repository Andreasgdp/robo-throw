#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./app/App.h"
#include "./gripperHandling/GripperController.h"
#include "./jointPoseGetter/JointPoseGetter.h"
#include <Eigen/Eigen>
#include "coordinateTranslator/CoordinateTranslator.h"

using namespace std;
using namespace Eigen;

Eigen::Matrix4d CoordinateTranslator::_transformationMatrix;
Eigen::Matrix4d CoordinateTranslator::_inverseTransformationMatrix;

int main()
{
    // Calibration points in robot and world frame
    Vector3d P_robot1 = {-0.05907, -0.31838, 0.17};
    Vector3d P_robot2 = {0.06377, -0.48364, 0.17};
    Vector3d P_robot3 = {0.23051, -0.36235 ,0.17};
    vector<Vector3d> P_robot;
    P_robot.insert(P_robot.end(), {P_robot1, P_robot2, P_robot3});

    Vector3d P_world1 = {0.65, 0.55, 0.205};
    Vector3d P_world2 = {0.45 , 0.5, 0.205};
    Vector3d P_world3 = {0.5, 0.3, 0.205};
    vector<Vector3d> P_world;
    P_world.insert(P_world.end(), {P_world1, P_world2, P_world3});

    // Calibrate robot to table and compute entered point in world to robot frame
    CoordinateTranslator CoordinateTranslator(P_robot, P_world);

    CoordinateTranslator.calibrateRobotToTable();
    cout << "Entered point from world frame in robot frame: " << endl << CoordinateTranslator.computeRobotPointCoords(0.3, 0.4, 0.205) << endl;

    return 0;
}
