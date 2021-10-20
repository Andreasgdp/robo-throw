#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./app/App.h"
#include "./gripperHandling/GripperController.h"
#include "./jointPoseGetter/JointPoseGetter.h"
#include <Eigen/Eigen>
#include "coordinateTranslator/CoordinateTranslator.h"

using namespace std;
using namespace Eigen;

int main()
{

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


    CoordinateTranslator CoordTrans(P_robot, P_world);

    CoordTrans.calibrateRobotToTable();
    cout << "_transformationMatrix: " << endl << CoordTrans.getTransformationMatrix() << endl;

    //std::cout << "U: " << endl << svd.matrixU() << std::endl << endl;
    //std::cout << "V: " << endl << svd.matrixV() << std::endl << endl;

    /*
    for (int i = 0; i < P_robot.size(); i++) {
        cout << "q_robot" << i << ": " << endl << q_robot.at(i) << endl << endl;
    }


    for (int i = 0; i < P_world.size(); i++) {
        cout << "q_world" << i << ": " << endl << q_world.at(i) << endl << endl;
    }
    */

    return 0;
}
