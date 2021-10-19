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

    CoordTrans.setNumberOfPoints();
    vector<Vector3d> q_robot = CoordTrans.computeZeroCentroidPointSet(P_robot);
    vector<Vector3d> q_world = CoordTrans.computeZeroCentroidPointSet(P_world);

    for (int i = 0; i < P_robot.size(); i++) {
        cout << "q_robot" << i << ": " << endl << q_robot.at(i) << endl << endl;
    }


    for (int i = 0; i < P_world.size(); i++) {
        cout << "q_world" << i << ": " << endl << q_world.at(i) << endl << endl;
    }


    /*
    Eigen::MatrixXf A(3,3);
    A << -0.019889333333333342, -0.03729500000000002, 0, 0.01523733333333324, -0.0016675000000001758, -0.7806255642 * pow(10,-17), 0.3788053204 * pow(10,-16), 0.6978778666 * pow(10,-16), 0.2914335440 * pow(10,-17);

    //Matrix(3, 3, [[-0.019889333333333342, -0.03729500000000002, 0.],
    //              [0.01523733333333324, -0.0016675000000001758, -0.7806255642*10^(-17)],
    //              [0.3788053204*10^(-16), 0.6978778666*10^(-16), 0.2914335440*10^(-17)]])

    std::cout << "A: " << endl << A << std::endl << endl;

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "U: " << endl << svd.matrixU() << std::endl << endl;
    std::cout << "V: " << endl << svd.matrixV() << std::endl << endl;
    */

    return 0;
}
