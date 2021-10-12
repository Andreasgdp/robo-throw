#include <iostream>
#include <eigen3/Eigen/Dense>
#include "./jointPoseGetter/JointPoseGetter.h"

using namespace std;
using namespace Eigen;

int main()
{
    JointPoseGetter j;

    Vector3d startPos(1, 1, 1);
    Vector3d goalPos(3, 3, 3);
    int steps = 1;

    Vector3d r = goalPos - startPos;

    int currentStep = 1;

    Vector3d nextPos = startPos + ((1 / steps) * currentStep) * r;

    VectorXd posRot(6);

    posRot << nextPos[0], nextPos[1], nextPos[2], 1, 1, 1;

    VectorXd startJointPoses(6);
    startJointPoses << 1, 2, 3, 4, 5, 6;

    VectorXd nextJointPoses = j.jacobianInverse(startJointPoses[0], startJointPoses[1], startJointPoses[2], startJointPoses[3], startJointPoses[4], startJointPoses[5]) * posRot;

    cout << nextJointPoses << endl;

    return 0;
}
