#include <iostream>
#include <eigen3/Eigen/Dense>
#include "./jointPoseGetter/JointPoseGetter.h"
#include <vector>

using namespace std;
using namespace Eigen;

int main()
{
    JointPoseGetter j;

    vector<VectorXd> jointPoses;

    // These should be paramaters to the function
    // TODO get current joint poses (startJointPoses) given as a parameter
    VectorXd startJointPoses(6);
    startJointPoses << 1, 2, 3, 4, 5, 6;
    Vector3d startPos(1, 1, 1);
    Vector3d goalPos(3, 3, 3);
    int totalSteps = 1;

    Vector3d r = goalPos - startPos;

    double stepSize = (1 / totalSteps);
    int firstStep = 1;

    // nextPos is the path of movement.
    Vector3d nextPos = startPos + (stepSize * firstStep) * r;
    VectorXd posRot(6);
    // Rotation needs to be set to a resonable value from robot.
    posRot << nextPos[0], nextPos[1], nextPos[2], 1, 1, 1;
    VectorXd nextJointPoses = j.jacobianInverse(
                startJointPoses[0],
                startJointPoses[1],
                startJointPoses[2],
                startJointPoses[3],
                startJointPoses[4],
                startJointPoses[5])
                * posRot;

    jointPoses.push_back(nextJointPoses);



    for (int currentStep = 2; currentStep < totalSteps; currentStep++) {
        // nextPos is the path of movement.
        nextPos = startPos + (stepSize * currentStep) * r;
        // Rotation needs to be set to a resonable value from robot.
        posRot << nextPos[0], nextPos[1], nextPos[2], 1, 1, 1;
        VectorXd nextJointPoses = j.jacobianInverse(
                    jointPoses.at(currentStep-1)[0],
                    jointPoses.at(currentStep-1)[1],
                    jointPoses.at(currentStep-1)[2],
                    jointPoses.at(currentStep-1)[3],
                    jointPoses.at(currentStep-1)[4],
                    jointPoses.at(currentStep-1)[5])
                    * posRot;

        jointPoses.push_back(nextJointPoses);
    }


    return 0;
}
