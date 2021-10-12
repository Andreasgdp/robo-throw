#include "JointPoseGetter.h"
#include "cmath"

using namespace std;
using namespace Eigen;

JointPoseGetter::JointPoseGetter(/* args */)
{
}

JointPoseGetter::~JointPoseGetter()
{
}

Matrix6d JointPoseGetter::jacobian(double q1, double q2, double q3, double q4, double q5, double q6)
{
    Matrix6d jacobianData;
    jacobianData << (273 * cos(q1)) / 2500 + (3223 * cos(q1) * cos(q5)) / 10000 + (17 * cos(q2) * sin(q1)) / 40 - (947 * cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) / 10000 + (947 * sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) / 10000 - (3223 * sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 - (49 * sin(q1) * sin(q2) * sin(q3)) / 125 + (49 * cos(q2) * cos(q3) * sin(q1)) / 125, (17 * cos(q1) * sin(q2)) / 40 + (947 * cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) / 10000 - (947 * sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) / 10000 + (3223 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + (49 * cos(q1) * cos(q2) * sin(q3)) / 125 + (49 * cos(q1) * cos(q3) * sin(q2)) / 125, (947 * cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) / 10000 - (947 * sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) / 10000 + (3223 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + (49 * cos(q1) * cos(q2) * sin(q3)) / 125 + (49 * cos(q1) * cos(q3) * sin(q2)) / 125, (947 * cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) / 10000 - (947 * sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) / 10000 + (3223 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000, -(3223 * sin(q1) * sin(q5)) / 10000 - (3223 * cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000, 0,
        (273 * sin(q1)) / 2500 - (17 * cos(q1) * cos(q2)) / 40 + (3223 * cos(q5) * sin(q1)) / 10000 + (947 * cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) / 10000 + (947 * sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) / 10000 - (3223 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 - (49 * cos(q1) * cos(q2) * cos(q3)) / 125 + (49 * cos(q1) * sin(q2) * sin(q3)) / 125, (17 * sin(q1) * sin(q2)) / 40 - (947 * cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) / 10000 - (947 * sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) / 10000 + (3223 * sin(q5) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 + (49 * cos(q2) * sin(q1) * sin(q3)) / 125 + (49 * cos(q3) * sin(q1) * sin(q2)) / 125, (3223 * sin(q5) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 - (947 * sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) / 10000 - (947 * cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) / 10000 + (49 * cos(q2) * sin(q1) * sin(q3)) / 125 + (49 * cos(q3) * sin(q1) * sin(q2)) / 125, (3223 * sin(q5) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 - (947 * sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) / 10000 - (947 * cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) / 10000, (3223 * cos(q1) * sin(q5)) / 10000 + (3223 * cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000, 0,
        0, (49 * sin(q2) * sin(q3)) / 125 - (49 * cos(q2) * cos(q3)) / 125 - (17 * cos(q2)) / 40 - (3223 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 + (947 * cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) / 10000 + (947 * sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) / 10000, (49 * sin(q2) * sin(q3)) / 125 - (49 * cos(q2) * cos(q3)) / 125 - (3223 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 + (947 * cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) / 10000 + (947 * sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) / 10000, (947 * cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) / 10000 - (3223 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 + (947 * sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) / 10000, -(3223 * cos(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000, 0,
        0, sin(q1), sin(q1), sin(q1), cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)), cos(q5) * sin(q1) - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))),
        0, -cos(q1), -cos(q1), -cos(q1), cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)), sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - cos(q1) * cos(q5),
        1, 0, 0, 0, sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)), -sin(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)));

    return jacobianData;
}

Matrix6d JointPoseGetter::jacobianInverse(double q1, double q2, double q3, double q4, double q5, double q6)
{
    Matrix6d jacobian = this->jacobian(q1, q2, q3, q4, q5, q6);
    return jacobian.inverse();
}

vector<VectorXd> JointPoseGetter::getJointPoses(VectorXd startJointPoses, Vector3d startPos, Vector3d goalPos, int totalSteps)
{
    vector<VectorXd> jointPoses;

    // Direction vector
    Vector3d r = goalPos - startPos;

    double stepSize = (1 / totalSteps);
    int firstStep = 1;

    jointPoses.push_back(this->calcNextJointPoses(startPos, r, stepSize, firstStep, startJointPoses));

    for (int currentStep = 2; currentStep < totalSteps; currentStep++) {
        jointPoses.push_back(this->calcNextJointPoses(startPos, r, stepSize, firstStep, jointPoses.at(currentStep-1)));
    }

    return jointPoses;
}

VectorXd JointPoseGetter::calcNextJointPoses(Vector3d startPos, Vector3d r, double stepSize, int firstStep, VectorXd lastPos)
{
    // nextPos is the path of movement.
    Vector3d nextPos = startPos + (stepSize * firstStep) * r;
    VectorXd posRot(6);
    // Rotation needs to be set to a resonable value from robot.
    posRot << nextPos[0], nextPos[1], nextPos[2], 1, 1, 1;
    VectorXd nextJointPoses = this->jacobianInverse(
                lastPos[0],
                lastPos[1],
                lastPos[2],
                lastPos[3],
                lastPos[4],
                lastPos[5])
                * posRot;
    return nextJointPoses;
}
