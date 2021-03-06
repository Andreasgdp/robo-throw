#include "ThrowCalc.h"
#include "cmath"
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

ThrowCalc::ThrowCalc(/* args */)
{
}

ThrowCalc::~ThrowCalc()
{
}

Matrix6d ThrowCalc::jacobian2(double q1, double q2, double q3, double q4, double q5, double q6)
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

Matrix6d ThrowCalc::jacobian(double q1, double q2, double q3, double q4, double q5, double q6)
{
    Matrix6d jacobianData;
    jacobianData << (273 * cos(q1)) / 2500 + (2723 * cos(q1) * cos(q5)) / 10000 + (17 * cos(q2) * sin(q1)) / 40 - (947 * cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) / 10000 + (947 * sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) / 10000 - (2723 * sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 - (49 * sin(q1) * sin(q2) * sin(q3)) / 125 + (49 * cos(q2) * cos(q3) * sin(q1)) / 125, (17 * cos(q1) * sin(q2)) / 40 + (947 * cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) / 10000 - (947 * sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) / 10000 + (2723 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + (49 * cos(q1) * cos(q2) * sin(q3)) / 125 + (49 * cos(q1) * cos(q3) * sin(q2)) / 125, (947 * cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) / 10000 - (947 * sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) / 10000 + (2723 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + (49 * cos(q1) * cos(q2) * sin(q3)) / 125 + (49 * cos(q1) * cos(q3) * sin(q2)) / 125, (947 * cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) / 10000 - (947 * sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) / 10000 + (2723 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000, -(2723 * sin(q1) * sin(q5)) / 10000 - (2723 * cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000, 0,
        (273 * sin(q1)) / 2500 - (17 * cos(q1) * cos(q2)) / 40 + (2723 * cos(q5) * sin(q1)) / 10000 + (947 * cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) / 10000 + (947 * sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) / 10000 - (2723 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 - (49 * cos(q1) * cos(q2) * cos(q3)) / 125 + (49 * cos(q1) * sin(q2) * sin(q3)) / 125, (17 * sin(q1) * sin(q2)) / 40 - (947 * cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) / 10000 - (947 * sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) / 10000 + (2723 * sin(q5) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 + (49 * cos(q2) * sin(q1) * sin(q3)) / 125 + (49 * cos(q3) * sin(q1) * sin(q2)) / 125, (2723 * sin(q5) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 - (947 * sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) / 10000 - (947 * cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) / 10000 + (49 * cos(q2) * sin(q1) * sin(q3)) / 125 + (49 * cos(q3) * sin(q1) * sin(q2)) / 125, (2723 * sin(q5) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 - (947 * sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) / 10000 - (947 * cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) / 10000, (2723 * cos(q1) * sin(q5)) / 10000 + (2723 * cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000, 0,
        0, (49 * sin(q2) * sin(q3)) / 125 - (49 * cos(q2) * cos(q3)) / 125 - (17 * cos(q2)) / 40 - (2723 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 + (947 * cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) / 10000 + (947 * sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) / 10000, (49 * sin(q2) * sin(q3)) / 125 - (49 * cos(q2) * cos(q3)) / 125 - (2723 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 + (947 * cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) / 10000 + (947 * sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) / 10000, (947 * cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) / 10000 - (2723 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 + (947 * sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) / 10000, -(2723 * cos(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000, 0,
        0, sin(q1), sin(q1), sin(q1), cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)), cos(q5) * sin(q1) - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))),
        0, -cos(q1), -cos(q1), -cos(q1), cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)), sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - cos(q1) * cos(q5),
        1, 0, 0, 0, sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)), -sin(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)));

    return jacobianData;
}

Matrix6d ThrowCalc::jacobianInverse(double q1, double q2, double q3, double q4, double q5, double q6)
{
    Matrix6d jacobian = this->jacobian(q1, q2, q3, q4, q5, q6);
    return jacobian.inverse();
}

VectorXd ThrowCalc::linearFitFunction(double t, const VectorXd &q_start, const VectorXd &accVector)
{
    return accVector * t;
}

vector<VectorXd> ThrowCalc::getJointVelocities(double endTime, const VectorXd &q_end, const VectorXd &q_start, const VectorXd &dx, VectorXd &accVector)
{
    VectorXd dq_end = this->jacobianInverse(q_end[0], q_end[1], q_end[2], q_end[3], q_end[4], q_end[5]) * dx;

    VectorXd dq_start(6);
    dq_start << 0, 0, 0, 0, 0, 0;

    double startTime = 0;

    double stepSize = 0.008;
    int totalSteps = endTime / stepSize;

    vector<VectorXd> jointVelocities;
    for (int step = 0; step <= totalSteps; step++)
    {
        jointVelocities.push_back(linearFitFunction(step * stepSize, q_start, accVector));
    }

    return jointVelocities;
}

VectorXd ThrowCalc::velocityCalc(double xWorld, double yWorld, double zWorld, VectorXd throwpos)
{
    // Initialise the "variables"
    double v0x, vzx, vx, v0y, vzy, vy, x, y, z;
    double g = 9.82;
    double a = 25;
    int t = 0;
    VectorXd velocityXYZ(6);

    // Calc the angle in radian
    double angle = (a * M_PI) / 180;

    // Convert world coordinates to robot coordinates
    CoordinateTranslator coordinateTranslator;
    Vector3d posOfTargetInRobotCoord = coordinateTranslator.computeRobotPointCoords(xWorld, yWorld, zWorld);

    // Translate 0.0.0 to the throwpos
    x = posOfTargetInRobotCoord[0] - throwpos[0];
    y = posOfTargetInRobotCoord[1] - throwpos[1];
    z = posOfTargetInRobotCoord[2] - throwpos[2];

    // In the x,z plane
    // The general speed
    int signX = (x >= 0) ? 1 : -1;

    v0x = (signX * 0.70771 * x * sqrt(abs(-g / (z - tan(angle) * x)))) / (cos(angle));
    // The speed in x and the speed in z
    vx = v0x * cos(angle);
    vzx = v0x * sin(angle) - g * t;

    // In the y,z plane
    // The general speed
    int signY = (y <= 0) ? 1 : -1;

    v0y = (signY * 0.70771 * y * sqrt(abs(-g / (z - tan(angle) * y)))) / (cos(angle));
    // The speed in x and the speed in z
    vy = v0y * cos(angle);
    vzy = v0y * sin(angle) - g * t;

    // adding the two z speeds
    double vz = abs(vzx) + abs(vzy);
    cout << "vzx: " << vzx << endl
         << endl;
    cout << "vzy: " << vzy << endl
         << endl;

    velocityXYZ << vx, vy, vz, 0, 0, 0;

    cout << "Vel Vec: " << velocityXYZ << endl
         << endl;

    return velocityXYZ;
}

double ThrowCalc::TCPAngleCalc(double targetX, double targetY, Eigen::VectorXd throwpos)
{
    double a = abs(throwpos[1] - targetY);
    double b = throwpos[0] - targetX;
    double c = sqrt(pow(a, 2) + pow(b, 2));
    double C = (90 * M_PI) / 180;
    ;
    double v = asin((sin(C) * a) / (c));
    int signV = (targetY > throwpos[1]) ? -1 : 1;
    v = v * signV;
    cout << "(-65.59 * M_PI)/180): " << (-65.59 * M_PI) / 180 << endl;
    cout << "v: " << v << endl;
    if (((-65.59 * M_PI) / 180) + v < (-90 * M_PI) / 180)
    {
        v = ((-90 + 65.59) * M_PI) / 180;
    }
    return v;
}
