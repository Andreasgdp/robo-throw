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

VectorXd JointPoseGetter::linearFitFunction(double t, double startTime, const VectorXd &dq_end,const VectorXd &dq_start, double endTime)
{
    // Find slope of each joint velocity based on start, end and delta-time
    VectorXd next_dq(6);
    double dq1 = ((dq_end[0] - dq_start[0])/(endTime - startTime)) * t;
    double dq2 = ((dq_end[1] - dq_start[1])/(endTime - startTime)) * t;
    double dq3 = ((dq_end[2] - dq_start[2])/(endTime - startTime)) * t;
    double dq4 = ((dq_end[3] - dq_start[3])/(endTime - startTime)) * t;
    double dq5 = ((dq_end[4] - dq_start[4])/(endTime - startTime)) * t;
    double dq6 = ((dq_end[5] - dq_start[5])/(endTime - startTime)) * t;
    next_dq << dq1, dq2, dq3, dq4, dq5, dq6;
    return next_dq;
}

vector<VectorXd> JointPoseGetter::getJointVelocities(const VectorXd &q_start, const VectorXd &q_end,const VectorXd &dx)
{
    VectorXd dq_end = this->jacobianInverse(q_end[0], q_end[1], q_end[2], q_end[3], q_end[4], q_end[5]) * dx;

    VectorXd dq_start(6);
    dq_start << 0, 0, 0, 0, 0, 0;

    double startTime = 0;
    double endTime = 4;

    double stepSize = 0.008;
    int totalSteps = endTime/stepSize;
    cout << "total steps: " << totalSteps << endl;

    vector<VectorXd> jointVelocities;
    for(int step = 0; step <= totalSteps; step++) {
        jointVelocities.push_back(linearFitFunction(step * stepSize, startTime, dq_end, dq_start, endTime));
    }

    return jointVelocities;
}
