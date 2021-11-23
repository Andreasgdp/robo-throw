#include "opencv2/opencv.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../coordinateTranslator/CoordinateTranslator.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;


class ThrowCalc
{
public:
    ThrowCalc(/* args */);
    ~ThrowCalc();
    Matrix6d jacobian(double q1, double q2, double q3, double q4, double q5, double q6);
    Matrix6d jacobianInverse(double q1, double q2, double q3, double q4, double q5, double q6);
//    Eigen::VectorXd linearFitFunction(double t, double startTime, const Eigen::VectorXd &dq_end,const Eigen::VectorXd &dq_start, double endTime);
    Eigen::VectorXd linearFitFunction(double t, const Eigen::VectorXd &q_start,const Eigen::VectorXd &accVector);
    std::vector<Eigen::VectorXd> getJointVelocities(double endTime, const Eigen::VectorXd &q_end, const Eigen::VectorXd &q_start, const Eigen::VectorXd &dx, Eigen::VectorXd &accVector);
    Eigen::VectorXd velocityCalc(double xWorld, double yWorld, double zWorld, Eigen::VectorXd throwpos);

private:
};

