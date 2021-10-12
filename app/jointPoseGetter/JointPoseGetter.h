#include "opencv2/opencv.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;


class JointPoseGetter
{
private:
	/* data */
public:
	JointPoseGetter(/* args */);
	~JointPoseGetter();
    Matrix6d jacobian(double q1, double q2, double q3, double q4, double q5, double q6);
    Matrix6d jacobianInverse(double q1, double q2, double q3, double q4, double q5, double q6);
    std::vector<Eigen::VectorXd> getJointPoses(Eigen::VectorXd startJointPoses, Eigen::Vector3d startPos, Eigen::Vector3d goalPos, int totalSteps);

private:
    Eigen::VectorXd calcNextJointPoses(Eigen::Vector3d startPos, Eigen::Vector3d r, double stepSize, int firstStep, Eigen::VectorXd lastPos);

};

