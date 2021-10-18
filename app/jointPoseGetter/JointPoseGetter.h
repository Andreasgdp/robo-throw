#include "opencv2/opencv.hpp"
#include <vector>

class JointPoseGetter
{
private:
	/* data */
public:
	JointPoseGetter(/* args */);
	~JointPoseGetter();
    std::vector<std::vector<double>> jacobian(double q1, double q2, double q3, double q4, double q5, double q6);
    void showJacobian();
private:
    std::vector<std::vector<double>> j;
};

