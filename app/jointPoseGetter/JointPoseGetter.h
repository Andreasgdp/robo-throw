#include "opencv2/opencv.hpp"

class JointPoseGetter
{
private:
	/* data */
public:
	JointPoseGetter(/* args */);
	~JointPoseGetter();
	cv::Mat jacobian(double q1, double q2, double q3, double q4, double q5, double q6);
private:
};

