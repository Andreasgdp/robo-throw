#include <iostream>
#include <eigen3/Eigen/Dense>
#include "./jointPoseGetter/JointPoseGetter.h"


using namespace std;
using namespace Eigen;

typedef Matrix<float, 6, 6> Matrix6f;

int main()
{
    JointPoseGetter j;

    cout << j.jacobian(1, 1, 1, 1, 1, 1).inverse() << endl;

    return 0;
}
