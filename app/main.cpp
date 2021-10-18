#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./app/App.h"
#include "./gripperHandling/GripperController.h"
#include "./jointPoseGetter/JointPoseGetter.h"


using namespace std;

int main()
{
    JointPoseGetter j;

    vector<vector<double>> test = j.jacobian(1, 1, 1, 1, 1, 1);
    j.showJacobian();

    return 0;
}
