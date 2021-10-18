#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./app/App.h"
#include "./gripperHandling/GripperController.h"
#include "./jointPoseGetter/JointPoseGetter.h"


using namespace std;

int main()
{
    ImageProcessing imageprocessing;

    imageprocessing.calibrate();

    return 0;
}
