#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "./robotConnection/RobotConnection.h"
#include "imageProcessing/ImageProcessing.h"
#include <math.h>
#include <chrono>
#include <thread>
#include "coordinateTranslator/CoordinateTranslator.h"
#include "gripperHandling/GripperController.h"
#include "simulation/Simulation.h"
#include "api/Api.h"
#include "app/App.h"

using namespace std;
using namespace Eigen;
using namespace ur_rtde;

int main(int argc, char *argv[])
{

    App app("192.168.100.49");
    bool wannaThrow = true;

    while (true)
    {
        app.findAndGrabObject();
        app.throwObject();
    }

    return 0;
}
