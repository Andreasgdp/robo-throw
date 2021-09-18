#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./app/App.h"

using namespace std;

int main()
{
    // TODO: use IP of robot.
    App app("130.226.87.132");
    app.calibrateCam();
    app.findAndGrabObject();
    std::vector<double> goalPos;
    app.throwObject(goalPos);

    return 0;
}
