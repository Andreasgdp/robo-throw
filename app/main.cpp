#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./app/App.h"
#include "./gripperHandling/GripperController.h"

using namespace std;

int main()
{

    // TODO: use IP of robot.
    App app("130.226.87.132");
    app.calibrateCam();
    app.findAndGrabObject();
    std::vector<double> goalPos;
    app.throwObject(goalPos);


    GripperController myGripper("192.168.100.11", 1000);
    try {
        myGripper.moveHome();

        myGripper.close();
        myGripper.open();

    }  catch (exception& e) {
        cout << e.what() << endl;
        return 1;
    }

    return 0;
}
