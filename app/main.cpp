#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./app/App.h"
#include "./gripperHandling/GripperController.h"

using namespace std;

int main()
{
    cout << "hello!" << endl;
//    // TODO: use IP of robot.
//    App app("130.226.87.132");
//    app.calibrateCam();
//    app.findAndGrabObject();
//    std::vector<double> goalPos;
//    app.throwObject(goalPos);

    RobotConnection conn("192.168.100.49");
    vector<double> test = conn.getActualJointPoses();

    for (auto& elm: test) {
        cout << elm  << ", ";
    }

    vector<double> jointPoses = {1.74947, -1.3914, 1.09235, -2.86513, -1.58442, 0.0157585};

    conn.moveJ(jointPoses);


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
