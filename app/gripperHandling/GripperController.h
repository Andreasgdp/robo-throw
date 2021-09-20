#ifndef GRIPPERCONTROLLER_H
#define GRIPPERCONTROLLER_H

#include <rl/hal/WeissWsg50.h>

class GripperController
{
public:
    //TODO: Go over the default values and make them relevant to this project.
    GripperController(const std::string &address="192.168.1.20", const unsigned short int &port=1000, const float &acceleration=3.0f, const float &forceLimit=40.0f, const unsigned int &period=10);
    void close();
    void open();

private:
    rl::hal::WeissWsg50 gripper;
    std::string address;
    unsigned short int port;
    float acceleration;
    float forceLimit;
    unsigned int period;
};

#endif // GRIPPERCONTROLLER_H
