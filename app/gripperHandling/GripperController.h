#ifndef GRIPPERCONTROLLER_H
#define GRIPPERCONTROLLER_H

#include <rl/hal/WeissWsg50.h>

class GripperController
{
public:
    //TODO: Go over the default values and make them relevant to this project.
    GripperController(
            const std::string &address="192.168.1.20",
            const unsigned short int &port=1000,
            const float &acceleration=3.0f,
            const float &forceLimit=40.0f,
            const unsigned int &period=10);
    void close();
    void open();
    rl::hal::WeissWsg50::GraspingState getGraspingState();
    void setAcceleration(const float &acceleration);
    void setForceLimit(const float &force);
    void setSoftLimits(const float &limitMinus, const float &limitPlus);

private:
    rl::hal::WeissWsg50 gripper;
};

#endif // GRIPPERCONTROLLER_H
