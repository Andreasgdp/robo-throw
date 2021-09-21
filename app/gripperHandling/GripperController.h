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
    /*
     * @param Closes the gripper
     */
    void open();
    /*
     * @param Opens the gripper
     */
    rl::hal::WeissWsg50::GraspingState getGraspingState();
    /*
     * @param Gets the graspingstate
     * @return Graspingstate
     */
    void setAcceleration(const float &acceleration);
    /*
     * @param Sets the acceleration
     */
    float getAcceleration();
    /*
     * @return Acceleration of the gripper
     */
    void setForceLimit(const float &force);
    /*
     * @param Sets the forcelimit of the gripper
     */
    float getForceLimit();
    /*
     * @return Forcelimit of the gripper
     */
    void setSoftLimits(const float &limitMinus, const float &limitPlus);
    /*
     * @param Sets the softlimits of the gripper
     */

private:
    rl::hal::WeissWsg50 gripper;
};

#endif // GRIPPERCONTROLLER_H
