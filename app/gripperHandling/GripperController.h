#ifndef GRIPPERCONTROLLER_H
#define GRIPPERCONTROLLER_H

#include <rl/hal/WeissWsg50.h>

class GripperController
{
public:
    //TODO: Go over the default values and make them relevant to this project.
    GripperController(
            const std::string &address="192.168.100.10",
            const unsigned short int &port=1000,
            const float &acceleration=3.0f,
            const float &forceLimit=10.0f,
            const unsigned int &period=10);

    ~GripperController();

    /*
     * @brief Conncects to the gripper
     */
    void connect();

    /*
     * @brief Disconnect from the gripper
     */
    void disconnect();

    /*
     * @brief Close the gripper and stop on block
     */
    void close();

    /*
     * @brief Opens the gripper
     * @param The speed the gripper opens with
     */
    void open(const float &speed = 0.05f);

    /*
     * @brief Move gripper to home posistion, fully opened
     */
    void moveHome();

    /*
     * @brief Gets the state of the gripper
     * @return the name of the grasping state
     */
    std::string getGraspingState();

    /*
     * @brief Sets the acceleration
     */
    void setAcceleration(const float &acceleration);

    /*
     * @return Acceleration of the gripper
     */
    float getAcceleration();

    /*
     * @brief Sets the forcelimit of the gripper
     */
    void setForceLimit(const float &force);

    /*
     * @return Forcelimit of the gripper
     */
    float getForceLimit();

    /*
     * @brief Sets the softlimits of the gripper
     */
    void setSoftLimits(const float &limitMinus, const float &limitPlus);

private:
    rl::hal::WeissWsg50 _gripper;

    /*
     * @brief Gets the graspingstate
     * @return Graspingstate as integer
     */
    rl::hal::WeissWsg50::GraspingState _getGraspingState();
};

#endif // GRIPPERCONTROLLER_H
