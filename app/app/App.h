#ifndef APP_H
#define APP_H
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "../imageProcessing/ImageProcessing.h"
#include "../robotConnection/RobotConnection.h"

class App
{
public:
    App(std::string IP);
    void initializeApp(const cv::Mat &calibrationImg = cv::Mat());
    void findAndGrabObject(const cv::Mat &objectImg = cv::Mat());
    void throwObject(const std::vector<double> &goalPos);
    void moveHome();


private:
    // Member variables
    //TODO: when all member variables are created, create getters and setters (automatic)
    std::string IP;
    std::vector<double> jointPoses;
    double speed = 1.0;
    double acceleration = 1.0;
    std::vector<double> homeJointPoses;
    ImageProcessing imgProcessor;
    RobotConnection roboConn;
    // add member variable for jointPoseGetter
    // add member variable for gripperHandling
    // add member variable for api
    // add member variable for image
    // add member variable for processed image
    // add member variable for object posision
    // add member variable for target pos

    // Functions
    bool isImageProvided(cv::Mat image);
    bool robotHasMovedToPos(const std::vector<double> &pos);
    void waitForMoveRobot(const std::vector<double> &pos);
    void setDefaultSpeedAcceleration();


};

#endif // APP_H
