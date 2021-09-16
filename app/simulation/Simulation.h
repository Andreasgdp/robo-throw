#ifndef SIMULATION_H
#define SIMULATION_H

#include "opencv2/opencv.hpp"

class Simulation
{
public:
    Simulation();
    cv::Mat loadImage(std::string imageFileName, std::string imageFileType);
    void showImage(cv::Mat image, std::string windowName = "Window");

private:
    // TODO: Add member varible for main
};

#endif // SIMULATION_H
