#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H
#include "opencv2/opencv.hpp"
#include "iostream"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"
#include "vector"
#include <pylon/PylonIncludes.h>


class ImageProcessing
{
public:
    ImageProcessing();
    cv::Mat loadImage();
    void calibrate();
    cv::Mat loadImagePC(std::string numbner);
    void getBoardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f>>& foundCorners);
    void showImage(cv::Mat image, std::string windowName = "Window");
    void run();
    std::vector<cv::Mat> pylonPic();
};

#endif // IMAGEPROCESSING_H
