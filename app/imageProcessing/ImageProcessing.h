#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H
#include "opencv2/opencv.hpp"
#include "iostream"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"
#include "vector"


class ImageProcessing
{
public:
    ImageProcessing();
    void loadImage();
    void calibrate(cv::Mat frame);
    cv::Mat loadImagePC();
    void getBoardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f>>& foundCorners);
    void showImage(cv::Mat image, std::string windowName = "Window");
    void run();
};

#endif // IMAGEPROCESSING_H







