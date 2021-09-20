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
    std::vector<cv::Mat> loadImagePC();
    void getBoardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f>>& foundCorners);
    void run();
};

#endif // IMAGEPROCESSING_H

#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H
#include "opencv2/opencv.hpp"
#include "iostream"


class ImageProcessing
{
public:
    ImageProcessing();
    void loadImage();
    cv::Mat loadLocalImage(std::string imageFileName, std::string imageFileType);
    void showImage(cv::Mat image, std::string windowName = "Window");
};

#endif // IMAGEPROCESSING_H
