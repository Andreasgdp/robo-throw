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
    std::vector<cv::Mat> loadImagePC();
    void getBoardCorners(std::vector<cv::Mat> images);
    void showImage(cv::Mat image, std::string windowName = "Window");
    std::vector<cv::Mat> pylonPic();
    void getCornersV2();

private:
    const cv::Size BoardSize{6,9};
    std::vector<std::vector<cv::Point3f>> Q; //Checkerboard voordinates
};

#endif // IMAGEPROCESSING_H
