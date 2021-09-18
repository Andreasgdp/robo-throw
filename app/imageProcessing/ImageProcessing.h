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
