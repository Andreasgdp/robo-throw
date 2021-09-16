#include "Simulation.h"

using namespace std;

Simulation::Simulation() {}

cv::Mat Simulation::loadImage(std::string imageFileName, std::string imageFileType) {
    // Read the image file given the file path
    // TODO: Change cv::IMREAD_COLOR if needed for the project
    cv::Mat image = cv::imread("../app/simulation/images/" + imageFileName + "." + imageFileType, cv::IMREAD_COLOR);

    // Error Handling
    if (image.empty()) throw "Image File Not Found";
    return image;
}

void Simulation::showImage(cv::Mat image, std::string windowName) {
    // Show Image inside a window with the window name provided
    cv::imshow("Image show", image);

    // Wait for any keystroke
    cv::waitKey(0);
}
