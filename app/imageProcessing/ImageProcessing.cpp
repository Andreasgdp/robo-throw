#include "ImageProcessing.h"

ImageProcessing::ImageProcessing(){}

void ImageProcessing::calibrate()
{
    std::vector<cv::Mat> tmp;

    std::string hCalib;


    std::cout<<"Run new calibration? [y/n]";
    std::cin>> hCalib;

    if(hCalib=="y"){
        this->getCornersV2(this->pylonPic());
    } else if(hCalib=="n"){
        this->getCornersV2(this->loadLoaclimg());
    }
    imgAmt = 1;
    this->pylonPic();






}

std::vector<cv::Mat> ImageProcessing::pylonPic(){
    std::vector<cv::Mat> imgVector;
    imgVector.clear();
    int myExposure = 20000;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera( Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap& nodemap= camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();
        // Create pointers to access the camera Width and Height parameters.
        GenApi::CIntegerPtr width= nodemap.GetNode("Width");
        GenApi::CIntegerPtr height= nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        //camera.MaxNumBuffer = 5;

        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        // Specify the output pixel format.
        formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
        // Create a PylonImage that will be used to create OpenCV images later.
        Pylon::CPylonImage pylonImage;

        // Create an OpenCV image.
        cv::Mat openCvImage;

        // Set exposure to manual
        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto)){
            exposureAuto->FromString("Off");
            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
        if(exposureTime.IsValid()) {
            if(myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax()) {
                exposureTime->SetValue(myExposure);
            }else {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << ">> Exposure has been set with the minimum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }else {

            std::cout << ">> Failed to set exposure value." << std::endl;

        }
        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        while ( camera.IsGrabbing() && imgVector.size()<imgAmt)
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            if(cv::waitKey(1) == 'q'){       //quit
                camera.Close();
                break;
            }

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);
                // Create an OpenCV image from a pylon image.
                openCvImage= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());
                //If calibration has run _calibrationMat will be bigger than 0 and this code runs
                if(_calibrationMat.size()>0){
                    cv::Mat mapX, mapY;
                    mapX = _calibrationMat[0].clone();
                    mapY = _calibrationMat[1].clone();
                    cv::Mat imgUndistorted;
                    cv::remap(openCvImage, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
                    cv::Mat grayUndist;
                    cv::cvtColor(imgUndistorted, grayUndist, cv::COLOR_RGB2GRAY);
                    if(!autoImg ){cv::imshow( "Undistorted image"+std::to_string(imgVector.size()), grayUndist);}
                    if(cv::waitKey(1) == 'p' || autoImg){
                        cv::Mat tmp=imgUndistorted.clone();
                        imgVector.push_back(tmp);
                        if(showimg || !autoImg){cv::destroyWindow("Undistorted image"+std::to_string(imgVector.size()-1));}
                        if(imgVector.size()>=imgAmt){
                            camera.Close();
                            break;
                        }
                    }
                }
                //If not calibrated take X amount op pics
                else{
//                    cv::Rect iCrop(100, 10, 900, 600);
//                    cv::Mat cropImg = openCvImage(iCrop);
                    cv::imshow( "myWindow"+std::to_string(imgVector.size()), openCvImage);}
                    if(cv::waitKey(1) == 'p'){
                        cv::Mat tmp=openCvImage.clone();
                        imgVector.push_back(tmp);
                        cv::destroyWindow("myWindow"+std::to_string(imgVector.size()-1));
                        if(imgVector.size()>=imgAmt){
                            camera.Close();
                        }
                    }
            }
            else
            {
                std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            }
        }
    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
    }
    return imgVector;
}

cv::Point ImageProcessing::ballDetection() {
    cv::Mat src = this->pylonPic().at(0);
    std::vector<cv::Point> points;

    cv::Mat gray;
    cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Vec3f> circles;
    HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, 1, 30); // change the last two parameters
    // (min_radius & max_radius) to detect larger circles

    for( size_t i = 0; i < circles.size(); i++ ) {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // Store points
        points.push_back(center);

        // circle center
        circle( gray, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        circle( gray, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);

        cv::imshow("showImage" + std::to_string(i), src);
        cv::waitKey();
        cv::destroyWindow("showImage" + std::to_string(i));
    }

    if(points.size() > 1) {
        std::cout << "Put in the correct picture number: ";
        std::string input;
        std::cin >> input;
        std::cout << "Picture number " + input + " have been selected" << std::endl;

        return points.at(std::stoi(input));
    } else
        return points.at(0);

}

void ImageProcessing::getCornersV2(std::vector<cv::Mat> imgVec)
{

    for(int i = 0; i<imgVec.size();i++){
        cv::imwrite("../app/imageProcessing/images/calibration" + std::to_string(i) + ".jpg", imgVec.at(i));
    }

    std::vector<std::vector<cv::Point2f>> q(imgVec.size());

    std::vector<std::vector<cv::Point3f>> Q;
    // 1. Generate checkerboard (world) coordinates Q. The board has 25 x 18
    // fields with a size of 15x15mm

    //int checkerBoard[2] = {6,9};
    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i = 1; i<=BoardSize.height; i++){
        for(int j = 1; j<=BoardSize.width; j++){
            objp.push_back(cv::Point3f(j,i,0));
        }
    }

    std::vector<cv::Point2f> imgPoint;
    // Detect feature points
    std::size_t i = 0;
    for (auto const &f : imgVec) {
        //std::cout << std::string(f) << std::endl;

        // 2. Read in the image an call cv::findChessboardCorners()
        cv::Mat img = f.clone();
        cv::Mat gray;

        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

        bool patternFound = cv::findChessboardCorners(gray, BoardSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        // 2. Use cv::cornerSubPix() to refine the found corner detections
        if(patternFound){
            cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            Q.push_back(objp);
        }
        if(showimg){
            // Display
            cv::drawChessboardCorners(img, BoardSize, q[i], patternFound);
            cv::imshow("chessboard detection", img);
            cv::waitKey(0);
        }
        i++;
    }


    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
            cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(1440,1080);

    std::cout << "Calibrating..." << std::endl;
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;

    // Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);
    // Show lens corrected images
    cv::imwrite("../app/imageProcessing/images/mapX.jpg", mapX);
    cv::imwrite("../app/imageProcessing/images/mapY.jpg", mapY);
    for (auto const &f : imgVec) {

        //std::cout << std::string(f) << std::endl;

        cv::Mat img = f.clone();
        //std::cout<<"test"<<std::endl;
        cv::Mat imgUndistorted;
        // 5. Remap the image using the precomputed interpolation maps.
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
        // Display
        if(showimg){
            cv::imshow("undistorted image", imgUndistorted);
            cv::waitKey(0);
        }
    }
    _calibrationMat.push_back(mapX);
    _calibrationMat.push_back(mapY);
}

std::vector<cv::Mat> ImageProcessing::loadLoaclimg()
{
    std::vector<cv::Mat> imgVec;
    std::vector<cv::String> fileNames;
    cv::glob("../app/imageProcessing/images/calibration*.jpg", fileNames, false);
    for(int i = 0; i<fileNames.size();i++){
        imgVec.push_back(cv::imread(fileNames[i]));
        //cv::imshow( "myWindow" +std::to_string(i), imgVec[i]);
        //cv::waitKey(0);
    }
    return imgVec;
}





