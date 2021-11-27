#include "ImageProcessing.h"

ImageProcessing::ImageProcessing(){
    std::cout << "Imageprocessing app initializing..." << std::endl;

    cv::Mat calibrationImage = cv::imread("../app/imageProcessing/images/calibration29.jpg");
    cv::Mat cornerDetectionImage = cv::imread("../app/imageProcessing/images/cornerDetection.jpg");

    if (calibrationImage.empty()) {
        std::cout << "You have no chessboard calibration images." << std::endl;
        std::cout << "When ready to run calibration press enter.";
        std::cin.get();
        this->chessboardDetection(this->grabImage(30));
    } else {
        this->chessboardDetection(this->loadCalibImages());
    }

    if (cornerDetectionImage.empty()) {
        std::cout << "You have no corner calibration image." << std::endl;
        std::cout << "Place the red dots on the table and press enter.";
        std::cin.get();
        cv::imwrite("../app/imageProcessing/images/cornerDetection.jpg", this->grabImage(1)[0]);
        this->cornersHoughCircles(cv::imread("../app/imageProcessing/images/cornerDetection.jpg"));
    } else {
        this->cornersHoughCircles(cv::imread("../app/imageProcessing/images/cornerDetection.jpg"));
    }
}

void ImageProcessing::calibrate() {
    while (true) {
        std::cout << std::endl;
        std::cout << "Select the desired calibration: " << std::endl;
        std::cout << "1: Run new chessboard calibration" << std::endl;
        std::cout << "2: run new corner calibration" << std::endl;
        std::cout << "press q when done calibrating" << std::endl;
        std::cout << "Input number and press enter: ";
        std::string input;
        std::cin >> input;

        if (input == "1") {
            std::cout << "When ready to run calibration press enter.";
            std::cin.get();
            this->chessboardDetection(this->grabImage(30));
        } else if (input == "2") {
            std::cout << "Place the red dots on the table and press enter.";
            std::cin.get();
            cv::imwrite("../app/imageProcessing/images/cornerDetection.jpg", this->grabImage(1)[0]);
            this->cornersHoughCircles(cv::imread("../app/imageProcessing/images/cornerDetection.jpg"));
        } else if (input == "q") {
            break;
        }
    }
}

std::vector<double> ImageProcessing::getBallCoords() {
    cv::Mat   newImage  = this->rotateImg(this->cropImg(this->grabImage(1)[0]));
    cv::Point imgPoints = this->ballDetection(newImage);

    while(imgPoints.x == -1 || imgPoints.y==-1){
        std::cout << "take new image, when ready press p" << std::endl;

        if(cv::waitKey()=='p'){
            newImage    = this->rotateImg(this->cropImg(this->grabImage(1)[0]));
            imgPoints   = this->ballDetection(newImage);
        }
    }

    std::vector<double> realCoords = this->coordConvert(imgPoints, newImage);

    return realCoords;
}

std::vector<cv::Mat> ImageProcessing::grabImage(int imgAmt){
    std::vector<cv::Mat> imgVector;
    imgVector.clear();
    int myExposure = 20000;

    if (imgAmt == 1) {
        std::vector<cv::Mat> temp;
        temp.push_back(cv::imread("../app/imageProcessing/images/cornerDetection.jpg"));
        return temp;
    }

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
            //            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        //        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
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


            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                if(imgVector.size()>=imgAmt || cv::waitKey(1) == 'q'){
                    camera.Close();
                    break;
                }
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
                    //cv::Mat crop = this->cropImg(imgUndistorted).clone();

                    //cv::Mat tmp=imgUndistorted.clone();
                    imgVector.push_back(imgUndistorted);
                    if(imgVector.size()>=imgAmt){
                        camera.Close();
                        break;
                    }
                }
                //If not calibrated take X amount op pics
                else{
                    cv::imshow( "myWindow"+std::to_string(imgVector.size()), openCvImage);}
                if(cv::waitKey(50) == 'p'){
                    cv::Mat tmp=openCvImage.clone();
                    imgVector.push_back(tmp);
                    cv::destroyWindow("myWindow"+std::to_string(imgVector.size()-1));
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

void ImageProcessing::chessboardDetection(std::vector<cv::Mat> imgVec) {
    const cv::Size BoardSize{6,9};

    for(int i = 0; i<imgVec.size();i++){
        cv::imwrite("../app/imageProcessing/images/calibration" + std::to_string(i) + ".jpg", imgVec.at(i));
    }

    std::vector<std::vector<cv::Point2f>> q(imgVec.size());
    std::vector<std::vector<cv::Point3f>> Q;
    // 1. Generate checkerboard (world) coordinates Q. The board has 25 x 18
    // fields with a size of 15x15mm
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
        // 2. Read in the image an call cv::findChessboardCorners()
        cv::Mat img = f.clone();
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

        bool patternFound = cv::findChessboardCorners(gray, BoardSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        if(!patternFound){
            std::cout<<"No chessboard found on image" + std::to_string(i)<<std::endl;
            std::cout<<"Take new images!"<<std::endl;
            break;
        }

        // 2. Use cv::cornerSubPix() to refine the found corner detections
        if(patternFound){
            cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            Q.push_back(objp);
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

    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

    // Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);
    // Show lens corrected images
    cv::imwrite("../app/imageProcessing/images/mapX.jpg", mapX);
    cv::imwrite("../app/imageProcessing/images/mapY.jpg", mapY);
    for (auto const &f : imgVec) {
        cv::Mat img = f.clone();
        cv::Mat imgUndistorted;
        // 5. Remap the image using the precomputed interpolation maps.
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
    }

    cv::destroyAllWindows();
    _calibrationMat.push_back(mapX);
    _calibrationMat.push_back(mapY);
}

std::vector<cv::Mat> ImageProcessing::loadCalibImages() {
    std::vector<cv::Mat> imgVec;
    std::vector<cv::String> fileNames;
    cv::glob("../app/imageProcessing/images/calibration*.jpg", fileNames, false);
    for(int i = 0; i<fileNames.size();i++){
        imgVec.push_back(cv::imread(fileNames[i]));
    }
    return imgVec;
}


cv::Mat ImageProcessing::cropImg(cv::Mat img)
{
    // first upper left
    //seckond upper right
    int y = (_cropCornerPoints[0].y+_cropCornerPoints[1].y)/2; // Average Y height
    int x = (_cropCornerPoints[1].x+_cropCornerPoints[2].x)/2;// Average x with
    cv::Mat crop = img(cv::Range(y,_cropCornerPoints[2].y),cv::Range(_cropCornerPoints[0].x, x)).clone(); // Slicing to crop the image

    return crop;
}

std::vector<double> ImageProcessing::coordConvert(cv::Point imgPos, cv::Mat img)
{
    //cam height 139 cm
    float x1, x2, y1, y2, z1, z2, maxZ, theta;
    float xWidth = img.cols, yWidth = img.rows;
    float realX = 80, realY = 75; //cm
    float lengthPerPixelX = realX/xWidth;
    float lengthPerPixelY = realY/yWidth;

    maxZ = sqrt(pow(realX,2.0)+pow(realY,2.0));
    x1 = imgPos.x*lengthPerPixelX;
    y1 = imgPos.y*lengthPerPixelY;
    z1 = sqrt(pow(x1,2.0)+pow(y1,2.0));

    theta = acos(x1/z1);
    z2 = z1 * (1 + 0.009 * (z1 / maxZ)); // times the scaling times the percent of max length
    x2 = cos(theta) * z2;
    y2 = sin(theta) * z2;

    std::vector<double> points;
    points.push_back(x2/100);
    points.push_back(y2/100);

    std::cout << "coordinates for the object is: (" << std::to_string(x2) << "; " << std::to_string(y2) << ")" << std::endl;

    return points;
}

cv::Mat ImageProcessing::rotateImg(cv::Mat img) {
    double pi = 3.14159265358979323846;
    double x1=_cropCornerPoints[0].x, x2 = _cropCornerPoints[1].x, y1 = _cropCornerPoints[0].y, y2 = _cropCornerPoints[1].y;
    double a = (y2-y1)/(x2-x1), b = y1 -x1 * a;
    double fWidth = a*img.cols+b;
    cv::Point2f A1(0,b);
    cv::Point2f A2(img.cols,fWidth);
    cv::Point2f B(img.cols/2,img.rows);
    cv::Point2f C(img.cols/2,img.rows/2);

    double b1 = cv::norm(cv::Mat(A1),cv::Mat(C)), b2 = cv::norm(cv::Mat(A2),cv::Mat(C));
    double c1 = cv::norm(cv::Mat(A1),cv::Mat(B)), c2 = cv::norm(cv::Mat(A2),cv::Mat(B));
    double a1 = cv::norm(cv::Mat(B),cv::Mat(C));
    double theta1 = acos((a1*a1+b1*b1-c1*c1)/(2*b1*a1))*180/pi;
    double theta2 = acos((a1*a1+b2*b2-c2*c2)/(2*b2*a1))*180/pi;

    cv::Mat rMatrix = cv::getRotationMatrix2D(C,(theta1-theta2)/2,1), rImage;
    cv::warpAffine(img,rImage,rMatrix,img.size());
    cv::imwrite("../app/imageProcessing/images/rotatedImg.jpg", rImage);
    return rImage;
}

std::vector<cv::Point> ImageProcessing::cornersTempleMatching(cv::Mat img)
{
    std::vector<cv::Mat> results, tpl;
    std::vector<cv::Point> points;
    cv::Mat res, gref, gtpl;

    cv::Mat tplR = cv::imread("../app/imageProcessing/images/tokenRight.jpg").clone();
    cv::Mat tplL = cv::imread("../app/imageProcessing/images/tokenLeft1.jpg").clone();
    cv::Mat ref = img.clone();

    tpl.push_back( tplR.clone());
    tpl.push_back( tplL.clone());

    for(int i=0;i<tpl.size();i++){
        cv::cvtColor(ref, gref, cv::COLOR_BGR2GRAY);
        cv::cvtColor(tpl[i], gtpl, cv::COLOR_BGR2GRAY);

        const int low_canny = 30;
        cv::Canny(gref, gref, low_canny, low_canny*3);
        cv::Canny(gtpl, gtpl, low_canny, low_canny*3);

        cv::Mat res_32f(ref.rows - tpl[i].rows + 1, ref.cols - tpl[i].cols + 1, CV_32FC1);
        matchTemplate(gref, gtpl, res_32f, cv::TM_CCOEFF_NORMED);

        res_32f.convertTo(res, CV_8U, 255.0);

        int size = ((tpl[i].cols + tpl[i].rows) / 4) * 2 + 1; //force size to be odd
        cv::adaptiveThreshold(res, res, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, size, -100);

        double minval, maxval;
        cv::Point minloc, maxloc;
        cv::minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);
        points.push_back(cv::Point(maxloc.x + (tpl[i].cols/2), maxloc.y + (tpl[i].rows/2)));
        cv::circle( ref, points[i], 10, cv::Scalar(0,100,255), 5, cv::LINE_AA); // Draws center
    }

    cv::imshow("Correct corners? (y/n)", ref);
    return points;
}

cv::Point ImageProcessing::ballDetection(cv::Mat img) {
    std::vector<cv::Point> points;
    std::vector<cv::Mat> bufferImages;
    cv::Mat src_grey;
    cvtColor(img, src_grey, cv::COLOR_BGR2GRAY);
    cv::medianBlur(src_grey, src_grey, 5);

    std::vector<cv::Vec3f> circles;
    HoughCircles(src_grey, circles, cv::HOUGH_GRADIENT, 1, src_grey.rows/16, 100, 30, 15, 30); // The last two parameters is min & max radius
    for( size_t i = 0; i < circles.size(); i++ ) {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        points.push_back(center);
        circle( src_grey, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA); // Draws center
        circle( src_grey, center, c[2], cv::Scalar(255,0,255), 3, cv::LINE_AA); // Draws radius
        bufferImages.push_back(src_grey.clone());
    }
    if(points.size() > 1) {
        std::cout << "There have been found " << bufferImages.size() << " balls, press enter to show the pictures";
        std::cin.get(); std::cin.get(); // <- It works.. Don't question this!

        for (unsigned int i = 0; i < bufferImages.size(); i++) {
            cv::imshow("showImage" + std::to_string(i), bufferImages.at(i));
            cv::waitKey();
        }

        cv::destroyAllWindows();
        std::cout << "The correct picture number is: ";
        std::string input;
        std::cin >> input;

        return points.at(std::stoi(input));
    } else if (points.size() == 1){
        cv::imshow("Ball found", src_grey);
        cv::waitKey();
        cv::destroyAllWindows();

        return points.at(0);
    } else {
        std::cout << "No table tennis ball found!" << std::endl;
        cv::imshow("PointsNotFound", src_grey);
        cv::waitKey();
        return cv::Point(-1, -1);
    }
}

cv::Point ImageProcessing::liveHoughCircles(cv::Mat img) {
    std::vector<cv::Point> points;
    std::vector<cv::Mat> bufferImages;
    cv::Mat img_grey;
    cvtColor(img, img_grey, cv::COLOR_BGR2GRAY);
    cv::medianBlur(img_grey, img_grey, 5);
    std::vector<cv::Vec3f> balls;
    std::vector<cv::Vec3f> circles;
    HoughCircles(img_grey, balls, cv::HOUGH_GRADIENT, 1, img_grey.rows/16, 100, 30, 15, 29); // The last two parameters is min & max radius
    HoughCircles(img_grey, circles, cv::HOUGH_GRADIENT, 1, img_grey.rows/16, 100, 30, 30, 40); // The last two parameters is min & max radius
    cv::Point center;

    for( size_t i = 0; i < balls.size(); i++ ) {
        cv::Vec3i c = balls[i];
        center = cv::Point(c[0], c[1]);
        points.push_back(center);
        cv::putText(img_grey, "ball", cv::Point(center.x - 20, center.y - 20), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 120, 0));
        circle( img_grey, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA); // Draws center
        circle( img_grey, center, c[2], cv::Scalar(255,0,255), 3, cv::LINE_AA); // Draws radius
        bufferImages.push_back(img_grey.clone());
    }

    for( size_t i = 0; i < circles.size(); i++ ) {
        cv::Vec3i c = circles[i];
        center = cv::Point(c[0], c[1]);
        points.push_back(center);
        cv::putText(img_grey, "circle", cv::Point(center.x - 20, center.y - 20), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 120, 0));
        circle( img_grey, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA); // Draws center
        circle( img_grey, center, c[2], cv::Scalar(255,0,255), 3, cv::LINE_AA); // Draws radius
        bufferImages.push_back(img_grey.clone());
    }

    return center;
}

void ImageProcessing::cornersHoughCircles(cv::Mat src){
    _cropCornerPoints.clear();

    std::vector<cv::Point> points;
    cv::Mat image_hsv, image_bgr;
    image_bgr = src;
    cv::cvtColor(src, image_hsv, cv::COLOR_BGR2HSV);
    cv::medianBlur(image_hsv, image_hsv, 9);

    cv::Mat lower_red_hue_range, upper_red_hue_range, red_hue_image;

    cv::inRange(image_hsv, cv::Scalar(0, 100, 100), cv::Scalar(30, 255, 255), lower_red_hue_range);
    cv::inRange(image_hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(red_hue_image, circles, cv::HOUGH_GRADIENT, 1, red_hue_image.rows/16, 100, 30, 15, 30); // The last two parameters is min & max radius

    if(circles.size() == 0) std::exit(-1);
    for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
        cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
        cv::circle(image_bgr, center, 20, cv::Scalar(0, 255, 0), 2);
        points.push_back(center);
    }

    double minX{-1}, maxY{-1};
    int minXIndx{-1}, maxYIndx{-1};

    for (unsigned int i = 0; i < points.size(); i++) {
        if (minX > points.at(i).x) {
            minX = points.at(i).x;
            minXIndx = i;
        } else if (minX == -1) {
            minX = points.at(i).x;
            minXIndx = i;
        }

        if (maxY < points.at(i).y) {
            maxY = points.at(i).y;
            maxYIndx = i;
        }
    }

    _cropCornerPoints.push_back(points.at(minXIndx));

    if (points.size() < 3) {
        //std::cout<<"not enough nips found, press enter to continue";
        cv::imshow("not enough corners found, press enter to continue", image_bgr);
        cv::waitKey();
        cv::destroyAllWindows();
        this->cornersHoughCircles(this->grabImage(1)[0]);
    }
    else if((minXIndx == 0 && maxYIndx == 1) || (minXIndx == 1 && maxYIndx == 0))
        _cropCornerPoints.push_back(points.at(2));
    else if ((minXIndx == 1 && maxYIndx == 2) || (minXIndx == 2 && maxYIndx == 1))
        _cropCornerPoints.push_back(points.at(0));
    else if ((minXIndx == 2 && maxYIndx == 0) || (minXIndx == 0 && maxYIndx == 2))
        _cropCornerPoints.push_back(points.at(1));
    else if (minXIndx > 2 || maxYIndx > 2) {
        //std::cout<<"not enough nips found, press enter to continue";
        cv::imshow("not enough corners found, press enter to continue", image_bgr);
        cv::waitKey();
        cv::destroyAllWindows();
        this->cornersHoughCircles(this->grabImage(1)[0]);
    }

    _cropCornerPoints.push_back(points.at(maxYIndx));
    if(_cropCornerPoints.size()>2){
        _cropCornerPoints.at(0).x -= 20;
        _cropCornerPoints.at(0).y -= 20;
        _cropCornerPoints.at(1).x += 20;
        _cropCornerPoints.at(1).y -= 20;
        _cropCornerPoints.at(2).x += 20;
        _cropCornerPoints.at(2).y += 20;
    }else {
        //std::cout<<"not enough nips found, press enter to continue";
        cv::imshow("not enough corners found, press enter to continue", image_bgr);
        cv::waitKey();
        cv::destroyAllWindows();
        this->cornersHoughCircles(this->grabImage(1)[0]);
    }
}
