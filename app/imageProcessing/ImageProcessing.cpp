#include "ImageProcessing.h"

ImageProcessing::ImageProcessing(){}

void ImageProcessing::calibrate()
{
    std::string input;
    std::cout<<"Run new calibration? [y/n]";
    std::cin>> input;

    if(input=="y"){
        this->chessboardDetection(this->pylonPic());
        std::cout << "Ready for cropping? (y)" << std::endl;
        std::cin >> input;
    } else if(input=="n")
        this->chessboardDetection(this->loadLocalImg());

    input = "";
    imgAmt = 1;
    std::vector<cv::Point> tempPoints;
    cv::Mat tmp = cv::imread("../app/imageProcessing/images/cornerDetection.jpg");
    this->cornersHoughCircles(tmp);
//    tmp = this->cropImg(tmp);
//    tmp = this->rotateImg(tmp);

}

std::vector<double> ImageProcessing::getBallCoords() {
    std::string input;
    std::vector<double> realCoords{0};
    cv::destroyAllWindows();
    while (true){
        std::cout <<"ready for ball finding? (y)" <<std::endl;
        std::cin >> input;

        if (input == "y") {
            cv::Mat img = cv::imread("../app/imageProcessing/images/cornerDetection.jpg"); //this->pylonPic()[0].clone();
            cv::Mat crop = this->cropImg(img);
            cv::Mat rot = this->rotateImg(crop);
            this->cornersHoughCircles(rot);
            this->rotateImg(rot);
            cv::Point imgPoints = this->ballDetection(crop);
            while(imgPoints.x == -1 && imgPoints.y==-1){
                std::cout<<"take new image when ready, press p"<<std::endl;
                if(cv::waitKey()=='p'){
                    imgPoints = this->ballDetection(this->cropImg(this->pylonPic()[0]));
                }
            }
            realCoords = this->coordConvert(imgPoints,crop);
            break;
        }
    }

    std::cout << "realCoords" << std::endl;
    std::cout << realCoords.at(0) << std::endl;
    std::cout << realCoords.at(1) << std::endl << std::endl;

    return realCoords;
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

                    if(!autoImg ){cv::imshow( "Undistorted image"+std::to_string(imgVector.size()), imgUndistorted);}
                    if(cv::waitKey(1) == 'p' || autoImg){
                        //cv::Mat tmp=imgUndistorted.clone();
                        imgVector.push_back(imgUndistorted);
                        if(showimg || !autoImg){cv::destroyWindow("Undistorted image"+std::to_string(imgVector.size()-1));}
                        if(imgVector.size()>=imgAmt){
                            camera.Close();
                            break;
                        }
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
        cv::Mat img = f.clone();
        cv::Mat imgUndistorted;
        // 5. Remap the image using the precomputed interpolation maps.
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
        // Display
        if(showimg){
            cv::imshow("undistorted image", imgUndistorted);
            cv::waitKey(0);
        }
    }
    cv::destroyAllWindows();
    _calibrationMat.push_back(mapX);
    _calibrationMat.push_back(mapY);
}

std::vector<cv::Mat> ImageProcessing::loadLocalImg()
{
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
    int y = (cropCornerPoints[0].y+cropCornerPoints[1].y)/2; // Average Y height
    int x = (cropCornerPoints[1].x+cropCornerPoints[2].x)/2;// Average x with
    cv::Mat crop = img(cv::Range(y,cropCornerPoints[2].y),cv::Range(cropCornerPoints[0].x, x)).clone(); // Slicing to crop the image

    return crop;
}

cv::Mat ImageProcessing::Threshold(cv::Mat image)
{
    cv::Mat img, crop, gray, out, outNorm, outNormSc;
    img=image.clone();

    //MANUAL THRESHOLDING - RGB
    crop = img(cv::Range(400,800),cv::Range(500,900)).clone();
    cv::Mat imgMod = img.clone();
    //cv::vec3b m = cv::mean(crop);
    uchar deviation = 30;

    long avrgTmp1=0, avrgTmp2=0, avrgTmp3=0;
    long iter=1;

    for (int r = 0; r < crop.rows; r++) {
        for (int c = 0; c < crop.cols; c++) {
            avrgTmp1 = avrgTmp1+crop.at<cv::Vec3b>(r,c)[0];
            avrgTmp2 = avrgTmp2+crop.at<cv::Vec3b>(r,c)[1];
            avrgTmp3 = avrgTmp3+crop.at<cv::Vec3b>(r,c)[2];
            iter++;
        }
    }
    avrgTmp1=avrgTmp1/iter;
    avrgTmp2=avrgTmp2/iter;
    avrgTmp3=avrgTmp3/iter;


    for (int r = 0; r < imgMod.rows; r++) {
        for (int c = 0; c < imgMod.cols; c++) {


            if(imgMod.at<cv::Vec3b>(r,c)[0] > avrgTmp1 + deviation){imgMod.at<cv::Vec3b>(r,c)[0] = imgMod.at<cv::Vec3b>(r,c)[0]*0;
                imgMod.at<cv::Vec3b>(r,c)[1] = imgMod.at<cv::Vec3b>(r,c)[1]*0;
                imgMod.at<cv::Vec3b>(r,c)[2] = imgMod.at<cv::Vec3b>(r,c)[2]*0;}

            else if(imgMod.at<cv::Vec3b>(r,c)[1] > avrgTmp2 + deviation){imgMod.at<cv::Vec3b>(r,c)[1] = imgMod.at<cv::Vec3b>(r,c)[1]*0;
                imgMod.at<cv::Vec3b>(r,c)[0] = imgMod.at<cv::Vec3b>(r,c)[0]*0;
                imgMod.at<cv::Vec3b>(r,c)[2] = imgMod.at<cv::Vec3b>(r,c)[2]*0;}

            else if(imgMod.at<cv::Vec3b>(r,c)[2] > avrgTmp3 + deviation){imgMod.at<cv::Vec3b>(r,c)[2] = imgMod.at<cv::Vec3b>(r,c)[2]*0;
                imgMod.at<cv::Vec3b>(r,c)[0] = imgMod.at<cv::Vec3b>(r,c)[0]*0;
                imgMod.at<cv::Vec3b>(r,c)[1] = imgMod.at<cv::Vec3b>(r,c)[1]*0;}
        }
    }
    cv::imshow("bin", imgMod);
    cv::waitKey();


    //MANUAL THRESHOLDING - GRAY
    cv::cvtColor(imgMod, gray, cv::COLOR_BGR2GRAY);

    crop = gray(cv::Range(400,800),cv::Range(500,900)).clone();
    cv::Mat grayMod = gray.clone();
    cv::Scalar m = cv::mean(crop);

    for (int r = 0; r < gray.rows; r++) {
        for (int c = 0; c < gray.cols; c++) {
            if(gray.at<uchar>(r,c)> m[0]+deviation){grayMod.at<uchar>(r,c) = 255;}
            else if(gray.at<uchar>(r,c)< m[0]-deviation){grayMod.at<uchar>(r,c) = 255;}
            else{grayMod.at<uchar>(r,c)=0;}


        }
    }
    cv::imshow("bin", grayMod);
    cv::waitKey();

    return grayMod;
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

    std::cout << "unscaled x = " << std::to_string(x1) << ", unscaled y = " << std::to_string(y1);
    std::cout << ", which means the hypotenuse = " << std::to_string(z1) << std::endl;
    std::cout << "scaled x = " << std::to_string(x2) << ", unscaled y = " << std::to_string(y2);
    std::cout << ", which means the hypotenuse = " << std::to_string(z2) << std::endl;

    return points;
}

void ImageProcessing::lastStand(cv::Mat img)
{
    cv::Mat image, gray, crop;
    cv::Mat output, output_norm, output_norm_scaled;
    image = img.clone();
    crop = image(cv::Range(350,700),cv::Range(300,1100)).clone();

    // Detecting corners using the goodFeaturesToTrack built in function
    std::vector<cv::Point2f> corners;
    goodFeaturesToTrack(crop,
                        corners,
                        100,            // Max corners to detect
                        0.1,           // Minimal quality of corners
                        10,             // Minimum Euclidean distance between the returned corners
                        cv::Mat(),          // Optional region of interest
                        10,              // Size of an average block for computing a derivative covariation matrix over each pixel neighbothood
                        false,          // Use Harri Detector or cornerMinEigenVal - Like when you create your own
                        0.04);          // Free parameter for the Harris detector


    // Drawing a circle around corners
    for (size_t i = 0; i < corners.size(); i++){
        circle(crop, corners[i], 4, cv::Scalar(0, 255, 0), 2, 8, 0);
    }

    // Displaying the result
    cv::imshow("Output Shi-Tomasi", crop);
    cv::waitKey();
}

cv::Mat ImageProcessing::rotateImg(cv::Mat img)
{
    double pi = 3.14159265358979323846;
    double x1=cropCornerPoints[0].x, x2 = cropCornerPoints[1].x, y1 = cropCornerPoints[0].y, y2 = cropCornerPoints[1].y;
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

    cv::Mat rMatrix = cv::getRotationMatrix2D(C,theta1-theta2,1), rImage;
    cv::warpAffine(img,rImage,rMatrix,img.size());
    cv::destroyAllWindows();
    cv::imshow("rotated", rImage);
    cv::waitKey();
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

cv::Point ImageProcessing::ballDetection(cv::Mat src) {
    std::vector<cv::Point> points;
    std::vector<cv::Mat> bufferImages;
    cv::Mat src_grey;
    cvtColor(src, src_grey, cv::COLOR_BGR2GRAY);
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
        for (unsigned int i = 0; i < bufferImages.size(); i++) {
            cv::imshow("showImage" + std::to_string(i), bufferImages.at(i));
            cv::waitKey();
            cv::destroyWindow("showImage" + std::to_string(i));
        }
        std::cout << "Put in the correct picture number: ";
        std::string input;
        std::cin >> input;
        std::cout << "Picture number " + input + " have been selected" << std::endl;

        return points.at(std::stoi(input));
    } else if (points.size() == 1){

        cv::imshow("testo", src_grey);
        cv::waitKey();


        return points.at(0);
    } else {
        std::cout << "No table tennis ball found!" << std::endl;
        cv::imshow("PointsNotFound", src_grey);
        cv::waitKey();
        return cv::Point(-1, -1);
    }
}

void ImageProcessing::cornersHoughCircles(cv::Mat src){
    std::cout<<"make new corner callibration? [y/n]"<<std::endl;
    std::string tmp;
    std::cin>> tmp;
    cropCornerPoints.clear();
    if(tmp == "n")
        src = cv::imread("../app/imageProcessing/images/cornerDetection.jpg");

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

    cropCornerPoints.push_back(points.at(minXIndx));

    if (points.size() < 3) {
        //std::cout<<"not enough nips found, press enter to continue";
        cv::imshow("not enough corners found, press enter to continue", image_bgr);
        cv::waitKey();
        cv::destroyAllWindows();
        this->cornersHoughCircles(this->pylonPic()[0]);
    }
    else if((minXIndx == 0 && maxYIndx == 1) || (minXIndx == 1 && maxYIndx == 0))
        cropCornerPoints.push_back(points.at(2));
    else if ((minXIndx == 1 && maxYIndx == 2) || (minXIndx == 2 && maxYIndx == 1))
        cropCornerPoints.push_back(points.at(0));
    else if ((minXIndx == 2 && maxYIndx == 0) || (minXIndx == 0 && maxYIndx == 2))
        cropCornerPoints.push_back(points.at(1));
    else if (minXIndx > 2 || maxYIndx > 2) {
        //std::cout<<"not enough nips found, press enter to continue";
        cv::imshow("not enough corners found, press enter to continue", image_bgr);
        cv::waitKey();
        cv::destroyAllWindows();
        this->cornersHoughCircles(this->pylonPic()[0]);
    }

    cropCornerPoints.push_back(points.at(maxYIndx));
    if(cropCornerPoints.size()>2){

    cropCornerPoints.at(0).x -= 20;
    cropCornerPoints.at(0).y -= 20;
    cropCornerPoints.at(1).x += 20;
    cropCornerPoints.at(1).y -= 20;
    cropCornerPoints.at(2).x += 20;
    cropCornerPoints.at(2).y += 20;
    }else {
        //std::cout<<"not enough nips found, press enter to continue";
        cv::imshow("not enough corners found, press enter to continue", image_bgr);
        cv::waitKey();
        cv::destroyAllWindows();
        this->cornersHoughCircles(this->pylonPic()[0]);
    }
}
