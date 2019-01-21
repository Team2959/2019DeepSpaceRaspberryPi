#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <iostream>

#include "Pipeline.hpp"
#include "../2019RaspPIRoboRioShared/SharedNames.h"

Pipeline::Pipeline(std::shared_ptr<nt::NetworkTable> networkTable)
{
    _networkTable = networkTable;
}

void Pipeline::Process(cv::Mat& mat)
{
    ++_value;

    _networkTable->PutNumber(FRAME_NUMBER, _value);

std::vector<double> numbers;
numbers.push_back(0.0);
numbers.push_back(1.0);
numbers.push_back(2.0);
numbers.push_back(3.0);

    _networkTable->PutNumberArray(TARGET_COORDS, numbers);


}

/*

void FindOrange()
{
    // read the image from a file (can be from a live frame of camera data)
    auto    image{imread("FindOrange.bmp",CV_LOAD_IMAGE_COLOR)};
    Mat     converted;

    // convert from Red-Green-Blue to Hue-Saturation-Value
    cvtColor(image, converted, ColorConversionCodes::COLOR_BGR2HSV);

    imwrite("o1.bmp", converted);

    //  Find only pixel that are in range of the HSV value
    //   in this case orange (0-60) with all saturations and upper have values
    Mat     threshold;
    inRange(converted, cv::Scalar(0, 0, 128), cv::Scalar(60, 255, 255), threshold);

    imwrite("o2.bmp", threshold);

    std::vector<std::vector<cv::Point> > contours;

    // get the bounding pixels for each area that were the in range 
    cv::findContours(threshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    std::ofstream    output{"o3.txt"};

    for(auto& contour : contours)
    {
        // change the contour pixels to be a rectangle
        auto    rect{cv::boundingRect(contour)};

        // draw the rectangle on the original image
        cv::rectangle(image, rect, cv::Scalar(255, 0, 0), 4);

        output << rect.x << ',' << rect.y << ',' << rect.width << ',' << rect.height << "\r\n";
    } 

    output.close();

    imwrite("o4.bmp", image);
}

void FindRectangles()
{
// Adapted from https://stackoverflow.com/questions/34237253/detect-centre-and-angle-of-rectangles-in-an-image-using-opencv

    // Loads image from file
    Mat image(imread("Rectangles.bmp",CV_LOAD_IMAGE_COLOR));


    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(image,gray,CV_BGR2GRAY);

    imwrite("R1.bmp", gray);

    // extract contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Go through each countor
    for(int i=0; i<contours.size(); ++i)
    {
        // fit bounding rectangle around contour
        cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

        // read points and angle
        cv::Point2f rect_points[4]; 
        rotatedRect.points( rect_points );

        float  angle = rotatedRect.angle; // angle

        // read center of rotated rect
        cv::Point2f center = rotatedRect.center; // center

        // draw rotated rect
        for(unsigned int j=0; j<4; ++j)
            cv::line(image, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,255,0));

        // draw center and print text
        std::stringstream ss;   ss << angle; // convert float to string
        cv::circle(image, center, 5, cv::Scalar(0,255,0)); // draw center
        cv::putText(image, ss.str(), center + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255)); // print angle
    }

    // drawContours(image, contours, -1,Scalar(0,128,180), 10);

    imwrite("R2.bmp", image);
}
*/