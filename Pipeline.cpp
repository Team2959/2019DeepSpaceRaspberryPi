#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "Pipeline.hpp"
#include "../2019RaspPIRoboRioShared/SharedNames.h"

Pipeline::Pipeline(std::shared_ptr<nt::NetworkTable> networkTable) : m_networkTable{ networkTable } { }

void Pipeline::FindCargo(cv::Mat& mat)
{
    cv::Mat hsvImage;

    // convert from Red-Green-Blue to Hue-Saturation-Value
    cv::cvtColor(mat, hsvImage, cv::ColorConversionCodes::COLOR_BGR2HSV);

/*
    //  Find only pixel that are in range of the HSV value
    //   in this case orange (0-60) with all saturations and upper have values
    cv::Mat     threshold;
    cv::inRange(hsvImage, cv::Scalar(0, 0, 128), cv::Scalar(60, 255, 255), threshold);

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

    imwrite("o4.bmp", image); */
}

void Pipeline::IncrementFrameNumber()
{
    ++m_frameNumber;
    std::cout << "Frame Number = " << m_frameNumber << '\n';
    m_networkTable->PutNumber(FRAME_NUMBER, m_frameNumber);

    // Flush cout since we are on a separate thread here.
    std::cout.flush();                       
}

void Pipeline::Process(cv::Mat& mat)
{
    IncrementFrameNumber();
    FindCargo(mat);
}

/*
    std::vector<double> numbers;
    numbers.push_back(0.25);
    numbers.push_back(0.3);
    numbers.push_back(0.5);
    numbers.push_back(0.15);

    _networkTable->PutNumberArray(TARGET_COORDS, numbers);
*/

/*

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