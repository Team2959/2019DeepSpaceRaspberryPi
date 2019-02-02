//*****************************************************************************
//*                                                                           *
//* Pipeline.cpp - Pipeline class definition.                                 *
//*                                                                           *
//*****************************************************************************

#include <networktables/NetworkTable.h>
#include <opencv2/opencv.hpp>
#include <vision/VisionPipeline.h>
#include "../2019RaspPIRoboRioShared/SharedNames.h"
#include "Pipeline.hpp"

Rpi2959::Pipeline::Pipeline(std::shared_ptr<nt::NetworkTable> networkTable, wpi::StringRef targetsKey,
  wpi::StringRef frameNumberKey, wpi::StringRef cargoResultsKey, wpi::StringRef floorTapeResultsKey,
  wpi::StringRef hatchResultsKey, wpi::StringRef portTapeResultsKey) :
  m_networkTable{ std::move(networkTable) }, m_targetsKey{ std::move(targetsKey) },
  m_frameNumberKey{ std::move( frameNumberKey) }, m_cargoResultsKey{ std::move( cargoResultsKey ) },
  m_floorTapeResultsKey{ std::move( floorTapeResultsKey ) },
  m_hatchResultsKey{ std::move( hatchResultsKey ) },
  m_portTapeResultsKey{ std::move( portTapeResultsKey ) } { }

cv::Rect2d Rpi2959::Pipeline::FindCargo(const cv::Mat& mat)
{
    cv::Mat hsvImage;

    // convert from Red-Green-Blue to Hue-Saturation-Value
    cv::cvtColor(mat, hsvImage, cv::ColorConversionCodes::COLOR_BGR2HSV);

    //  Find only pixel that are in range of the HSV value
    //   in this case orange (0-60) with upper half saturations and upper half values
    cv::Mat     threshold;
    cv::inRange(hsvImage, cv::Scalar(0, 128, 128), cv::Scalar(60, 255, 255), threshold);

    std::vector<std::vector<cv::Point>> contours;

    // get the bounding pixels for each area that were the in range 
    cv::findContours(threshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    cv::Rect    largestRect;

    for(auto& contour : contours)
    {
        auto    rect{cv::boundingRect(contour)};    // change the contour pixels to be a rectangle
        if(largestRect.area() < rect.area())        // keep the largest rect
            largestRect = rect;
    }
    
    // Convert largest rect in pixel offset coordinates to frame relative coordinates.
    return GetRelativeRect(largestRect, mat);
}

cv::Point2d Rpi2959::Pipeline::GetRelativePoint(const cv::Point& point, const cv::Mat& mat)
{
    return cv::Point2d{ static_cast<double>(point.x) / mat.cols, static_cast<double>(point.y) / mat.rows };
}

cv::Rect2d Rpi2959::Pipeline::GetRelativeRect(const cv::Rect& rect, const cv::Mat& mat)
{
    return cv::Rect2d{ static_cast<double>(rect.x) / mat.cols, static_cast<double>(rect.y) / mat.rows,
        static_cast<double>(rect.width) / mat.cols, static_cast<double>(rect.height) / mat.rows };
}

void Rpi2959::Pipeline::Process(cv::Mat& mat)
{
    m_networkTable->PutNumber(m_frameNumberKey, ++m_frameNumber);
    auto    targets{ static_cast<Rpi2959Shared::ProcessingTargets>(m_networkTable->GetNumber(m_targetsKey, 0.0)) };
    if((targets & Rpi2959Shared::ProcessingTargets::Cargo) == Rpi2959Shared::ProcessingTargets::Cargo)
        m_networkTable->PutNumberArray(m_cargoResultsKey, AsArrayRef(FindCargo(mat)));
    else
        m_networkTable->Delete(m_cargoResultsKey);
}

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