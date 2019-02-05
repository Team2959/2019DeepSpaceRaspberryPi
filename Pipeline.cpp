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
    cv::inRange(hsvImage, cv::Scalar(0, 128, 64), cv::Scalar(60, 255, 196), threshold);

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

std::tuple<cv::Point2d, cv::Point2d> Rpi2959::Pipeline::FindFloorTape(const cv::Mat& mat)
{

}

std::tuple<cv::Point2d, cv::Point2d> Rpi2959::Pipeline::FindPortTape(const cv::Mat& mat)
{
/*    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(mat, gray, CV_BGR2GRAY); */

    cv::Mat hsvImage;

    // convert from Red-Green-Blue to Hue-Saturation-Value
    cv::cvtColor(mat, hsvImage, cv::ColorConversionCodes::COLOR_BGR2HSV);

    //  Find only pixel that are in range of the HSV value
    //   in this case green (40-80) with upper half saturations and upper half values
    cv::Mat     threshold;
    cv::inRange(hsvImage, cv::Scalar(40, 128, 64), cv::Scalar(80, 255, 196), threshold);

    // extract contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(threshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    cv::RotatedRect leftTape;
    cv::RotatedRect rightTape;
    float           leftArea{ 0.0f };
    float           rightArea{ 0.0f };

std::cout << "Tape Contours - " << contours.size() << '\n';
std::cout.flush();
    // Go through each countor
    for(auto& contour : contours)
    {
        // fit bounding rectangle around contour
        cv::RotatedRect rotatedRect = cv::minAreaRect(contour);

        // read points and angle
        std::array<cv::Point2f, 4> rect_points; 
        rotatedRect.points( rect_points.data() );

        auto    orientedPoints{ GetOrientedPoints(rect_points) };

        if((orientedPoints.TopPoint.x - orientedPoints.LeftPoint.x) > 
            (orientedPoints.RightPoint.x - orientedPoints.TopPoint.x))
            rotatedRect.angle += 90.0;

        auto    rectArea{GetArea(orientedPoints)};
        
        // See if rect is a left side tape image
        if((rotatedRect.angle >= 30)&&(rotatedRect.angle <= 60))
        {
            if(rectArea > leftArea)
                leftTape = rotatedRect;
        }
        // See if rect is a right side tape image
        else if((rotatedRect.angle >= -60)&&(rotatedRect.angle <= -30))
        {
            if(rectArea > rightArea)
                rightTape = rotatedRect;
        }            
    }
    return std::make_tuple(GetRelativePoint(leftTape.center, mat), GetRelativePoint(rightTape.center,mat));
}

cv::Rect2d Rpi2959::Pipeline::FindHatch(const cv::Mat& mat)
{

}

cv::Point2f Rpi2959::Pipeline::FindLeftPoint(const std::array<cv::Point2f, 4>& points)
{
    cv::Point2f result{ points[0] };
    for(auto i = 1; i < points.size(); ++i)
    {
        auto&   point{points[i]};
        if(result.x > point.x)
            result = point;
    }
    return result;
 }
 
 cv::Point2f Rpi2959::Pipeline::FindTopPoint(const std::array<cv::Point2f, 4>& points)
 {
cv::Point2f result{ points[0] };
    for(auto i = 1; i < points.size(); ++i)
    {
        auto&   point{points[i]};
        if(result.y > point.y)
            result = point;
    }
    return result;
 }
 cv::Point2f Rpi2959::Pipeline::FindRightPoint(const std::array<cv::Point2f, 4>& points)
 {
cv::Point2f result{ points[0] };
    for(auto i = 1; i < points.size(); ++i)
    {
        auto&   point{points[i]};
        if(result.x < point.x)
            result = point;
    }
    return result;
 }
 cv::Point2f Rpi2959::Pipeline::FindBottomPoint(const std::array<cv::Point2f, 4>& points)
 {
cv::Point2f result{ points[0] };
    for(auto i = 1; i < points.size(); ++i)
    {
        auto&   point{points[i]};
        if(result.y < point.y)
            result = point;
    }
    return result;
 }

 Rpi2959::Pipeline::OrientedPoints Rpi2959::Pipeline::GetOrientedPoints(const std::array<cv::Point2f, 4>& points)
 {
   return OrientedPoints{ FindLeftPoint(points), FindTopPoint(points), FindRightPoint(points), FindBottomPoint(points) };
 }
 float Rpi2959::Pipeline::GetDistance(const cv::Point& pt1, const cv::Point& pt2)
 {
     auto    xDistance{ (pt1.x - pt2.x) };
    auto    yDistance{ (pt1.y - pt2.y) };
    return std::sqrt(xDistance * xDistance + yDistance * yDistance);
 }
 float Rpi2959::Pipeline::GetArea(const OrientedPoints& points)
 {
return GetDistance(points.TopPoint, points.LeftPoint) *
        GetDistance(points.TopPoint, points.RightPoint);
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
    if((targets & Rpi2959Shared::ProcessingTargets::PortTape) == Rpi2959Shared::ProcessingTargets::PortTape)
        m_networkTable->PutNumberArray(m_portTapeResultsKey, AsArrayRef(FindPortTape(mat)));
    else
        m_networkTable->Delete(m_portTapeResultsKey);
    if((targets & Rpi2959Shared::ProcessingTargets::FloorTape) == Rpi2959Shared::ProcessingTargets::FloorTape)
        m_networkTable->PutNumberArray(m_floorTapeResultsKey, AsArrayRef(FindFloorTape(mat)));
    else
        m_networkTable->Delete(m_floorTapeResultsKey);
    if((targets & Rpi2959Shared::ProcessingTargets::Hatch) == Rpi2959Shared::ProcessingTargets::Hatch)
        m_networkTable->PutNumberArray(m_hatchResultsKey, AsArrayRef(FindHatch(mat)));
    else
        m_networkTable->Delete(m_hatchResultsKey);


}
