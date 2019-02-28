//*****************************************************************************
//*                                                                           *
//* Analyzer.cpp - Analyzer class definition.                                 *
//*                                                                           *
//*****************************************************************************

#include <opencv2/opencv.hpp>
#include "Analyzer.hpp"

std::tuple<cv::Rect2d, bool> Rpi2959::Analyzer::FindCargo() const
{
    cv::Mat hsvImage;

    // convert from Red-Green-Blue to Hue-Saturation-Value
    cv::cvtColor(_mat, hsvImage, cv::ColorConversionCodes::COLOR_BGR2HSV);

    // Find only pixels that are in range of the HSV coordinates
    cv::Mat     threshold;
    cv::inRange(hsvImage, cv::Scalar(6, 100, 140), cv::Scalar(20, 255, 255), threshold);

    // get the contours for each threshold area
    Contours_t contours;
    cv::findContours(threshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    if(contours.size() == 0)
        return std::make_tuple(cv::Rect2d{}, false);

    // Get the largest bounding rect from the contours
    return std::make_tuple(GetRelativeRect(GetLargestContourBoundingRect(contours)), true);
}

std::tuple<cv::Point2d, cv::Point2d, bool> Rpi2959::Analyzer::FindFloorTape() const
{
    // Not implemented
    return std::make_tuple(cv::Point2d{}, cv::Point2d{}, false);
}

std::tuple<cv::Point2d, cv::Point2d, bool> Rpi2959::Analyzer::FindPortTape() const
{
    cv::Mat gray;

    // Convert to grayscale
    cv::cvtColor(_mat, gray, cv::ColorConversionCodes::COLOR_BGR2GRAY);

    //  Find only the brightest pixels
    cv::Mat     threshold;
    cv::inRange(gray, cv::Scalar(210, 0, 0), cv::Scalar(255, 0, 0), threshold);

    // extract contours
    Contours_t  contours;
    cv::findContours(threshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // If fewer than two contours, we have no solution
    if(contours.size() < 2)
        return std::make_tuple(cv::Point2d{}, cv::Point2d{}, false);

    // Get our frame relative tape centers
    auto    relativePoints{GetRelativePointPair(GetTapeCenters(contours))};

    // Return the results
    return std::make_tuple(std::get<0>(relativePoints), std::get<1>(relativePoints), true);
}

std::tuple<cv::Rect2d, bool> Rpi2959::Analyzer::FindHatch() const
{
    // Not implemented
    return std::make_tuple(cv::Rect2d{}, false);
}

cv::Rect Rpi2959::Analyzer::GetLargestContourBoundingRect(const Contours_t& contours)
{
    cv::Rect    largestRect;
    auto        largestArea{ 0.0 };

    for(auto& contour : contours)
    {
        auto    rect{ cv::boundingRect(contour) };  // change the contour pixels to be a rectangle
        auto    area{ rect.area() };                // Get the area
        if(largestArea > area)                      // Ignore if smaller
            continue;
        largestRect = rect;                         // Keep the new one
        largestArea = area;
    }
    return largestRect;                             // Return the largest
}

Rpi2959::Analyzer::PointPair_t<cv::Point2d> Rpi2959::Analyzer::GetTapeCenters(const Contours_t& contours)
{
    // Convert contours to rotated bounding rects
    std::vector<cv::RotatedRect>    rects;
    std::transform(begin(contours), end(contours), back_inserter(rects), [](const Contour_t& contour) { return cv::minAreaRect(contour); });

    // Sort them in descending order based on size.
    std::sort(begin(rects), end(rects), [](const cv::RotatedRect& left, const cv::RotatedRect& right) { return left.size.area() > right.size.area(); });

    // Return the leftmost one first, then the rightmost one
    return (rects[0].center.x < rects[1].center.x) ? std::make_tuple(rects[0].center, rects[1].center) : std::make_tuple(rects[1].center, rects[0].center);
}
