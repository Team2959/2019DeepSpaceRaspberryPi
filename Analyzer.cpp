//*****************************************************************************
//*                                                                           *
//* Analyzer.cpp - Analyzer class definition.                                 *
//*                                                                           *
//*****************************************************************************

#include <opencv2/opencv.hpp>
#include "Analyzer.hpp"

std::tuple<cv::Rect2d, bool> Rpi2959::Analyzer::FindCargo() const
{
    // Will hold our hsv image
    cv::Mat hsvImage;

    // convert from Red-Green-Blue to Hue-Saturation-Value
    cv::cvtColor(_mat, hsvImage, cv::ColorConversionCodes::COLOR_BGR2HSV);

    // Find only pixels that are in range of the HSV coordinates
    cv::Mat     threshold;
    cv::inRange(hsvImage, cv::Scalar(6, 100, 140), cv::Scalar(20, 255, 255), threshold);
    hsvImage.release();     // Done with hsvImage

    // get the contours for each threshold area
    Contours_t contours;
    cv::findContours(threshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    threshold.release();    // Done with threshold

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
    // Will hold our grayscale image
    cv::Mat gray;

    // Convert to grayscale
    cv::cvtColor(_mat, gray, cv::ColorConversionCodes::COLOR_BGR2GRAY);

    //  Find only the brightest pixels
    cv::Mat     threshold;
    cv::inRange(gray, cv::Scalar(128, 0, 0), cv::Scalar(255, 0, 0), threshold);
    gray.release();         // Done with gray

    // Erode the image...tends to eliminate small elements
    cv::erode(threshold, threshold, cv::Mat{});

    // extract contours
    Contours_t  contours;
    cv::findContours(threshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    threshold.release();    // Done with threshold

    // Get our tape centers
    auto    centers{ GetTapeCenters(contours) };

    // If an empty result, we have no solution
    if(!std::get<2>(centers))
        return std::make_tuple(cv::Point2d{}, cv::Point2d{}, false);

    // Convert from absolute coordinates to frame relative
    return std::tuple_cat(GetRelativePointPair(std::make_tuple(std::get<0>(centers), std::get<1>(centers))), std::make_tuple(true));
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

std::tuple<cv::Point2d, cv::Point2d, bool> Rpi2959::Analyzer::GetTapeCenters(const Contours_t& contours) const
{
    // If we don't have enough contours, no need to do anything
    if(contours.size() < 2)
        return std::make_tuple(cv::Point2f{}, cv::Point2f{}, false);

    // Convert contours to rotated bounding rects
    std::vector<cv::RotatedRect>    rects;

    // Convert all contours to rotated rects
    std::transform(begin(contours),end(contours), back_inserter(rects), [](auto& contour) { return cv::minAreaRect(contour); });

    // Sort the rects by size in descening order
    std::sort(begin(rects), end(rects), [](auto& left, auto& right) { return left.size.area() > right.size.area(); } );

    // Get an area limit...we don't care about any rects that are smaller than this
    auto    areaLimit{ rects[0].size.area() / 4.0 };

    // Find the first rect that is smaller than the area limit
    auto    iFirstSmall{ std::find_if(begin(rects), end(rects), [areaLimit](auto& rect) { return rect.size.area() < areaLimit; }) };

    // Erase those rects that are too small.
    rects.erase(iFirstSmall, end(rects));

    // Sort the rects by size in order of distance from center
    std::sort(begin(rects), end(rects), [this](auto& left, auto& right)
    {
        auto    leftDistance{ std::hypot(left.center.x - _mat.cols / 2, left.center.y - _mat.rows / 2) };
        auto    rightDistance{ std::hypot(right.center.x - _mat.cols / 2, right.center.y - _mat.rows / 2) };
        return leftDistance < rightDistance;
    });

    // Go through the rects, get our tape side, and find the next good match
    for(auto& baseRect : rects)
    {
        auto    side{ GetTapeSide(baseRect) };  // Get our tape side
        if(side == TapeSide::Unknown)           // If we can't tell which side, move on
            continue;

        // Find the first one that is outside our baseRect
        if(side == TapeSide::Left)
            for(auto& rect : rects)
            {
                if(rect.center.x > baseRect.center.x)
                    return std::make_tuple(baseRect.center, rect.center, true); 
            }
        else
            for(auto& rect : rects)
            {
                if(rect.center.x < baseRect.center.x)
                    return std::make_tuple(rect.center, baseRect.center, true); 
            }
    }

    // If we get here, there was nothing outside of our center rect.  No solution
    return std::make_tuple(cv::Point2f{}, cv::Point2f{}, false);
}

Rpi2959::Analyzer::TapeSide Rpi2959::Analyzer::GetTapeSide(const cv::RotatedRect& rect) const
{
    // If the rect is not actually rotated, we can't tell
    if(rect.angle == -90.0)
        return TapeSide::Unknown;

    // Will hold the points
    std::array<cv::Point2f, 4>  vertices;

    // Get the vertices
    rect.points(vertices.data());

    // Find the leftmost point
    auto    iLeft{std::min_element(begin(vertices), end(vertices), [](auto& left, auto& right){ return left.x < right.x; })};

    // We are left side tape if and only if the Y value for the leftmost verex is greater than the center Y value
    return (iLeft->y > rect.center.y) ? TapeSide::Left : TapeSide::Right;
}
