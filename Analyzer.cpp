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
    // Angle boundaries for differentiating between left and right
    constexpr double LeftSideAngle{ -67.5 };
    constexpr double RightSideAngle{ -22.5 };
    constexpr double AngleRange{ 15.0 };
    constexpr double LeftSideAngleMin{ LeftSideAngle - AngleRange };
    constexpr double LeftSideAngleMax{ LeftSideAngle + AngleRange };
    constexpr double RightSideAngleMin{ RightSideAngle - AngleRange };
    constexpr double RightSideAngleMax{ RightSideAngle + AngleRange };

    cv::Mat gray;

    // Convert to grayscale
    cv::cvtColor(_mat, gray, cv::ColorConversionCodes::COLOR_BGR2GRAY);

    //  Find only the brightest pixels
    cv::Mat     threshold;
    cv::inRange(gray, cv::Scalar(240, 0, 0), cv::Scalar(255, 0, 0), threshold);

    // extract contours
    Contours_t  contours;
    cv::findContours(threshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    if(contours.size() == 0)
        return std::make_tuple(cv::Point2d{}, cv::Point2d{}, false);

    cv::RotatedRect leftTape;
    cv::RotatedRect rightTape;
    double          leftArea{ 0.0f };
    double          rightArea{ 0.0f };

    // Go through each contour
    for(auto& contour : contours)
    {
        // fit bounding rectangle around contour
        cv::RotatedRect rotatedRect{ cv::minAreaRect(contour) };

        // read points and angle
        std::array<cv::Point2f, 4> rect_points; 
        rotatedRect.points( rect_points.data() );

        auto    orientedPoints{ GetOrientedPoints(rect_points) };
        auto    rectArea{GetArea(orientedPoints)};
        
        // See if rect is a left side tape image
        if((LeftSideAngleMin <= rotatedRect.angle)&&(rotatedRect.angle <= LeftSideAngleMax))
        {
            if(rectArea > leftArea)
                leftTape = rotatedRect;
        }
        // See if rect is a right side tape image
        else if((RightSideAngleMin <= rotatedRect.angle)&&(rotatedRect.angle <= RightSideAngleMax))
        {
            if(rectArea > rightArea)
                rightTape = rotatedRect;
        }            
    }

    // If no left tape or no right tape
    if((leftTape.center == cv::Point2f{})||(rightTape.center == cv::Point2f{}))
        return std::make_tuple(cv::Point2d{}, cv::Point2d{}, false);

    // Get our frame relative tape
    auto    relativePoints{GetRelativePointPair(std::make_tuple(leftTape.center, rightTape.center))};

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
