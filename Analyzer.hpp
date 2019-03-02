//*****************************************************************************
//*                                                                           *
//* Analyzer.hpp - Analyzer class declaration.                                *
//*                                                                           *
//*****************************************************************************

namespace Rpi2959
{
    class Analyzer
    {
        public:
            Analyzer(const cv::Mat& mat) : _mat{ mat } { }
            std::tuple<cv::Rect2d, bool> FindCargo() const;
            std::tuple<cv::Point2d, cv::Point2d, bool> FindFloorTape() const;
            std::tuple<cv::Point2d, cv::Point2d, bool> FindPortTape() const;
            std::tuple<cv::Rect2d, bool> FindHatch() const;

        private:
            using Contour_t = std::vector<cv::Point>;
            using Contours_t = std::vector<Contour_t>;

            enum class TapeSide { Left, Right, Unknown };

            static double GetRelativeValue(int value, int extent) { return static_cast<double>(value) / extent; }
            static cv::Rect GetLargestContourBoundingRect(const Contours_t& contours);
            cv::Rect2d GetRelativeRect(const cv::Rect& rect) const { return cv::Rect2d{ GetRelativePoint(rect.tl()), GetRelativeSize(rect.size()) }; }
            cv::Point2d GetRelativePoint(const cv::Point& point) const { return cv::Point2d{ GetRelativeValue(point.x, _mat.cols ), GetRelativeValue(point.y, _mat.rows) }; }
            std::tuple<cv::Point2d, cv::Point2d> GetRelativePointPair(const std::tuple<cv::Point, cv::Point>& points) const { return std::make_tuple(GetRelativePoint(std::get<0>(points)), GetRelativePoint(std::get<1>(points))); }
            cv::Size2d GetRelativeSize(const cv::Size& size) const { return cv::Size2d{ GetRelativeValue(size.width, _mat.cols), GetRelativeValue(size.height, _mat.rows) }; }
            std::tuple<cv::Point2d, cv::Point2d, bool> GetTapeCenters(const Contours_t& contours) const;
            TapeSide GetTapeSide(const cv::RotatedRect& rect) const;

            const cv::Mat&    _mat;
    };
}
