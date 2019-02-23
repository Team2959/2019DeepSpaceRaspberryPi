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
            Analyzer(cv::Mat& mat) : _mat{ mat } { }
            std::tuple<cv::Rect2d, bool> FindCargo() const;
            std::tuple<cv::Point2d, cv::Point2d, bool> FindFloorTape() const;
            std::tuple<cv::Point2d, cv::Point2d, bool> FindPortTape() const;
            std::tuple<cv::Rect2d, bool> FindHatch() const;

        private:
            template<typename PointT> using PointPair_t = std::tuple<PointT, PointT>;
            using Contour_t = std::vector<cv::Point>;
            using Contours_t = std::vector<Contour_t>;

            struct OrientedPoints
            {
                cv::Point2f LeftPoint;
                cv::Point2f TopPoint;
                cv::Point2f RightPoint;
                cv::Point2f BottomPoint;
            };

            static cv::Point2f FindLeftPoint(const std::array<cv::Point2f, 4>& points) { return *std::min_element(begin(points), end(points), [](const cv::Point2d& a, const cv::Point2d& b) { return a.x < b.x; }); }
            static cv::Point2f FindTopPoint(const std::array<cv::Point2f, 4>& points) { return *std::min_element(begin(points), end(points), [](const cv::Point2d& a, const cv::Point2d& b) { return a.y < b.y; }); }
            static cv::Point2f FindRightPoint(const std::array<cv::Point2f, 4>& points) { return *std::max_element(begin(points), end(points), [](const cv::Point2d& a, const cv::Point2d& b) { return a.x < b.x; }); }
            static cv::Point2f FindBottomPoint(const std::array<cv::Point2f, 4>& points) { return *std::max_element(begin(points), end(points), [](const cv::Point2d& a, const cv::Point2d& b) { return a.y < b.y; }); }
            static double GetArea(const OrientedPoints& points) { return GetDistance(points.TopPoint, points.LeftPoint) * GetDistance(points.TopPoint, points.RightPoint); }
            static double GetDistance(const cv::Point& pt1, const cv::Point& pt2) { return std::hypot(pt1.x - pt2.x, pt1.y - pt2.y); }
            static OrientedPoints GetOrientedPoints(const std::array<cv::Point2f, 4>& points) { return OrientedPoints{ FindLeftPoint(points), FindTopPoint(points), FindRightPoint(points), FindBottomPoint(points) }; }
            static double GetRelativeValue(int value, int extent) { return static_cast<double>(value) / extent; }
            static cv::Rect GetLargestContourBoundingRect(const Contours_t& contours);
            cv::Rect2d GetRelativeRect(const cv::Rect& rect) const { return cv::Rect2d{ GetRelativePoint(rect.tl()), GetRelativeSize(rect.size()) }; }
            cv::Point2d GetRelativePoint(const cv::Point& point) const { return cv::Point2d{ GetRelativeValue(point.x, _mat.cols ), GetRelativeValue(point.y, _mat.rows) }; }
            PointPair_t<cv::Point2d> GetRelativePointPair(const PointPair_t<cv::Point>& points) const { return std::make_tuple(GetRelativePoint(std::get<0>(points)), GetRelativePoint(std::get<1>(points))); }
            cv::Size2d GetRelativeSize(const cv::Size& size) const { return cv::Size2d{ GetRelativeValue(size.width, _mat.cols), GetRelativeValue(size.height, _mat.rows) }; }

            cv::Mat&    _mat;
    };
}
