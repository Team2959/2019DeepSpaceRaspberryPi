//*****************************************************************************
//*                                                                           *
//* Pipeline.hpp - Pipeline class declaration.                                *
//*                                                                           *
//*****************************************************************************

namespace Rpi2959
{
    class Pipeline : public frc::VisionPipeline
    {
        public:
            Pipeline(std::shared_ptr<nt::NetworkTable> networkTable, wpi::StringRef targetsKey,
              wpi::StringRef frameNumberKey, wpi::StringRef cargoResultsKey,
              wpi::StringRef floorTapeResultsKey, wpi::StringRef hatchResultsKey,
              wpi::StringRef portTapeResultsKey);
            virtual void Process(cv::Mat& mat) override;

        private:
            struct OrientedPoints
            {
                cv::Point2f LeftPoint;
                cv::Point2f TopPoint;
                cv::Point2f RightPoint;
                cv::Point2f BottomPoint;
            };

            static wpi::ArrayRef<double> AsArrayRef(const cv::Rect2d& rect){ return std::vector<double>{ rect.x, rect.y, rect.width, rect.height }; }
            static wpi::ArrayRef<double> AsArrayRef(const std::tuple<cv::Point2d, cv::Point2d>& points){ return std::vector<double>{ std::get<0>(points).x, std::get<0>(points).y, std::get<1>(points).x, std::get<1>(points).y }; }
            static cv::Rect2d FindCargo(const cv::Mat& mat);
            static std::tuple<cv::Point2d, cv::Point2d> FindFloorTape(const cv::Mat& mat);
            static std::tuple<cv::Point2d, cv::Point2d> FindPortTape(const cv::Mat& mat);
            static cv::Rect2d FindHatch(const cv::Mat& mat);
            static cv::Point2f FindLeftPoint(const std::array<cv::Point2f, 4>& points);
            static cv::Point2f FindTopPoint(const std::array<cv::Point2f, 4>& points);
            static cv::Point2f FindRightPoint(const std::array<cv::Point2f, 4>& points);
            static cv::Point2f FindBottomPoint(const std::array<cv::Point2f, 4>& points);
            static OrientedPoints GetOrientedPoints(const std::array<cv::Point2f, 4>& points);
            static float GetDistance(const cv::Point& pt1, const cv::Point& pt2);
            static float GetArea(const OrientedPoints& points);
            static cv::Point2d GetRelativePoint(const cv::Point& point, const cv::Mat& mat);
            static cv::Rect2d GetRelativeRect(const cv::Rect& rect, const cv::Mat& mat);

            std::shared_ptr<nt::NetworkTable>   m_networkTable;
            wpi::StringRef                      m_targetsKey;
            wpi::StringRef                      m_frameNumberKey;
            wpi::StringRef                      m_cargoResultsKey;
            wpi::StringRef                      m_floorTapeResultsKey;
            wpi::StringRef                      m_hatchResultsKey;
            wpi::StringRef                      m_portTapeResultsKey;
            int                                 m_frameNumber{ 0 };
    };
}
