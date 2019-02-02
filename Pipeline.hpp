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
            static wpi::ArrayRef<double> AsArrayRef(const cv::Rect2d& rect){ return std::vector<double>{ rect.x, rect.y, rect.width, rect.height }; }
            static cv::Rect2d FindCargo(const cv::Mat& mat);
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
