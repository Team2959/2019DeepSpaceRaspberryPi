//*****************************************************************************
//*                                                                           *
//* Pipeline.hpp - Pipeline class declaration.                                *
//*                                                                           *
//*****************************************************************************

#include "../2019RaspPIRoboRioShared/Shared.hpp"

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
            static wpi::ArrayRef<double> AsArrayRef(const std::tuple<cv::Point2d, cv::Point2d>& points){ return std::vector<double>{ std::get<0>(points).x, std::get<0>(points).y, std::get<1>(points).x, std::get<1>(points).y }; }
            static bool IncludesTarget(Rpi2959Shared::ProcessingTargets targets, Rpi2959Shared::ProcessingTargets mask) { return (targets & mask) == mask; }

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
