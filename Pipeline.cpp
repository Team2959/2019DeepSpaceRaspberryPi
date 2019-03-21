//*****************************************************************************
//*                                                                           *
//* Pipeline.cpp - Pipeline class definition.                                 *
//*                                                                           *
//*****************************************************************************

#include <networktables/NetworkTable.h>
#include <opencv2/opencv.hpp>
#include <vision/VisionPipeline.h>
#include <wpi/raw_ostream.h>
#include "Pipeline.hpp"
#include "Analyzer.hpp"

Rpi2959::Pipeline::Pipeline(std::shared_ptr<nt::NetworkTable> networkTable, wpi::StringRef targetsKey,
  wpi::StringRef frameNumberKey, wpi::StringRef cargoResultsKey, wpi::StringRef floorTapeResultsKey,
  wpi::StringRef hatchResultsKey, wpi::StringRef portTapeResultsKey) :
  m_networkTable{ std::move(networkTable) }, m_targetsKey{ std::move(targetsKey) },
  m_frameNumberKey{ std::move( frameNumberKey) }, m_cargoResultsKey{ std::move( cargoResultsKey ) },
  m_floorTapeResultsKey{ std::move( floorTapeResultsKey ) },
  m_hatchResultsKey{ std::move( hatchResultsKey ) },
  m_portTapeResultsKey{ std::move( portTapeResultsKey ) } { }

void Rpi2959::Pipeline::Process(cv::Mat& mat)
{
    //  Update our frame number.  The RoboRio can watch for these changes and know that we are still alive
    m_networkTable->PutNumber(m_frameNumberKey, ++m_frameNumber);

    // Output a console message
    wpi::errs() << "Processing frame " << m_frameNumber << "\n";

    // Get the targets to examine
    auto    targets{ static_cast<Rpi2959Shared::ProcessingTargets>(m_networkTable->GetNumber(m_targetsKey, 0.0)) };

    // Make our analyzer
    Analyzer    analyzer{ mat };

    // For each target type, see if the RoboRio wants it.  If so, perform the calculations.  Otherwise,
    // Clear the result.
    if(IncludesTarget(targets, Rpi2959Shared::ProcessingTargets::Cargo))
    {
        auto    cargoRect{analyzer.FindCargo()};
        if(std::get<1>(cargoRect))
            m_networkTable->PutNumberArray(m_cargoResultsKey, AsArrayRef(std::get<0>(cargoRect)));
        else
            m_networkTable->Delete(m_cargoResultsKey);
    }
    else
        m_networkTable->Delete(m_cargoResultsKey);
    if(IncludesTarget(targets, Rpi2959Shared::ProcessingTargets::PortTape))
    {
        auto    tapePoints{analyzer.FindPortTape()};
        if(std::get<2>(tapePoints))
            m_networkTable->PutNumberArray(m_portTapeResultsKey, AsArrayRef(std::make_tuple(std::get<0>(tapePoints), std::get<1>(tapePoints))));
        else
            m_networkTable->Delete(m_portTapeResultsKey);
    }
    else
        m_networkTable->Delete(m_portTapeResultsKey);
    if(IncludesTarget(targets, Rpi2959Shared::ProcessingTargets::FloorTape))
    {
        auto    tapePoints{analyzer.FindFloorTape()};
        if(std::get<2>(tapePoints))
            m_networkTable->PutNumberArray(m_floorTapeResultsKey, AsArrayRef(std::make_tuple(std::get<0>(tapePoints), std::get<1>(tapePoints))));
        else
            m_networkTable->Delete(m_floorTapeResultsKey);
    }
    else
        m_networkTable->Delete(m_floorTapeResultsKey);
    if(IncludesTarget(targets, Rpi2959Shared::ProcessingTargets::Hatch))
    {
        auto    hatchRect{analyzer.FindCargo()};
        if(std::get<1>(hatchRect))
            m_networkTable->PutNumberArray(m_hatchResultsKey, AsArrayRef(std::get<0>(hatchRect)));
        else
            m_networkTable->Delete(m_hatchResultsKey);
    }
    else
        m_networkTable->Delete(m_hatchResultsKey);
}
