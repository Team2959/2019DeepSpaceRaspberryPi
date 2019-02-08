/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cameraserver/CameraServer.h>
#include <networktables/EntryListenerFlags.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/opencv.hpp>
#include <vision/VisionRunner.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>
#include "../2019RaspPIRoboRioShared/SharedNames.h"
#include "Pipeline.hpp"

namespace Rpi2959
{
  struct CameraConfig
  {
    std::string name;
    std::string path;
    wpi::json config;
    wpi::json streamConfig;
  };

  static std::string                configFile;
  static unsigned int               team;
  static bool                       server{ false };
  static std::vector<CameraConfig>  cameraConfigs;

  Pipeline CreatePipeline(std::string name, std::shared_ptr<nt::NetworkTable> networkTable);
  Pipeline CreatePipeline(bool isFront, std::shared_ptr<nt::NetworkTable> networkTable);
  std::string GetValueText(std::shared_ptr<nt::Value> value);
  wpi::raw_ostream& ParseError();
  bool ReadCameraConfig(const wpi::json& config);
  bool ReadConfig();
  cs::UsbCamera StartCamera(const CameraConfig& config);
}

int main(int argc, char* argv[]) 
{
  Rpi2959::configFile = (argc >= 2) ? argv[1] : "/boot/frc.json";

  // read configuration
  if (!Rpi2959::ReadConfig())
    return EXIT_FAILURE;

  // start NetworkTables
  auto  ntinst{ nt::NetworkTableInstance::GetDefault() };
  auto& outs{ wpi::outs() };
  if (Rpi2959::server)
  {
    outs << "Setting up NetworkTables server\n";
    ntinst.StartServer();
  } 
  else
  {
    outs << "Setting up NetworkTables client for team " << Rpi2959::team << '\n';
    ntinst.StartClientTeam(Rpi2959::team);
  }

  // This the Network Table which we use to pull configuration and send results to the
  std::shared_ptr<NetworkTable> networkTable{ ntinst.GetTable(Rpi2959Shared::Tables::TableName) };

  // This is testing code that just outputs the results that are put into Network Tables.
  // This does not need to be in the final release, once debugging is complete
  auto  listener{ networkTable->AddEntryListener([&outs](auto table, auto name, auto entry, auto value, auto flags)
  {
    /*std::string s{name};

     if((s.find("FrameNumber") != -1)||(s.find("CargoResults") != -1))
      return; */
    if(flags & nt::EntryListenerFlags::kDelete)
      (outs << "Key Deleted - " << name << '\n').flush();
    else if(flags & nt::EntryListenerFlags::kNew)
      (outs << "Key Added - " << name << " = " << Rpi2959::GetValueText(value) << '\n').flush();
    else if(flags & nt::EntryListenerFlags::kUpdate)
      (outs << "Key Updated - " << name << " = " << Rpi2959::GetValueText(value) << '\n').flush();
  }, nt::EntryListenerFlags::kLocal | nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate | nt::EntryListenerFlags::kDelete) };

  // This is also testing code...it starts the Raspberry PI to perform all processing front and back.
  // DEFINITELY REMOVE once debugging is complete.
  if(Rpi2959::server)
  {
    networkTable->PutNumber(Rpi2959Shared::Keys::FrontTargets, (double)15.0);
    networkTable->PutNumber(Rpi2959Shared::Keys::BackTargets, (double)15.0);
  }
  
  std::vector<std::thread>      threads;          // Will hold one thread per camera config
  threads.reserve(Rpi2959::cameraConfigs.size()); // Preallocate the space that we know that we will need

  // For each cameraConfig, create a thread with a pipeline to run image analysis
  std::transform(begin(Rpi2959::cameraConfigs), end(Rpi2959::cameraConfigs), std::back_inserter(threads),
    [&outs, &networkTable](auto& cameraConfig)
    {
      return std::thread([&outs, &cameraConfig, &networkTable]
      {
        auto  pipeline{ Rpi2959::CreatePipeline(cameraConfig.name, networkTable) };
        frc::VisionRunner<decltype(pipeline)>{ StartCamera(cameraConfig), &pipeline, [](decltype(pipeline)&){ } }.RunForever();
      });
    });

  // Wait for each thread to finish.  Since no thread will ever finish, we will never complete this
  // for loop.  However, the application exits if we don't block, and we don't want to waste
  // processor cycles with a "while(true);" loop
  for(auto& thread : threads)
    thread.join();

  // Quit listening to the network table changes (though we should never get here).
  networkTable->RemoveEntryListener(listener);

  // Exit the program with success
  return EXIT_SUCCESS;
}

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
   }
 */

namespace Rpi2959
{
  wpi::raw_ostream& ParseError() { return wpi::errs() << "config error in '" << configFile << "': "; }

  Pipeline CreatePipeline(std::string name, std::shared_ptr<nt::NetworkTable> networkTable)
  {
      // Make name text lower case
      std::transform(begin(name), end(name), begin(name), [](auto c) { return std::tolower(c); });
      auto  isFront{ name.find("back") == -1 };  // If the name does not contain "back" text, then we are a front camera
      return CreatePipeline(isFront, std::move(networkTable));
  }

  Pipeline CreatePipeline(bool isFront, std::shared_ptr<nt::NetworkTable> networkTable)
  {
      return isFront ? 
        Pipeline{networkTable, Rpi2959Shared::Keys::FrontTargets,
          Rpi2959Shared::Keys::FrontFrameNumber, Rpi2959Shared::Keys::FrontCargoResults,
          Rpi2959Shared::Keys::FrontFloorTapeResults, Rpi2959Shared::Keys::FrontHatchResults,
          Rpi2959Shared::Keys::FrontPortTapeResults} :
        Pipeline{networkTable, Rpi2959Shared::Keys::BackTargets,
          Rpi2959Shared::Keys::BackFrameNumber, Rpi2959Shared::Keys::BackCargoResults,
          Rpi2959Shared::Keys::BackFloorTapeResults, Rpi2959Shared::Keys::BackHatchResults,
          Rpi2959Shared::Keys::BackPortTapeResults};
  }

  std::string GetValueText(std::shared_ptr<nt::Value> value)
  {
    if(value->type() == NT_Type::NT_DOUBLE)
      return std::to_string(value->GetDouble());
    if(value->type() == NT_Type::NT_DOUBLE_ARRAY)
    {
      auto  values{value->GetDoubleArray()};
      std::stringstream ss;
      ss << "{";
      if(size(values) >0)
      {
        ss << values[0];
        for(auto i = 1; i < size(values); ++i)
          ss << ", " << values[i];
      }
      ss << "}";
      return ss.str();
    }
    return "Unknown";
  }

  bool ReadCameraConfig(const wpi::json& config)
  {
    CameraConfig c;

    // name
    try
    {
      c.name = config.at("name").get<std::string>();
    }
    catch (const wpi::json::exception& e)
    {
      ParseError() << "could not read camera name: " << e.what() << '\n';
      return false;
    }

    // path
    try 
    {
      c.path = config.at("path").get<std::string>();
    }
    catch (const wpi::json::exception& e)
    {
      ParseError() << "camera '" << c.name << "': could not read path: " << e.what() << '\n';
      return false;
    }

    // stream properties
    if (config.count("stream") != 0)
      c.streamConfig = config.at("stream");
    
    c.config = config;
    cameraConfigs.emplace_back(std::move(c));
    return true;
  }

  bool ReadConfig()
  {
    // open config file
    std::error_code     ec;
    wpi::raw_fd_istream is(configFile, ec);
    if (ec)
    {
      wpi::errs() << "could not open '" << configFile << "': " << ec.message() << '\n';
      return false;
    }

    // parse file
    wpi::json j;
    try
    {
      j = wpi::json::parse(is);
    }
    catch (const wpi::json::parse_error& e)
    {
      ParseError() << "byte " << e.byte << ": " << e.what() << '\n';
      return false;
    }

    // top level must be an object
    if (!j.is_object())
    {
      ParseError() << "must be JSON object\n";
      return false;
    }

    // team number
    try
    {
      team = j.at("team").get<unsigned int>();
    }
    catch (const wpi::json::exception& e)
    {
      ParseError() << "could not read team number: " << e.what() << '\n';
      return false;
    }

    // ntmode (optional)
    if (j.count("ntmode") != 0)
    {
      try
      {
        auto str = j.at("ntmode").get<std::string>();
        wpi::StringRef s(str);
        if (s.equals_lower("client")) 
        {
          server = false;
        }
        else if (s.equals_lower("server"))
        {
          server = true;
        } 
        else
        {
          ParseError() << "could not understand ntmode value '" << str << "'\n";
        }
      }
      catch (const wpi::json::exception& e)
      {
        ParseError() << "could not read ntmode: " << e.what() << '\n';
      }
    }

    // cameras
    try
    {
      for (auto&& camera : j.at("cameras"))
        if (!ReadCameraConfig(camera))
          return false;
    }
    catch (const wpi::json::exception& e)
    {
      ParseError() << "could not read cameras: " << e.what() << '\n';
      return false;
    }
    
    return true;
  }

  cs::UsbCamera StartCamera(const CameraConfig& config)
  {
    wpi::outs() << "Starting camera '" << config.name << "' on " << config.path << '\n';
    auto inst{ frc::CameraServer::GetInstance() };
    cs::UsbCamera camera{config.name, config.path};
    auto server{inst->StartAutomaticCapture(camera)};
    camera.SetConfigJson(config.config);
    camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
    if (config.streamConfig.is_object())
      server.SetConfigJson(config.streamConfig);
    return camera;
  }
}
