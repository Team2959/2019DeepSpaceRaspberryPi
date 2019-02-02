/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "cameraserver/CameraServer.h"
#include "Pipeline.hpp"
#include "../2019RaspPIRoboRioShared/SharedNames.h"

namespace
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

  wpi::raw_ostream& ParseError();
  bool ReadCameraConfig(const wpi::json& config);
  bool ReadConfig();
  cs::UsbCamera StartCamera(const CameraConfig& config);
}

int main(int argc, char* argv[]) 
{
  configFile = (argc >= 2) ? argv[1] : "/boot/frc.json";

  // read configuration
  if (!ReadConfig())
    return EXIT_FAILURE;

  // start NetworkTables
  auto ntinst{ nt::NetworkTableInstance::GetDefault() };
  if (server)
  {
    wpi::outs() << "Setting up NetworkTables server\n";
    ntinst.StartServer();
  } 
  else
  {
    wpi::outs() << "Setting up NetworkTables client for team " << team << '\n';
    ntinst.StartClientTeam(team);
  }

  // start processing threads with a camera and pipeline for each
  std::vector<std::thread>      threads;
  std::shared_ptr<NetworkTable> networkTable{ ntinst.GetTable(TABLE_NAME) };
  for (auto& cameraConfig : cameraConfigs)
    threads.emplace_back(std::thread([cameraConfig, networkTable]
    {
      auto      camera{StartCamera(cameraConfig)};
      Pipeline  pipeline{networkTable};
      frc::VisionRunner<Pipeline>{std::move(camera), &pipeline, [](Pipeline &p){ p.SendResults(); }}.RunForever();
    }));
  for(auto& thread : threads)
    thread.join();
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

namespace
{
  wpi::raw_ostream& ParseError() { return wpi::errs() << "config error in '" << configFile << "': "; }

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
