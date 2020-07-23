/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/client/device_info.h>

// HTTP client to get calibration info from sensors
#include <quanergy/client/http_client.h>

// to output some status
#include <iostream>

// for parsing device info
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace quanergy::client;

DeviceInfo::DeviceInfo(const std::string& host)
{
  HTTPClient http_client(host);
  std::stringstream device_info_stream;

  // get deviceInfo from sensor for calibration
  std::cout << "Attempting to get device info from " << host << std::endl;
  http_client.read(device_info_path_, device_info_stream);
  boost::property_tree::ptree device_info_tree;
  boost::property_tree::read_xml(device_info_stream, device_info_tree);

  // get model
  model_ = device_info_tree.get<std::string>("DeviceInfo.model");

  auto calibration_data = device_info_tree.get_child_optional("DeviceInfo.calibration");
  if (calibration_data)
  {
    // get encoder cal values
    amplitude_ = calibration_data->get_optional<double>("encoder.amplitude");
    phase_ = calibration_data->get_optional<double>("encoder.phase");

    // get vertical angles, if available
    auto laser_data = calibration_data->get_child_optional("lasers");
    if (laser_data)
    {
      vertical_angles_.resize(laser_data->get<unsigned int>("<xmlattr>.number"), 0.);

      for (auto& laser : *laser_data)
      {
        if (laser.first == "laser")
        {
          auto id = laser.second.get<unsigned int>("<xmlattr>.id");
          if (id < vertical_angles_.size())
          {
            vertical_angles_[id] = laser.second.get<double>("v");
          } // if id valid

        } // if laser

      } // for laser

    } // if laser data

  } // if cal data

} // constructor



