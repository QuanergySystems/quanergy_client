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

// for M_PI
#include <cmath>

using namespace quanergy::client;

std::size_t M_SERIES_NUM_ROT_ANGLES = 10400; // legacy number of encoder ticks

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

  // get calibration data, if available
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

  // get encoderTicks, if available
  auto encoder_ticks = device_info_tree.get_child_optional("DeviceInfo.encoderTicks");
  std::size_t num_angles = M_SERIES_NUM_ROT_ANGLES;
  if (encoder_ticks)
  {
    num_angles = encoder_ticks->get_value<std::size_t>();
  }

  // create horizontal angle lookup
  horizontal_angles_.resize(num_angles + 1);
  for (std::size_t i = 0; i < horizontal_angles_.size(); ++i)
  {
    // Shift by half the rot angles to keep the number positive when wrapping.
    std::size_t j = (i + (num_angles / 2) % num_angles);

    // normalized
    double n = static_cast<double>(j) / static_cast<double>(num_angles);

    double rad = n * M_PI * 2.0 - M_PI;

    horizontal_angles_[i] = rad;
  }

} // constructor



