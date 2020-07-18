/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_SENSOR_PIPELINE_SETTINGS_H
#define QUANERGY_CLIENT_SENSOR_PIPELINE_SETTINGS_H

#include <quanergy/parsers/data_packet_parser_m_series.h>

// for setting file
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace pipeline
  {
    /// \brief helper for loading the settings file
    class SettingsFileLoader;

    /// \brief struct to hold settings values for the sensor pipeline
    struct DLLEXPORT SensorPipelineSettings
    {
      // host sensor or IP
      std::string host; // there is no default value that makes sense

      // frame name inserted in the point cloud
      std::string frame = "quanergy";

      // return selection
      // options are 0, 1, 2 or quanergy::client::ALL_RETURNS which produces
      //   an unorganized cloud with all 3 returns (if 3 returns are available)
      int return_selection = 0;
      // return selection set; true if explicitly set
      // single return packet parsers will produce an error if explicitly set to non-matching return
      bool return_selection_set = false;

      // Encoder correction terms; for newer sensors, these are retrieved from the sensor but can be overridden
      // this is generally only applicable to high precision applications such as mapping
      bool calibrate = false; // whether to perform calibration and apply; takes precidence over other options
      double frame_rate = 10.0; // frame rate of the sensor is used for calibration calculation only
      bool override_encoder_params = false; // whether to override sensor values
      float amplitude = 0.f; // radians
      float phase = 0.f;     // radians

      // Distance filter; default values do not filter anything
      float min_distance = 0.0f;
      float max_distance = 500.f;

      // used to validate point cloud size from the M-series
      // defaults don't do anything because meaningful numbers depend on
      // frame rate and FOV being used
      std::int32_t min_cloud_size = 0;
      std::int32_t max_cloud_size = quanergy::client::MAX_CLOUD_SIZE;

      // Ring filter; generally this is not needed
      // Only can be configured in settings file
      // only relevant for M-series
      float ring_range[quanergy::client::M_SERIES_NUM_LASERS] = {0.f};
      std::uint16_t ring_intensity[quanergy::client::M_SERIES_NUM_LASERS] = {0};

      /** \brief load settings from SettingsFileLoader
       *  \param settings SettingsFileLoader to load from
       */
      void load(const SettingsFileLoader& settings);

      /// \brief convenience functions to convert between return selection int and string
      static int returnFromString(const std::string& r);
      static std::string stringFromReturn(int r);

    };

    class DLLEXPORT SettingsFileLoader : public boost::property_tree::ptree
    {
    public:
      using boost::property_tree::ptree::ptree;

      void loadXML(std::string const & file_name)
      {
        boost::property_tree::xml_parser::read_xml(file_name, static_cast<boost::property_tree::ptree&>(*this));
      }
    };
  }
}

#endif
