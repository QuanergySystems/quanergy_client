/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/pipelines/sensor_pipeline.h>

#include <quanergy/client/device_info.h>
#include <quanergy/client/exceptions.h>

namespace quanergy
{
  namespace pipeline
  {
    SensorPipeline::SensorPipeline(const SensorPipelineSettings& settings)
    {
      // get deviceInfo from sensor and apply calibration
      quanergy::client::DeviceInfo device_info(settings.host);

      // get sensor type
      auto model = device_info.model();
      std::cout << "got model from device info: " << model << std::endl;

      // 'model.rfind(sub, 0) == 0' checks only the first position (the beginning) of model for sub
      // and is true if sub was found there
      bool m_series = model.rfind("MQ", 0) == 0
                              || model.rfind("M8", 0) == 0
                              || model.rfind("M1", 0) == 0;

      if (m_series)
      {
        // encoder params
        if (settings.calibrate)
        {
          std::cout << "Encoder calibration will be performed" << std::endl;
          encoder_corrector.setFrameRate(settings.frame_rate);
        }
        else if (settings.override_encoder_params)
        {
          std::cout << "Encoder calibration parameters provided will be applied" << std::endl;
          encoder_corrector.setParams(settings.amplitude, settings.phase);
        }
        else if (device_info.amplitude() && device_info.phase())
        {
          std::cout << "Encoder calibration parameters from the sensor will be applied" << std::endl;
          encoder_corrector.setParams(*device_info.amplitude(), *device_info.phase());
        }
        else
        {
          std::cout << "No encoder calibration will be applied" << std::endl;
          encoder_corrector.setParams(0.f, 0.f); // turns off calibration procedure
        }

        auto vertical_angles = device_info.verticalAngles();
        if (!vertical_angles.empty())
        {
          if (model.rfind("M1", 0) == 0)
          {
            throw quanergy::client::InvalidVerticalAngles("M1 sensor found with vertical angles but none are expected");
          }

          // send the vertical angles to the parsers
          parser.get<PARSER_00_INDEX>().setVerticalAngles(vertical_angles);
          parser.get<PARSER_04_INDEX>().setVerticalAngles(vertical_angles);
        }
        else if (model.rfind("M8", 0) == 0)
        {
          std::cout << "No vertical angle calibration information available on sensor, proceeding with M8 defaults" << std::endl;

          // tell parsers to use M8 defaults
          parser.get<PARSER_00_INDEX>().setVerticalAngles(quanergy::client::SensorType::M8);
          parser.get<PARSER_04_INDEX>().setVerticalAngles(quanergy::client::SensorType::M8);
        }
        else if (model.rfind("MQ", 0) == 0)
        {
          // all MQ sensors should have vertical angles
          throw quanergy::client::InvalidVerticalAngles("MQ sensor found with no vertical angles on the sensor");
        }
      }

      // Setup modules
      // Parsers
      auto &parser00 = parser.get<PARSER_00_INDEX>();
      auto &parser01 = parser.get<PARSER_01_INDEX>();
      auto &parser04 = parser.get<PARSER_04_INDEX>();
      auto &parser06 = parser.get<PARSER_06_INDEX>();

      // Parser 00
      parser00.setFrameId(settings.frame);
      parser00.setReturnSelection(settings.return_selection);
      parser00.setCloudSizeLimits(
        settings.min_cloud_size,
        settings.max_cloud_size
      );

      // Parser 01
      parser01.setFrameId(settings.frame);

      // Parser 04
      parser04.setFrameId(settings.frame);
      if (settings.return_selection_set)
      {
        parser04.setReturnSelection(settings.return_selection);
      }
      parser04.setCloudSizeLimits(
        settings.min_cloud_size,
        settings.max_cloud_size
      );

      // Parser 06
      parser06.setFrameId(settings.frame);
      if (settings.return_selection_set)
      {
        parser06.setReturnSelection(settings.return_selection);
      }
      parser06.setCloudSizeLimits(
        settings.min_cloud_size,
        settings.max_cloud_size
      );

      // Filters
      // Distance Filter
      distance_filter.setMaximumDistanceThreshold(settings.max_distance);
      distance_filter.setMinimumDistanceThreshold(settings.min_distance);

      // ring intensity filter
      for (int i = 0; i < quanergy::client::M_SERIES_NUM_LASERS; ++i)
      {
        ring_intensity_filter.setRingFilterMinimumRangeThreshold(
          i, settings.ring_range[i]
        );
        ring_intensity_filter.setRingFilterMinimumIntensityThreshold(
          i, settings.ring_intensity[i]
        );
      }

      if (m_series)
      {
        // Connect modules for m_series
        // Parser to Encoder Corrector
        connections.push_back(
          parser.connect(
            [this](const ParserModule::ResultType& pc)
            { encoder_corrector.slot(pc); }
          )
        );

        // Encoder Corrector to Distance Filter
        connections.push_back(
          encoder_corrector.connect(
            [this](const quanergy::calibration::EncoderAngleCalibration::ResultType& pc)
            { distance_filter.slot(pc); }
          )
        );

        // Distance Filter to Ring Intensity Filter
        connections.push_back(
          distance_filter.connect(
            [this](const quanergy::client::DistanceFilter::ResultType& pc)
            { ring_intensity_filter.slot(pc); }
          )
        );

        // Ring Intensity Filter to Polar->Cartesian Converter
        connections.push_back(
          ring_intensity_filter.connect(
            [this](const quanergy::client::RingIntensityFilter::ResultType& pc)
            { cartesian_converter.slot(pc); }
          )
        );
      }
      else
      {
        // Parser to Distance Filter
        connections.push_back(
          parser.connect(
            [this](const ParserModule::ResultType& pc)
            { distance_filter.slot(pc); }
          )
        );

        // Distance Filter to Polar->Cartesian Converter
        connections.push_back(
          distance_filter.connect(
            [this](const quanergy::client::DistanceFilter::ResultType& pc)
            { cartesian_converter.slot(pc); }
          )
        );
      }

      // connect to an async module so downstream work happens on a separate thread
      connections.push_back(cartesian_converter.connect(
          [this](const quanergy::client::PolarToCartConverter::ResultType& pc){ async.slot(pc); }
      ));
    }

    SensorPipeline::~SensorPipeline()
    {
      // Clean up
      for (auto &connection : connections)
      {
        connection.disconnect();
      }
      connections.clear();
    }
  }
}
