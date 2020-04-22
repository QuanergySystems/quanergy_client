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

      if (model.find("MQ") != std::string::npos
          || model.find("M8") != std::string::npos)
      {
        // encoder params
        if (settings.calibrate)
        {
          std::cout << "Encoder calibration will be performed" << std::endl;
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
          // send the vertical angles to the parsers
          parser.get<PARSER_00_INDEX>().setVerticalAngles(vertical_angles);
          parser.get<PARSER_04_INDEX>().setVerticalAngles(vertical_angles);
        }
        else if (model.find("M8") != std::string::npos)
        {
          std::cout << "No vertical angle calibration information available on sensor, proceeding with M8 defaults" << std::endl;

          // tell parsers to use M8 defaults
          parser.get<PARSER_00_INDEX>().setVerticalAngles(quanergy::client::SensorType::M8);
          parser.get<PARSER_04_INDEX>().setVerticalAngles(quanergy::client::SensorType::M8);
        }
        else
        {
          throw quanergy::client::InvalidVerticalAngles("MQ sensor found with no vertical angles on the sensor");
        }
      }

      // Setup modules
      // Parsers
      auto &parser00 = parser.get<PARSER_00_INDEX>();
      auto &parser01 = parser.get<PARSER_01_INDEX>();
      auto &parser04 = parser.get<PARSER_04_INDEX>();

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
      parser04.setCloudSizeLimits(
        settings.min_cloud_size,
        settings.max_cloud_size
      );

      // Filters
      // Distance Filter
      distance_filter.setMaximumDistanceThreshold(settings.max_distance);
      distance_filter.setMinimumDistanceThreshold(settings.min_distance);

      // ring intensity filter
      for (int i = 0; i < quanergy::client::M8_NUM_LASERS; ++i)
      {
        ring_intensity_filter.setRingFilterMinimumRangeThreshold(
          i, settings.ring_range[i]
        );
        ring_intensity_filter.setRingFilterMinimumIntensityThreshold(
          i, settings.ring_intensity[i]
        );
      }

      // Connect modules
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
