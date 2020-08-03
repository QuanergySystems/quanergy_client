/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_SENSOR_PIPELINE_H
#define QUANERGY_CLIENT_SENSOR_PIPELINE_H

#include <memory>

// parsers for the data packets we want to support
#include <quanergy/parsers/variadic_packet_parser.h>
#include <quanergy/parsers/data_packet_parser_00.h>
#include <quanergy/parsers/data_packet_parser_01.h>
#include <quanergy/parsers/data_packet_parser_04.h>
#include <quanergy/parsers/data_packet_parser_06.h>

// filters
#include <quanergy/modules/distance_filter.h>
#include <quanergy/modules/ring_intensity_filter.h>

// conversion module from polar to Cartesian
#include <quanergy/modules/polar_to_cart_converter.h>

// module to apply encoder correction
#include <quanergy/modules/encoder_angle_calibration.h>

// async module for multithreading
#include <quanergy/pipelines/async.h>

// for setting file
#include <quanergy/pipelines/sensor_pipeline_settings.h>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace pipeline
  {
    /** \brief SensorPipeline covers all of the processing from raw packets to PCL point cloud
      * \details This is strictly for convenience; everything is public so the user has access to modify things
      */
    struct DLLEXPORT SensorPipeline
    {
      // the parser type being used; this supports all Quanergy sensors by checking which of the provided
      // parsers supports the data packet recieved
      using Parser =  quanergy::client::VariadicPacketParser<quanergy::PointCloudHVDIRPtr,      // return type
                                                     quanergy::client::DataPacketParser00,      // PARSER_00_INDEX
                                                     quanergy::client::DataPacketParser01,      // PARSER_01_INDEX
                                                     quanergy::client::DataPacketParser04,      // PARSER_04_INDEX
                                                     quanergy::client::DataPacketParser06>;     // PARSER_06_INDEX

      // convenience enum for accessing the individual parsers
      enum
      {
        PARSER_00_INDEX = 0,
        PARSER_01_INDEX = 1,
        PARSER_04_INDEX = 2,
        PARSER_06_INDEX = 3
      };

      // the parser module type
      using ParserModule = quanergy::client::PacketParserModule<Parser>;

      // the parser module; converts raw packets to a polar PCL point cloud
      ParserModule parser;
      // encoder calibration; improves angular accuracy
      quanergy::calibration::EncoderAngleCalibration encoder_corrector;
      // distance filter; allows filtering by range
      quanergy::client::DistanceFilter distance_filter;
      // ring intensity filter; allows filtering by a combination of range and intensity
      quanergy::client::RingIntensityFilter ring_intensity_filter;
      // polar to cart converter; converts from the polar PCL cloud to a Cartesian one
      quanergy::client::PolarToCartConverter cartesian_converter;
      // async module to put the processing of the output cloud on a separate thread
      using AsyncType = quanergy::pipeline::AsyncModule<boost::shared_ptr<pcl::PointCloud<quanergy::PointXYZIR>>>;
      AsyncType async;


      // vector to hold connections for better cleanup
      std::vector<boost::signals2::connection> connections;

      /** \brief constructor configures the pipeline based on the provided settings
       *  \param settings is the settings to use
       */
      SensorPipeline(const SensorPipelineSettings& settings);

      /// \brief destructor
      virtual ~SensorPipeline();

      /** \brief slot simply calls the parser slot
       *  \param the raw packet data
       */
      void slot(const std::shared_ptr<std::vector<char>>& packet)
      {
        parser.slot(packet);
      }

      /** \brief connect is just a convenience calling the polar to cart converters connect method
       *  \param subscriber is the slot to call; it is a function consuming
       *         const boost::shared_ptr<pcl::PointCloud<quanergy::PointXYZIR>>&
       *  \returns connection object created
       */
      boost::signals2::connection connect(
          const typename AsyncType::Signal::slot_type& subscriber)
      {
        return async.connect(subscriber);
      }
    };
  }
}

#endif
