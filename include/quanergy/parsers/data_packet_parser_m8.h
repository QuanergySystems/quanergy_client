/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file data_packet_parser_m8.h
 *
 *   \brief Provide pointcloud parser functionality for m8 and MQ8 data.
 */
 
#ifndef QUANERGY_PARSERS_DATA_PACKET_PARSER_M8_H
#define QUANERGY_PARSERS_DATA_PACKET_PARSER_M8_H

#include <quanergy/parsers/data_packet_parser.h>

#include <quanergy/client/m8_data_packet.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {

    const double M8_VERTICAL_ANGLES[] = { 
      -0.318505, 
      -0.2692, 
      -0.218009, 
      -0.165195, 
      -0.111003, 
      -0.0557982, 
      0.f, 
      0.0557982 };

    const double MQ8_VERTICAL_ANGLES[] = {
      -0.24435,
      -0.18326,
      -0.14137,
      -0.10297,
      -0.07854,
      -0.055327,
      -0.041364,
      -0.027402 };

    enum struct SensorType {M8, MQ8};

    const std::int32_t M8_NUM_ROT_ANGLES = 10400;

    /** \brief Used to specify 'all' returns */
    const int ALL_RETURNS = -1;

    /** \brief limits cloud size for memory considerations; this is much larger than needed */
    const std::int32_t MAX_CLOUD_SIZE = 1E6;

    /** \brief Not a specialization because it is intended to be used by others. */
    struct DLLEXPORT DataPacketParserM8 : public DataPacketParser
    {
      DataPacketParserM8();

      virtual bool parse(const M8DataPacket& data_packet, PointCloudHVDIRPtr& result);

      void setReturnSelection(int return_selection);
      void setCloudSizeLimits(std::int32_t szmin, std::int32_t szmax);
      void setDegreesOfSweepPerCloud(double degrees_per_cloud);
      
      double getDegreesOfSweepPerCloud() const { return degrees_per_cloud_; }

      /// set vertical angles to use
      void setVerticalAngles(const std::vector<double>& vertical_angles);
      /// set vertical angles to the default values for the specified sensors
      void setVerticalAngles(SensorType sensor);

    protected:
      static void organizeCloud(PointCloudHVDIRPtr & current_pc,
                                PointCloudHVDIRPtr & temp_pc);

      /// global packet counter
      uint32_t packet_counter_;

      /// global cloud counter
      uint32_t cloud_counter_;

      /// last accounted for azimuth angle
      double last_azimuth_;

      /// timestamp of previous data packet (microseconds)
      std::uint64_t previous_packet_stamp_ = 0;

      PointCloudHVDIRPtr current_cloud_;
      PointCloudHVDIRPtr worker_cloud_;

      /// lookup table for horizontal angle
      std::vector<double> horizontal_angle_lookup_table_;

      /// lookup table for vertical angle
      std::vector<double> vertical_angle_lookup_table_;

      /// return selection
      int return_selection_ = 0;

      /// cloud size limits
      std::int32_t minimum_cloud_size_ = 1;
      std::int32_t maximum_cloud_size_ = MAX_CLOUD_SIZE;
      
      /// cloud degrees of sweep
      double start_azimuth_;
      double degrees_per_cloud_;

      /// direction
      int direction_ = 1; // start with an assumed direction until we can calculate

      /// previous status
      StatusType previous_status_ = StatusType::GOOD;
    };

  } // namespace client

} // namespace quanergy

#endif
