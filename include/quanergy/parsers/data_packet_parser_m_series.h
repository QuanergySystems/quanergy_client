/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file data_packet_parser_m_series.h
 *
 *   \brief Provide pointcloud parser functionality for M8 and MQ8 data.
 */
 
#ifndef QUANERGY_PARSERS_DATA_PACKET_PARSER_M_H
#define QUANERGY_PARSERS_DATA_PACKET_PARSER_M_H

#include <quanergy/parsers/data_packet_parser.h>

#include <quanergy/client/m_series_data_packet.h>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace client
  {

    static const double M8_VERTICAL_ANGLES[] = {
      -0.318505, 
      -0.2692, 
      -0.218009, 
      -0.165195, 
      -0.111003, 
      -0.0557982, 
      0., 
      0.0557982 };

    static const double MQ8_VERTICAL_ANGLES[] = {
      -0.24435,
      -0.18326,
      -0.14137,
      -0.10297,
      -0.07854,
      -0.055327,
      -0.041364,
      -0.027402 };

    enum struct SensorType {M8, MQ8};

    static const std::int32_t M_SERIES_NUM_ROT_ANGLES = 10400;

    /** \brief Used to specify 'all' returns */
    static const int ALL_RETURNS = -1;

    /** \brief limits cloud size for memory considerations; this is much larger than needed */
    static const std::int32_t MAX_CLOUD_SIZE = 1E6;

    /** \brief Not a specialization because it is intended to be used by others. */
    struct DLLEXPORT DataPacketParserMSeries : public DataPacketParser
    {
      DataPacketParserMSeries();

      void setReturnSelection(int return_selection);
      void setCloudSizeLimits(std::int32_t szmin, std::int32_t szmax);
      void setDegreesOfSweepPerCloud(double degrees_per_cloud);
      
      double getDegreesOfSweepPerCloud() const { return angle_per_cloud_*180./M_PI; }

      /// set vertical angles to use for M8/MQ8
      void setVerticalAngles(const std::vector<double>& vertical_angles);
      /// set vertical angles to the default values for the specified sensors
      void setVerticalAngles(SensorType sensor);

    protected:
      // validate status and throw error if appropriate, print message if changed
      void validateStatus(const StatusType& status);

      // register new packet for time and direction
      void registerNewPacket(const std::uint64_t& current_packet_stamp_ms,
        const int& start_pos, const int& mid_pos, const int& end_pos);

      // check whether the cloud is complete; if so, fill result and return true
      bool checkComplete(const float& azimuth_angle, PointCloudHVDIRPtr& result);
      
      // add firing of data
      void addFiring(const PointCloudHVDIRPtr& firing_cloud);

      // organize current_pc with height specified; throws if size not divisible by height
      void organizeCloud(PointCloudHVDIRPtr& current_pc,
        unsigned int height = M_SERIES_NUM_LASERS);

      /// global cloud counter
      std::uint32_t cloud_counter_ = 0;

      /// last accounted for azimuth angle
      double last_azimuth_ = 65000.;

      /// timestamp of previous data packet (microseconds)
      std::uint64_t current_packet_stamp_ms_ = 0;
      std::uint64_t previous_packet_stamp_ms_ = 0;

      /// cloud used for each firing defined here to reduce construct/resize costs
      PointCloudHVDIRPtr firing_cloud_;
      /// cloud that gets built up over time
      PointCloudHVDIRPtr current_cloud_;
      /// temp cloud for organization used to reduce construct and resize costs
      PointCloudHVDIRPtr worker_cloud_;

      /// lookup table for horizontal angle
      std::vector<double> horizontal_angle_lookup_table_;

      /// lookup table for vertical angle
      std::vector<double> vertical_angle_lookup_table_;

      /// return selection; default to return 0
      int return_selection_ = 0;
      /// whether return selection was explicitly set
      bool return_selection_set_ = false;

      /// cloud size limits
      std::int32_t minimum_cloud_size_ = 1;
      std::int32_t maximum_cloud_size_ = MAX_CLOUD_SIZE;
      
      /// cloud degrees of sweep
      double start_azimuth_ = 0.;
      double angle_per_cloud_ = 2*M_PI;

      /// direction
      int direction_ = 1; // start with an assumed direction until we can calculate

      /// previous status
      StatusType previous_status_ = StatusType::GOOD;

      /// firing number in packet
      int firing_number_ = 0;
    };

  } // namespace client

} // namespace quanergy

#endif
