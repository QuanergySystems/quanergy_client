/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**
 *  @file data_packet_parser_06.h
 *
 *  @brief Provide pointcloud parser functionality for data type 0x06.
 */

#ifndef QUANERGY_CLIENT_PARSERS_DATA_PACKET_PARSER_06_H
#define QUANERGY_CLIENT_PARSERS_DATA_PACKET_PARSER_06_H

#include <quanergy/parsers/packet_parser.h>

#include <quanergy/parsers/data_packet_06.h>
#include <quanergy/parsers/data_packet_parser_m_series.h>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace client
  {
    class DLLEXPORT DataPacketParser06 : public DataPacketParserMSeries
    {
    public:
      // Constructor
      DataPacketParser06() = default;

      virtual bool validate(const std::vector<char>& packet) override;
  
      virtual bool parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result) override;

    private:
      // templated parse method for M1 (only valid for 1 or 3 returns)
      template<std::uint8_t R>
      inline typename std::enable_if<R == 1 || R == 3, bool>::type parse(
                        const std::vector<char>& packet, PointCloudHVDIRPtr& result)
      {
        // deserialize
        DataPacket06<R> data_packet;
        deserialize(packet.data(), data_packet);

        // parse
        bool result_updated = false;

        // throws error if status is fatal
        validateStatus(static_cast<StatusType>(data_packet.data_header.status));

        // If the return selection has been explicitly set,
        // verify that the return ID matches what has been requested
        if (R == 1 && return_selection_set_ &&
            data_packet.data_header.return_id != return_selection_)
        {
          throw ReturnIDMismatchError();
        }

        // this time is used for the cloud stamp which is a 64 bit integer in units of microseconds
        std::uint64_t current_packet_stamp_ms =
          static_cast<std::uint64_t>(data_packet.packet_header.seconds) * 1000000ull +
          static_cast<std::uint64_t>(data_packet.packet_header.nanoseconds) / 1000ull;

        const auto& start = data_packet.data.firings[0].position;
        const auto& mid   = data_packet.data.firings[M_SERIES_FIRING_PER_PKT/2].position;
        const auto& end   = data_packet.data.firings[M_SERIES_FIRING_PER_PKT-1].position;
        registerNewPacket(current_packet_stamp_ms, start, mid, end);

        // Tens of micrometers.
        double distance_scaling = 0.00001;

        // for each firing
        for (int firing_index = 0; firing_index < M_SERIES_FIRING_PER_PKT; ++firing_index)
        {
          M1FiringData<R> const & firing = data_packet.data.firings[firing_index];
          firing_cloud_->clear();
          firing_cloud_->is_dense = true;
          PointCloudHVDIR::PointType hvdir;

          // populate firing cloud
          hvdir.h = horizontal_angle_lookup_table_[firing.position];
          hvdir.v = 0.;
          hvdir.ring = 0;

          if (R == M_SERIES_NUM_RETURNS && return_selection_ == quanergy::client::ALL_RETURNS)
          {
            // for the all case, we won't keep NaN points and we'll compare
            // distances to illiminate duplicates
            // index 2 could equal index 0 and/or index 1
            // index 1 could equal index 0 but only if all 3 are equal so don't need to check that as separate case
            std::uint32_t dist2 = firing.radius[2];

            std::uint32_t dist0 = firing.radius[0];
            if (dist0 != 0 && dist0 != dist2)
            {
              hvdir.intensity = firing.intensity[0];
              hvdir.d = static_cast<float>(dist0) * distance_scaling; // convert range to meters
              // add the point to the current firing
              firing_cloud_->push_back(hvdir);
            }

            std::uint32_t dist1 = firing.radius[1];
            if (dist1 != 0 && dist1 != dist2)
            {
              hvdir.intensity = firing.intensity[1];
              hvdir.d = static_cast<float>(dist1) * distance_scaling; // convert range to meters
              // add the point to the current firing
              firing_cloud_->push_back(hvdir);
            }

            if (dist2 != 0)
            {
              hvdir.intensity = firing.intensity[2];
              hvdir.d = static_cast<float>(dist2) * distance_scaling; // convert range to meters
              // add the point to the current firing
              firing_cloud_->push_back(hvdir);
            }

          } // if (R == M_SERIES_NUM_RETURNS && return_selection_ == quanergy::client::ALL_RETURNS)
          else if(R == M_SERIES_NUM_RETURNS)
          {
            // We only want 1 return, find the correct one
            hvdir.intensity = firing.intensity[return_selection_];

            if (firing.radius[return_selection_] == 0)
            {
              hvdir.d = std::numeric_limits<float>::quiet_NaN();
              // if the range is NaN, the cloud is not dense
              firing_cloud_->is_dense = false;
            }
            else
            {
              hvdir.d = static_cast<float>(firing.radius[return_selection_]) * distance_scaling; // convert range to meters
            }

            // add the point to the current firing
            firing_cloud_->push_back(hvdir);

          } // else if (R == M_SERIES_NUM_RETURNS)
          else
          {
            // single return case
            hvdir.intensity = firing.intensity[0];

            if (firing.radius[0] == 0)
            {
              hvdir.d = std::numeric_limits<float>::quiet_NaN();
              // if the range is NaN, the cloud is not dense
              firing_cloud_->is_dense = false;
            }
            else
            {
              hvdir.d = static_cast<float>(firing.radius[0]) * distance_scaling; // convert range to meters
            }

            // add the point to the current firing
            firing_cloud_->push_back(hvdir);

          } // else (R != M_SERIES_NUM_RETURNS)

          // check whether cloud is complete
          bool complete = checkComplete(hvdir.h, result);
          // add firing to scan
          addFiring(firing_cloud_);

          // with height of 1, there is no need to organize

          result_updated = result_updated || complete;

        } // for firing index

        return result_updated;

      } // parse

    };

  } // namespace client

} // namespace quanergy

#endif
