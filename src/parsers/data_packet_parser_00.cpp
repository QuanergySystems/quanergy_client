/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/parsers/data_packet_parser_00.h>

namespace quanergy
{
  namespace client
  {

    bool DataPacketParser00::validate(const std::vector<char>& packet)
    {
      const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());

      return (deserialize(h->packet_type) == 0x00
              && deserialize(h->version_major) == 0x00
              && deserialize(h->version_minor) == 0x01
              && deserialize(h->version_patch) == 0x00);
    }

    bool DataPacketParser00::parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result)
    {
      // deserialize
      DataPacket00 data_packet;
      deserialize(packet.data(), data_packet);

      // parse
      bool result_updated = false;

      // throws error if status is fatal
      validateStatus(static_cast<StatusType>(data_packet.data_body.status));

      // check that vertical angles have been defined
      if (vertical_angle_lookup_table_.empty())
      {
        throw InvalidVerticalAngles("In parse, the vertical angle lookup table is empty; need to call setVerticalAngles.");
      }

      // get the timestamp of the last point in the packet as 64 bit integer in units of microseconds
      std::uint64_t current_packet_stamp_ms;
      if (data_packet.data_body.version <= 3 && data_packet.data_body.version != 0)
      {
        // some versions of API put 10 ns increments in this field
        current_packet_stamp_ms = static_cast<std::uint64_t>(data_packet.packet_header.seconds) * 1000000ull
                               + static_cast<std::uint64_t>(data_packet.packet_header.nanoseconds) / 100ull;
      }
      else
      {
        current_packet_stamp_ms = static_cast<std::uint64_t>(data_packet.packet_header.seconds) * 1000000ull
                               + static_cast<std::uint64_t>(data_packet.packet_header.nanoseconds) / 1000ull;
      }

      const auto& start = data_packet.data_body.data[0].position;
      const auto& mid   = data_packet.data_body.data[M_SERIES_FIRING_PER_PKT/2].position;
      const auto& end   = data_packet.data_body.data[M_SERIES_FIRING_PER_PKT-1].position;
      registerNewPacket(current_packet_stamp_ms, start, mid, end);

      double distance_scaling = 0.01;
      if (data_packet.data_body.version >= 5)
      {
        distance_scaling = 0.00001;
      }

      // for each firing
      for (int firing_index = 0; firing_index < M_SERIES_FIRING_PER_PKT; ++firing_index)
      {
        const MSeriesFiringData &firing = data_packet.data_body.data[firing_index];
        firing_cloud_->clear();
        firing_cloud_->is_dense = true;
        PointCloudHVDIR::PointType hvdir;

        // populate firing cloud
        hvdir.h = horizontal_angle_lookup_table_[firing.position];

        // for each laser
        for (int laser_index = 0; laser_index < M_SERIES_NUM_LASERS; laser_index++)
        {
          hvdir.v = vertical_angle_lookup_table_[laser_index];
          hvdir.ring = laser_index;

          if (return_selection_ == quanergy::client::ALL_RETURNS)
          {
            // for the all case, we won't keep NaN points and we'll compare
            // distances to illiminate duplicates
            // index 0 (max return) could equal index 1 (first) and/or index 2 (last)
            hvdir.intensity = firing.returns_intensities[0][laser_index];
            std::uint32_t d = firing.returns_distances[0][laser_index];
            if (d != 0)
            {
              hvdir.d = static_cast<float>(d) * distance_scaling; // convert range to meters
              // add the point to the current firing
              firing_cloud_->push_back(hvdir);
            }

            if (firing.returns_distances[1][laser_index] != 0 && firing.returns_distances[1][laser_index] != d)
            {
              hvdir.intensity = firing.returns_intensities[1][laser_index];
              hvdir.d = static_cast<float>(firing.returns_distances[1][laser_index]) * distance_scaling; // convert range to meters
              // add the point to the current firing
              firing_cloud_->push_back(hvdir);
            }

            if (firing.returns_distances[2][laser_index] != 0 && firing.returns_distances[2][laser_index] != d)
            {
              hvdir.intensity = firing.returns_intensities[2][laser_index];
              hvdir.d = static_cast<float>(firing.returns_distances[2][laser_index]) * distance_scaling; // convert range to meters
              // add the point to the current firing
              firing_cloud_->push_back(hvdir);
            }

          } // if (return_selection_ == quanergy::client::ALL_RETURNS)
          else
          {
            // just want a single return case
            for (int return_index = 0; return_index < quanergy::client::M_SERIES_NUM_RETURNS; ++return_index)
            {
              if (return_selection_ == return_index)
              {
                hvdir.intensity = firing.returns_intensities[return_index][laser_index];

                if (firing.returns_distances[return_index][laser_index] == 0)
                {
                  hvdir.d = std::numeric_limits<float>::quiet_NaN();
                  // if the range is NaN, the cloud is not dense
                  firing_cloud_->is_dense = false;
                }
                else
                {
                  hvdir.d = static_cast<float>(firing.returns_distances[return_index][laser_index]) * distance_scaling; // convert range to meters
                }

                // add the point to the current firing
                firing_cloud_->push_back(hvdir);

              } // if (return_selection_ == return_index)

            } // for return index

          } // else (return_selection_ != quanergy::client::ALL_RETURNS)

        } // for laser index

        // check whether cloud is complete
        bool complete = checkComplete(hvdir.h, result);

        // add firing to scan
        addFiring(firing_cloud_);

        // organize if appropriate
        if (complete && return_selection_ != quanergy::client::ALL_RETURNS)
        {
          organizeCloud(result, M_SERIES_NUM_LASERS);
        }

        result_updated = result_updated || complete;

      } // for firing index

      return result_updated;

    } // parse

  } // namespace client

} // namespace quanergy
