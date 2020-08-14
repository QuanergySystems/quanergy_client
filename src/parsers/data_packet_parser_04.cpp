/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/parsers/data_packet_parser_04.h>

namespace quanergy
{
  namespace client
  {

    bool DataPacketParser04::validate(std::vector<char> const & packet)
    {
      const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());

      return (deserialize(h->packet_type) == 0x04
              && deserialize(h->version_major) == 0x00
              && deserialize(h->version_minor) == 0x01
              && deserialize(h->version_patch) == 0x00);
    }

    bool DataPacketParser04::parse(const std::vector<char>& packet, PointCloudHVDIRPtr & result)
    {
      // deserialize
      DataPacket04 data_packet;
      deserialize(packet.data(), data_packet);

      // parse
      bool result_updated = false;

      // check status
      validateStatus(static_cast<StatusType>(data_packet.data.data_header.status));

      // check that vertical angles have been defined
      if (vertical_angle_lookup_table_.empty())
      {
        throw InvalidVerticalAngles("In parse, the vertical angle lookup table is empty; need to call setVerticalAngles.");
      }

      // If the return selection has been explicitly set,
      // verify that the return ID matches what has been requested
      if (return_selection_set_ &&
          return_selection_ != quanergy::client::ALL_RETURNS &&
          data_packet.data.data_header.return_id != return_selection_)
      {
        throw ReturnIDMismatchError();
      }

      // this time is used for the cloud stamp which is a 64 bit integer in units of microseconds
      std::uint64_t current_packet_stamp =
        static_cast<std::uint64_t>(data_packet.packet_header.seconds) * 1000000ull +
        static_cast<std::uint64_t>(data_packet.packet_header.nanoseconds) / 1000ull;

      const auto& start = data_packet.data.firings[0].position;
      const auto& mid   = data_packet.data.firings[M_SERIES_FIRING_PER_PKT/2].position;
      const auto& end   = data_packet.data.firings[M_SERIES_FIRING_PER_PKT-1].position;
      registerNewPacket(current_packet_stamp, start, mid, end);

      // Tens of micrometers.
      double distance_scaling = 0.00001;

      // for each firing
      for (int firing_index = 0; firing_index < M_SERIES_FIRING_PER_PKT; ++firing_index)
      {
        MSeriesFiringData04 const & firing = data_packet.data.firings[firing_index];
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
          hvdir.intensity = firing.intensity[laser_index];

          if (firing.radius[laser_index] == 0)
          {
            hvdir.d = std::numeric_limits<float>::quiet_NaN();
            // if the range is NaN, the cloud is not dense
            firing_cloud_->is_dense = false;
          }
          else
          {
            hvdir.d = static_cast<float>(firing.radius[laser_index]) * distance_scaling; // convert range to meters
          }

          // add the point to the current firing
          firing_cloud_->push_back(hvdir);

        } // for laser index

        // check whether cloud is complete
        bool complete = checkComplete(hvdir.h, result);
        // add firing to scan
        addFiring(firing_cloud_);

        // organize if appropriate
        if (complete)
        {
          organizeCloud(result, M_SERIES_NUM_LASERS);
        }

        result_updated = result_updated || complete;

      } // for firing index

      return result_updated;

    } // parse

  } // namespace client

} // namespace quanergy
