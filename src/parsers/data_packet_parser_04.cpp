/****************************************************************
 **                                                            **
 **  Copyright(C) 2017 Quanergy Systems. All Rights Reserved.  **
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


    bool DataPacketParser04::parse(DataPacket04 const & data_packet, PointCloudHVDIRPtr & result)
    {
      bool success = false;

      StatusType current_status = StatusType(data_packet.data.data_header.status);

      if (current_status != StatusType::GOOD)
      {
        std::cerr << "Sensor status nonzero: " << data_packet.data.data_header.status;
        if (static_cast<std::uint16_t>(data_packet.data.data_header.status) & static_cast<std::uint16_t>(StatusType::SENSOR_SW_FW_MISMATCH))
        {
          throw FirmwareVersionMismatchError();
        }
        else if (static_cast<std::uint16_t>(data_packet.data.data_header.status) & static_cast<std::uint16_t>(StatusType::WATCHDOG_VIOLATION))
        {
          throw FirmwareWatchdogViolationError();
        }

        // Status flag is set, but the value is not known by this
        // version of the software.  Since the status is not
        // necessarily fatal, print the status value, but otherwise do
        // nothing.
      }

      if (current_status != previous_status_)
      {
        std::cerr << "Sensor status: " << std::uint16_t(current_status) << std::endl;

        previous_status_ = current_status;
      }


      // If the return selection has been explicitly set,
      // verify that the return ID matches what has been requested
      if (return_selection_ != quanergy::client::ALL_RETURNS &&
          data_packet.data.data_header.return_id != return_selection_)
      {
        throw ReturnIDMismatchError();
      }

      // this time is used for the cloud stamp which is a 64 bit integer in units of microseconds
      std::uint64_t current_packet_stamp =
        data_packet.packet_header.seconds * 1E6 + 
        data_packet.packet_header.nanoseconds * 1E-3;

      if (previous_packet_stamp_ == 0)
      {
        previous_packet_stamp_ = current_packet_stamp;
      }

      ++packet_counter_;

      M8DataPacket04 const & data = data_packet.data;

      // get spin direction
      // check 3 points in the packet to figure out which way it is spinning
      // if the measurements disagree, it could be wrap so we'll ignore that
      if ((data.firings[0].position - data.firings[M8_FIRING_PER_PKT/2].position < 0) &&
          (data.firings[M8_FIRING_PER_PKT/2].position - data.firings[M8_FIRING_PER_PKT-1].position < 0))
      {
        direction_ = 1;
      }
      else if ((data.firings[0].position - data.firings[M8_FIRING_PER_PKT/2].position > 0) && 
               (data.firings[M8_FIRING_PER_PKT/2].position - data.firings[M8_FIRING_PER_PKT-1].position > 0))
      {
        direction_ = -1;
      }

      // Tens of micrometers.
      double distance_scaling = 0.00001;

      bool cloudfull = (current_cloud_->size() >= maximum_cloud_size_);

      // for each firing
      for (int i = 0; i < M8_FIRING_PER_PKT; ++i)
      {
        M8FiringData04 const & firing = data.firings[i];

        // calculate the angle in degrees
        float azimuth_angle = (static_cast<float> ((firing.position+(M8_NUM_ROT_ANGLES/2))%M8_NUM_ROT_ANGLES) / float(M8_NUM_ROT_ANGLES) * 360.0f) - 180.0f;

        float delta_angle = 0;

        if ( cloud_counter_ == 0 && start_azimuth_ == 0 )
        {
          start_azimuth_ = azimuth_angle;
        } 
        else 
        {
          // calculate delta
          delta_angle = direction_*(azimuth_angle - start_azimuth_);
          while ( delta_angle < 0.0 )
          {
            delta_angle += 360.0;
          }
        }

        // check for wrap which indicates completion of a scan
        if ( (delta_angle >= degrees_per_cloud_) ||
             ((degrees_per_cloud_ == 360.0) &&
              (direction_ * azimuth_angle < direction_ * last_azimuth_)))
        {
          start_azimuth_ = azimuth_angle;

          if (current_cloud_->size() > minimum_cloud_size_)
          {
            if (cloudfull)
            {
              std::cerr << "Warning: Maximum cloud size limit of ("
                        << maximum_cloud_size_ << ") exceeded" << std::endl;
            }

            // interpolate the timestamp from the previous packet timestamp to the timestamp of this firing
            double const time_since_previous_packet =
              (current_packet_stamp - previous_packet_stamp_) * i / static_cast<double>(M8_FIRING_PER_PKT);
            std::uint64_t const current_firing_stamp = static_cast<uint64_t>(std::round(
                previous_packet_stamp_ + time_since_previous_packet));

            current_cloud_->header.stamp = current_firing_stamp;
            current_cloud_->header.seq = cloud_counter_;
            current_cloud_->header.frame_id = frame_id_;

            // can't organize if we kept all points
            organizeCloud(current_cloud_, worker_cloud_);

            ++cloud_counter_;

            // fire the signal that we have a new cloud
            result = current_cloud_;

            success = true;
          }
          else if(current_cloud_->size() > 0)
          {
            std::cerr << "Warning: Minimum cloud size limit of (" << minimum_cloud_size_
                      << ") not reached (" << current_cloud_->size() << ")" << std::endl;
          }

          // start a new cloud
          current_cloud_.reset(new PointCloudHVDIR());
          // at first we assume it is dense
          current_cloud_->is_dense = true;
          current_cloud_->reserve(maximum_cloud_size_);
          cloudfull = false;
        }

        if(cloudfull)
          continue;

        double const horizontal_angle = horizontal_angle_lookup_table_[firing.position];

        for (int j = 0; j < M8_NUM_LASERS; j++)
        {
          // output point
          PointCloudHVDIR::PointType hvdir;

          float const vertical_angle = vertical_angle_lookup_table_[j];

          hvdir.h = horizontal_angle;
          hvdir.v = vertical_angle;
          hvdir.ring = j;
          hvdir.intensity = firing.intensity[j];

          if (firing.radius[j] == 0)
          {
            hvdir.d = std::numeric_limits<float>::quiet_NaN();
            // if the range is NaN, the cloud is not dense
            current_cloud_->is_dense = false;
          }
          else
          {
            hvdir.d = static_cast<float>(firing.radius[j]) * distance_scaling; // convert range to meters
          }

          // add the point to the current scan
          current_cloud_->push_back(hvdir);
        }

        last_azimuth_ = azimuth_angle;
      }

      previous_packet_stamp_ = current_packet_stamp;

      return success;
    }
    
    
    bool DataPacketParser04::parse(const std::vector<char>& packet, PointCloudHVDIRPtr & result)
    {
      DataPacket04 data_packet;
      deserialize(packet.data(), data_packet);
      return parse(data_packet, result);
    }
  } // namespace client

} // namespace quanergy
