/**************************************************************** 
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/parsers/data_packet_parser_m_series.h>

namespace quanergy
{
  namespace client
  {

    DataPacketParserMSeries::DataPacketParserMSeries()
      : firing_cloud_(new PointCloudHVDIR())
      , current_cloud_(new PointCloudHVDIR())
      , worker_cloud_(new PointCloudHVDIR())
      , horizontal_angle_lookup_table_(M_SERIES_NUM_ROT_ANGLES+1)
    {
      // Reserve space ahead of time for incoming data
      current_cloud_->reserve(maximum_cloud_size_);
      worker_cloud_->reserve(maximum_cloud_size_);

      for (std::uint32_t i = 0; i <= M_SERIES_NUM_ROT_ANGLES; i++)
      {
        // Shift by half the rot angles to keep the number positive when wrapping.
        std::uint32_t j = (i + M_SERIES_NUM_ROT_ANGLES/2) % M_SERIES_NUM_ROT_ANGLES;

        // normalized
        double n = static_cast<double>(j) / static_cast<double>(M_SERIES_NUM_ROT_ANGLES);

        double rad = n * M_PI * 2.0 - M_PI;

        horizontal_angle_lookup_table_[i] = rad;
      }
    }

    void DataPacketParserMSeries::setReturnSelection(int return_selection)
    {
      if ((return_selection != quanergy::client::ALL_RETURNS) &&
          (return_selection < 0 || return_selection >= M_SERIES_NUM_RETURNS))
      {
        throw InvalidReturnSelection();
      }

      return_selection_ = return_selection;
      return_selection_set_ = true;
    }

    void DataPacketParserMSeries::setCloudSizeLimits(std::int32_t szmin, std::int32_t szmax)
    {
      if(szmin > MAX_CLOUD_SIZE || szmax > MAX_CLOUD_SIZE)
      {
        throw std::invalid_argument(std::string("Cloud size limits cannot be larger than ")
                                    + std::to_string(MAX_CLOUD_SIZE));
      }

      if(szmin > 0)
        minimum_cloud_size_ = std::max(1,szmin);
      if(szmax > 0)
        maximum_cloud_size_ = std::max(minimum_cloud_size_,szmax);
    }
    
    void DataPacketParserMSeries::setDegreesOfSweepPerCloud(double degrees_per_cloud)
    {
      if ( degrees_per_cloud < 0 || degrees_per_cloud > 360.0 ) 
      {
        throw InvalidDegreesPerCloud();
      }
      angle_per_cloud_ = degrees_per_cloud*M_PI/180.;
    }

    void DataPacketParserMSeries::setVerticalAngles(const std::vector<double> &vertical_angles)
    {
      // this is only intended for M8/MQ8
      if (vertical_angles.size() != M_SERIES_NUM_LASERS)
      {
        throw InvalidVerticalAngles(std::string("Vertical Angles must be size: ")
                                    + std::to_string(M_SERIES_NUM_LASERS)
                                    + "; got a vector of length: "
                                    + std::to_string(vertical_angles.size()));
      }

      vertical_angle_lookup_table_ = vertical_angles;

    }

    void DataPacketParserMSeries::setVerticalAngles(SensorType sensor)
    {
      if (sensor == SensorType::M8)
      {
        std::vector<double> vertical_angles(quanergy::client::M8_VERTICAL_ANGLES,
                                            quanergy::client::M8_VERTICAL_ANGLES + quanergy::client::M_SERIES_NUM_LASERS);
        setVerticalAngles(vertical_angles);
      }
      else if (sensor == SensorType::MQ8)
      {
        std::vector<double> vertical_angles(quanergy::client::MQ8_VERTICAL_ANGLES,
                                            quanergy::client::MQ8_VERTICAL_ANGLES + quanergy::client::M_SERIES_NUM_LASERS);
        setVerticalAngles(vertical_angles);
      }
    }

    void DataPacketParserMSeries::validateStatus(const StatusType& status)
    {
      if (status != StatusType::GOOD)
      {
        if (static_cast<std::uint16_t>(status) & static_cast<std::uint16_t>(StatusType::SENSOR_SW_FW_MISMATCH))
        {
          throw FirmwareVersionMismatchError();
        }
        else if (static_cast<std::uint16_t>(status) & static_cast<std::uint16_t>(StatusType::WATCHDOG_VIOLATION))
        {
          throw FirmwareWatchdogViolationError();
        }

        // Status flag is set, but the value is not currently known in
        // this version of the software.  Since the status is not
        // necessarily fatal, print the status value, but otherwise do
        // nothing.
      }

      if (status != previous_status_)
      {
        std::cerr << "Sensor status: " << std::uint16_t(status) << std::endl;

        previous_status_ = status;
      }
    }

    // register new packet for time and direction
    void DataPacketParserMSeries::registerNewPacket(const std::uint64_t& current_packet_stamp_ms,
      const int& start_pos, const int& mid_pos, const int& end_pos)
    {
      if (previous_packet_stamp_ms_ == 0)
      {
        previous_packet_stamp_ms_ = current_packet_stamp_ms;
      }
      else
      {
        previous_packet_stamp_ms_ = current_packet_stamp_ms_;
      }

      current_packet_stamp_ms_  = current_packet_stamp_ms;

      // get spin direction
      // check 3 points in the packet to figure out which way it is spinning
      // if the measurements disagree, it could be wrap so we'll ignore that
      if (start_pos - mid_pos < 0 && mid_pos - end_pos < 0)
      {
        direction_ = 1;
      }
      else if (start_pos - mid_pos > 0 && mid_pos - end_pos > 0)
      {
        direction_ = -1;
      }

      firing_number_ = 0;
    }

    bool DataPacketParserMSeries::checkComplete(const float& azimuth_angle, PointCloudHVDIRPtr& result)
    {
      bool result_updated = false;

      bool cloudfull = (current_cloud_->size() >= maximum_cloud_size_);

      // get swept angle
      double delta_angle = 0;
      if (cloud_counter_ == 0 && start_azimuth_ == 0)
      {
        start_azimuth_ = azimuth_angle;
      } 
      else 
      {
        // calculate delta
        delta_angle = direction_ * (azimuth_angle - start_azimuth_);
        while (delta_angle < 0.0)
        {
          delta_angle += 2*M_PI;
        }
      }
        
      // check if we've completed the sweep; if so complete the point cloud
      if (delta_angle >= angle_per_cloud_ || (angle_per_cloud_==2*M_PI && (direction_*azimuth_angle < direction_*last_azimuth_)))
      {
        start_azimuth_ = azimuth_angle;
        if (current_cloud_->size () > minimum_cloud_size_)
        {
          // we have a successful packet

          if(cloudfull)
          {
            std::cout << "Warning: Maximum cloud size limit of ("
                << maximum_cloud_size_ << ") exceeded" << std::endl;
          }

          // interpolate the timestamp from the previous packet timestamp to the timestamp of this firing
          const double time_since_previous_packet_ms =
              static_cast<double>((current_packet_stamp_ms_ - previous_packet_stamp_ms_) * firing_number_) 
              / static_cast<double>(M_SERIES_FIRING_PER_PKT);
          const std::uint64_t current_firing_stamp = previous_packet_stamp_ms_
              + static_cast<std::uint64_t>(std::round(time_since_previous_packet_ms));

          current_cloud_->header.stamp = current_firing_stamp;
          current_cloud_->header.seq = cloud_counter_;
          current_cloud_->header.frame_id = frame_id_;

          ++cloud_counter_;

          // fire the signal that we have a new cloud
          result = current_cloud_;
          // set the size which until organized is 1 x num_points
          result->height = 1;
          result->width = result->size();
          result_updated = true;
        }
        else if(current_cloud_->size() > 0)
        {
          std::cout << "Warning: Minimum cloud size limit of (" << minimum_cloud_size_
              << ") not reached (" << current_cloud_->size() << ")" << std::endl;
        }

        // start a new cloud
        current_cloud_.reset(new PointCloudHVDIR());
        // at first we assume it is dense
        current_cloud_->is_dense = true;
        current_cloud_->reserve(maximum_cloud_size_);
        cloudfull = false;
      }

      last_azimuth_ = azimuth_angle;

      return result_updated;
    }

    void DataPacketParserMSeries::addFiring(const PointCloudHVDIRPtr& firing_cloud)
    {
      if (firing_cloud_->empty())
        return;

      bool cloudfull = (current_cloud_->size() >= maximum_cloud_size_);

      // if the cloud isn't full, add the firing
      if (!cloudfull)
      {
        ++firing_number_;

        current_cloud_->points.insert(current_cloud_->points.end(),
          firing_cloud->points.begin(), firing_cloud->points.end());

        current_cloud_->is_dense = current_cloud_->is_dense && firing_cloud->is_dense;
      }
    }

    void DataPacketParserMSeries::organizeCloud(PointCloudHVDIRPtr& current_pc, 
                                                unsigned int height)
    {
      if (height == 0 || current_pc->size() % height != 0)
      {
        throw std::runtime_error("Can't organize cloud when size is not divisible by height");
      }

      unsigned int width = current_pc->size() / height;
      
      // if height is 1, there is no need to transpose the cloud
      if (height != 1)
      {
        // transpose the cloud
        worker_cloud_->clear();

        worker_cloud_->header.stamp = current_pc->header.stamp;
        worker_cloud_->header.seq = current_pc->header.seq;
        worker_cloud_->header.frame_id = current_pc->header.frame_id;

        // reserve space
        worker_cloud_->reserve(current_pc->size());

        unsigned int temp_index;

        // iterate through each ring from top down
        for (int i = height - 1; i >= 0; --i)
        {
          // iterate through width in collect order
          for (unsigned int j = 0; j < width; ++j)
          {
            // original data is in collect order and laser order
            temp_index = j * height + i;

            worker_cloud_->push_back(current_pc->points[temp_index]);
          }
        }

        current_pc.swap(worker_cloud_);
      }

      current_pc->height = height;
      current_pc->width  = width;
    }

  } // namespace client

} // namespace quanergy
