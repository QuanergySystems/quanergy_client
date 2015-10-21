/** \file pointcloud_generator_failover.h
  * \brief provide pointcloud generator functionality for m8 data
  */

#ifndef QUANERGY_POINTCLOUD_GENERATOR_M8_H
#define QUANERGY_POINTCLOUD_GENERATOR_M8_H

#include "deserialize_00.h"
#include "pointcloud_generator.h"
#include "pcl/point_cloud.h"

namespace quanergy
{
  const double M8_VERTICAL_ANGLES[] = { -0.318505, -0.2692, -0.218009, -0.165195, -0.111003, -0.0557982, 0.f, 0.0557982 };
  const int M8_NUM_ROT_ANGLES = 10400;

  /** \brief not a specialization because it is intended to be used by others */
  struct PointCloudGeneratorM8 : public PointCloudGeneratorBase
  {
    PointCloudGeneratorM8()
      : PointCloudGeneratorBase()
      , packet_counter_(0)
      , cloud_counter_(0)
      , last_azimuth_(65000)
      , current_cloud_(new PointCloud)
      , cos_lookup_table_(M8_NUM_ROT_ANGLES+1)
      , sin_lookup_table_(M8_NUM_ROT_ANGLES+1)
    {
      const double to_rad (M_PI / 180.f);
      for (unsigned int i = 0; i <= M8_NUM_ROT_ANGLES; i++)
      {
        double rad = to_rad * ((double (i) / M8_NUM_ROT_ANGLES) * 360.f);
        cos_lookup_table_[i] = std::cos (rad);
        sin_lookup_table_[i] = std::sin (rad);
      }

      const double* angle_in_radians = M8_VERTICAL_ANGLES;
      for (int i = 0; i < M8_NUM_LASERS; ++i, ++angle_in_radians)
      {
        sin_vertical_angles_[i] = std::sin (*angle_in_radians);
        cos_vertical_angles_[i] = std::cos (*angle_in_radians);
      }
    }

    inline void toPointCloud(const M8DataPacket& data_packet)
    {
      // don't do the work unless someone is listening
      if (cloud_signal_ && cloud_signal_->num_slots() == 0)
        return;

      if (data_packet.status != 0)
      {
        std::cerr << "Sensor status nonzero: " << data_packet.status;
        if (data_packet.status == 1)
          throw FirmwareVersionMismatchError();
        return; // don't process if sensor in error
      }

      time_t time;
      if (data_packet.version <= 3)
      {
        // early versions of API put 10 ns increments in this field
        time = data_packet.seconds * 1e9 + data_packet.nanoseconds * 10;
      }
      else
        time = data_packet.seconds * 1e9 + data_packet.nanoseconds;

      ++packet_counter_;

      int direction = 0;
      if (data_packet.data[0].position - data_packet.data[M8_FIRING_PER_PKT-1].position > 0)
        direction = (data_packet.data[0].position - data_packet.data[M8_FIRING_PER_PKT-1].position > 4000) ? 1 : -1;
      else
        direction = (data_packet.data[M8_FIRING_PER_PKT-1].position - data_packet.data[0].position > 4000) ? -1 : 1;

      for (int i = 0; i < M8_FIRING_PER_PKT; ++i)
      {
        const M8FiringData &data = data_packet.data[i];

        // calculate the angle in degrees
        double azimuth_angle = (static_cast<double> ((data.position+(M8_NUM_ROT_ANGLES/2))%M8_NUM_ROT_ANGLES) / (M8_NUM_ROT_ANGLES) * 360.0) - 180.;
        // check that the sensor is not spinning backward
        if (direction * azimuth_angle < direction * last_azimuth_)
        {
          if (current_cloud_->size () > 0)
          {
            organizeCloud(current_cloud_);

            current_cloud_->header.stamp = time;
            current_cloud_->header.seq = cloud_counter_;

            ++cloud_counter_;

            // fire the signal that we have a new cloud
            (*cloud_signal_)(current_cloud_);
          }

          // start a new cloud
          current_cloud_.reset(new PointCloud);
          // at first we assume it is dense
          current_cloud_->is_dense = true;
        }

        // get the cosine corresponding
        const double cos_horizontal_angle = cos_lookup_table_[data.position];
        // get the sine corresponding
        const double sin_horizontal_angle = sin_lookup_table_[data.position];

        for (int j = 0; j < M8_NUM_LASERS; j++)
        {
          // convert range to meters
          float range = data.returns_distances[0][j] * .01;
          unsigned char intensity = data.returns_intensities[0][j];

          if (range < 1E-4)
            range = std::numeric_limits<float>::quiet_NaN ();

          // output point
          PointCloud::PointType xyzi;
          // convert to cartezian coordinates and populate x, y and z members
          computeXYZ(range, cos_horizontal_angle, sin_horizontal_angle,
                      cos_vertical_angles_[j], sin_vertical_angles_[j], xyzi);

          // intensity value is fetched directly
          xyzi.intensity = intensity;
          // add the point to the current scan
          current_cloud_->push_back (xyzi);
          // if the range is NaN, the cloud is not dense, one point is sufficient
          if (current_cloud_->is_dense && std::isnan (range))
            current_cloud_->is_dense = false;
        }

        last_azimuth_ = azimuth_angle;
      }
    }

  private:
    static void organizeCloud(PointCloud::Ptr& current_pc)
    {
      // transpose the cloud
      PointCloud::Ptr temp_pc(new PointCloud);

      // reserve space
      temp_pc->reserve(current_pc->size());

      unsigned int temp_index;
      unsigned int width = current_pc->size () / M8_NUM_LASERS; // CONSTANT FOR NUM BEAMS

      // iterate through each ring from top down
      for (int i = M8_NUM_LASERS - 1; i >= 0; --i)
      {
        // iterate through width in collect order
        for (unsigned int j = 0; j < width; ++j)
        {
          // original data is in collect order and laser order
          temp_index = j * M8_NUM_LASERS + i;

          temp_pc->push_back(current_pc->points[temp_index]);
        }
      }

      current_pc.swap(temp_pc);

      current_pc->height = M8_NUM_LASERS;
      current_pc->width  = width;
    }

    static void computeXYZ (const double range,
                         const double cos_hz_angle, const double sin_hz_angle,
                         const double cos_vt_angle, const double sin_vt_angle,
                         pcl::PointXYZI& point)
    {
      if (std::isnan (range))
      {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
        return;
      }

      // get the distance to the XY plane
      double xy_distance = range * cos_vt_angle;
      // set y
      point.y = static_cast<float> (xy_distance * sin_hz_angle);
      // set x
      point.x = static_cast<float> (xy_distance * cos_hz_angle);
      // set z
      point.z = static_cast<float> (range * sin_vt_angle);
    }

    /// global packet counter
    uint32_t packet_counter_;

    /// global cloud counter
    uint32_t cloud_counter_;

    /// last accounted for azimuth angle
    double last_azimuth_;

    boost::shared_ptr<PointCloud> current_cloud_;

    /// lookup table for cosines
    std::vector<double> cos_lookup_table_;
    /// lookup table for sinus
    std::vector<double> sin_lookup_table_;
    double cos_vertical_angles_[M8_NUM_LASERS];
    double sin_vertical_angles_[M8_NUM_LASERS];
  };
}
#endif
