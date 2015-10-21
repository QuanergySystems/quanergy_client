/** \file pointcloud_generator_01.h
  * \brief provide pointcloud generator functionality for data type 0x01
  */

#ifndef QUANERGY_POINTCLOUD_GENERATOR_01_H
#define QUANERGY_POINTCLOUD_GENERATOR_01_H

#include "deserialize_01.h"
#include "pointcloud_generator.h"
#include "pcl/point_cloud.h"

namespace quanergy
{
  /** \brief specialization for DataPacket01 */
  template <>
  struct PointCloudGenerator<DataPacket01> : public PointCloudGeneratorBase
  {
    static bool match(std::uint8_t type)
    {
      return type == 0x01;
    }

    inline void toPointCloud(std::uint8_t type, const std::vector<char>& packet)
    {
      if (match(type))
      {
        // don't do the work unless someone is listening
        if (cloud_signal_ && cloud_signal_->num_slots() == 0)
          return;

        DataPacket01 data_packet;
        deserialize(packet.data(), data_packet);

        PointCloud::Ptr pc(new PointCloud);
        pc->header.stamp = (std::uint64_t)data_packet.packet_header.seconds * 1E9 + data_packet.packet_header.nanoseconds;
        pc->header.seq = data_packet.data_header.sequence;
        pc->resize(data_packet.data_header.point_count);
        for (unsigned int i = 0; i < data_packet.data_header.point_count; ++i)
        {
          const DataPoint01& point = data_packet.data_points[i];
          PointCloud::PointType& pc_point = pc->points[i];
          // get the distance to the XY plane
          double xy_distance = static_cast<double>(point.range) * 1E-6 *
                               std::cos(static_cast<double>(point.vertical_angle) * 1E-4);
          // set x
          pc_point.x = static_cast<float>(xy_distance *
                                          std::cos(static_cast<double>(point.horizontal_angle) * 1E-4));
          // set y
          pc_point.y = static_cast<float>(xy_distance *
                                           std::sin(static_cast<double>(point.horizontal_angle) * 1E-4));
          // set z
          pc_point.z = static_cast<float>(static_cast<double>(point.range) * 1E-6 *
                                          std::sin(static_cast<double>(point.vertical_angle) * 1E-4));
          // set I
          pc_point.intensity = point.intensity;
        }

        // signal
        (*cloud_signal_)(pc);
      }
      else
        throw InvalidDataTypeError();
    }
  };
}
#endif
