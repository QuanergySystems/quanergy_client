/** \file pointcloud_generator_failover.h
  * \brief provide pointcloud generator functionality for m8 failover
  */

#ifndef QUANERGY_POINTCLOUD_GENERATOR_FAILOVER_H
#define QUANERGY_POINTCLOUD_GENERATOR_FAILOVER_H

#include "pointcloud_generator.h"
#include "pointcloud_generator_m8.h"
#include "pcl/point_cloud.h"

namespace quanergy
{
  /** \brief specialization for M8DataPacket */
  template <>
  struct PointCloudGenerator<M8DataPacket> : public PointCloudGeneratorM8
  {
    PointCloudGenerator()
      : PointCloudGeneratorM8()
    {
    }

    static bool match(std::uint8_t type)
    {
      return type == 0xFF;
    }

    inline void toPointCloud(std::uint8_t type, const std::vector<char>& packet)
    {
      if (match(type))
      {
        // old data was sent in little endian
        const M8DataPacket& data_packet = *reinterpret_cast<const M8DataPacket*>(packet.data());
        PointCloudGeneratorM8::toPointCloud(data_packet);
      }
      else
        throw InvalidDataTypeError();
    }
  };
}
#endif
