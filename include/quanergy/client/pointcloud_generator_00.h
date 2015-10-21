/** \file pointcloud_generator_00.h
  * \brief provide pointcloud generator functionality for data type 0x00
  */

#ifndef QUANERGY_POINTCLOUD_GENERATOR_00_H
#define QUANERGY_POINTCLOUD_GENERATOR_00_H

#include "deserialize_00.h"
#include "pointcloud_generator.h"
#include "pointcloud_generator_m8.h"
#include "pcl/point_cloud.h"

namespace quanergy
{
  /** \brief specialization for DataPacket00 */
  template <>
  struct PointCloudGenerator<DataPacket00> : public PointCloudGeneratorM8
  {
  public:
    PointCloudGenerator()
      : PointCloudGeneratorM8()
    {
    }

    static bool match(std::uint8_t type)
    {
      return type == 0x00;
    }

    inline void toPointCloud(std::uint8_t type, const std::vector<char>& packet)
    {
      if (match(type))
      {
        DataPacket00 data_packet;
        deserialize(packet.data(), data_packet);
        PointCloudGeneratorM8::toPointCloud(data_packet.data_body);
      }
      else
        throw InvalidDataTypeError();
    }
  };
}
#endif
