/** \file pointcloud_generator_00.h
  * \brief provide pointcloud generator functionality for data type 0x00
  */

#ifndef QUANERGY_POINTCLOUD_GENERATOR_00_H
#define QUANERGY_POINTCLOUD_GENERATOR_00_H

#include <quanergy/client/pointcloud_types.h>

#include <quanergy/client/packet_parser.h>
#include <quanergy/client/deserialize_00.h>
#include <quanergy/client/pointcloud_generator_m8.h>


namespace quanergy
{
  /** \brief specialization for DataPacket00 */
  template <>
  struct PacketParser<PointCloudXYZIPtr, DataPacket00> 
    : public PointCloudGeneratorM8
  {
  public:
    PacketParser()
      : PointCloudGeneratorM8()
    {
    }

    static bool match(std::uint8_t type)
    {
      return type == 0x00;
    }

    inline void parse(std::uint8_t type, const std::vector<char>& packet)
    {
      if (match(type))
      {
        DataPacket00 data_packet;
        deserialize(packet.data(), data_packet);
        PointCloudGeneratorM8::parse(data_packet.data_body);
      }
      else
        throw InvalidDataTypeError();
    }
  };
}
#endif
