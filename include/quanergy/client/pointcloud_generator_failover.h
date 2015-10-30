/** \file pointcloud_generator_failover.h
  * \brief provide pointcloud generator functionality for m8 failover
  */

#ifndef QUANERGY_POINTCLOUD_GENERATOR_FAILOVER_H
#define QUANERGY_POINTCLOUD_GENERATOR_FAILOVER_H

#include <quanergy/client/pointcloud_types.h>

#include <quanergy/client/packet_parser.h>
#include <quanergy/client/pointcloud_generator_m8.h>

namespace quanergy
{
  namespace client
  {
    /** \brief specialization for M8DataPacket */
    template <>
    struct PacketParser<PointCloudHVDIRPtr, M8DataPacket> : public PointCloudGeneratorM8
    {
      PacketParser(std::string const & frame_id)
        : PointCloudGeneratorM8(frame_id)
      {
      }

      static bool match(std::uint8_t type)
      {
        return type == 0xFF;
      }

      inline void parse(std::uint8_t type, const std::vector<char>& packet)
      {
        if (match(type))
        {
          // old data was sent in little endian
          const M8DataPacket& data_packet = *reinterpret_cast<const M8DataPacket*>(packet.data());
          PointCloudGeneratorM8::parse(data_packet);
        }
        else
          throw InvalidDataTypeError();
      }
    };

  } // namespace client

} // namespace quanergy
#endif
