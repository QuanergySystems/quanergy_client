/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/


/**  \file pointcloud_generator_failover.h
  *
  *  \brief Provide pointcloud generator functionality for m8
  *  failover.
  */

#ifndef QUANERGY_PARSERS_POINTCLOUD_GENERATOR_FAILOVER_H
#define QUANERGY_PARSERS_POINTCLOUD_GENERATOR_FAILOVER_H

#include <quanergy/client/packet_parser.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/parsers/pointcloud_generator_m8.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {
    /** \brief specialization for M8DataPacket */
    template <>
	struct DLLEXPORT PacketParser<PointCloudHVDIRPtr, M8DataPacket> : public PointCloudGeneratorM8
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
