/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file data_packet_parser_failover.h
 *
 *   \brief Provide pointcloud parser functionality for old M8 data.
 */

#ifndef QUANERGY_PARSERS_DATA_PACKET_PARSER_FAILOVER_H
#define QUANERGY_PARSERS_DATA_PACKET_PARSER_FAILOVER_H

#include <quanergy/parsers/packet_parser.h>
#include <quanergy/client/packet_header.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/parsers/data_packet_parser_m8.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {
    struct DLLEXPORT DataPacketParserFailover : public DataPacketParserM8
    {
      DataPacketParserFailover()
        : DataPacketParserM8()
      {
      }

      inline virtual bool validate(const std::vector<char>& packet)
      {
        const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());

        return (deserialize(h->signature) != SIGNATURE);
      }

      inline virtual bool parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result)
      {
        if (packet.size() != sizeof(M8DataPacket))
          throw SizeMismatchError();

        const M8DataPacket* data_packet = reinterpret_cast<const M8DataPacket*>(packet.data());
        return DataPacketParserM8::parse(*data_packet, result);
      }
    };

  } // namespace client

} // namespace quanergy
#endif
