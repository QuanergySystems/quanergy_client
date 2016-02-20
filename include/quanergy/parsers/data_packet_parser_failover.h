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

#include <quanergy/parsers/packet_parser.h>
#include <quanergy/client/packet_header.h>

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
    struct DLLEXPORT VariadicPacketParser<PointCloudHVDIRPtr, M8DataPacket> : public PointCloudGeneratorM8
    {
      VariadicPacketParser()
        : PointCloudGeneratorM8()
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
        return PointCloudGeneratorM8::parse(*data_packet, result);
      }
    };

    typedef VariadicPacketParser<PointCloudHVDIRPtr, M8DataPacket> M8DataPacketParser;

  } // namespace client

} // namespace quanergy
#endif
