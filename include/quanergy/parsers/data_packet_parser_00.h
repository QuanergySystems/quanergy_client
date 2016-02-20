/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file pointcloud_generator_00.h
 *
 *   \brief Provide pointcloud generator functionality for data type 0x00.
 */

#ifndef QUANERGY_PARSERS_POINTCLOUD_GENERATOR_00_H
#define QUANERGY_PARSERS_POINTCLOUD_GENERATOR_00_H

#include <quanergy/parsers/packet_parser.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/parsers/data_packet_00.h>
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

    /** \brief specialization for DataPacket00 */
    template <>
    struct DLLEXPORT VariadicPacketParser<PointCloudHVDIRPtr, DataPacket00>
      : public PointCloudGeneratorM8
    {
      VariadicPacketParser<PointCloudHVDIRPtr, DataPacket00>()
        : PointCloudGeneratorM8()
      {}

      inline virtual bool validate(const std::vector<char>& packet)
      {
        const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());

        return (deserialize(h->packet_type) == 0x00);
      }

      inline virtual bool parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result)
      {
        DataPacket00 data_packet;
        deserialize(packet.data(), data_packet);
        return PointCloudGeneratorM8::parse(data_packet.data_body, result);
      }
    };

    typedef VariadicPacketParser<PointCloudHVDIRPtr, DataPacket00> DataPacket00Parser;

  } // namespace client

} // namespace quanergy

#endif
