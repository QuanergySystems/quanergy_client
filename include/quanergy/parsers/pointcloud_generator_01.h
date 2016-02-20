/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file pointcloud_generator_01.h
 *
 *   \brief Provide pointcloud generator functionality for data type
 *   0x01.
 */

#ifndef QUANERGY_PARSERS_POINTCLOUD_GENERATOR_01_H
#define QUANERGY_PARSERS_POINTCLOUD_GENERATOR_01_H

#include <limits>

#include <quanergy/parsers/packet_parser.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/parsers/data_packet_01.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {

    /** \brief specialization for DataPacket01 */
    template <>
    struct DLLEXPORT VariadicPacketParser<PointCloudHVDIRPtr, DataPacket01>
      : public PacketParserBase<PointCloudHVDIRPtr>
    {
      VariadicPacketParser<PointCloudHVDIRPtr, DataPacket01>()
        : PacketParserBase<PointCloudHVDIRPtr>()
      {
      }

      inline virtual bool validate(const std::vector<char>& packet)
      {
        const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());

        return (deserialize(h->packet_type) == 0x01);
      }

      inline virtual bool parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result)
      {
        DataPacket01 data_packet;
        deserialize(packet.data(), data_packet);

        result.reset(new PointCloudHVDIR());

        // pcl pointcloud uses microseconds
        result->header.stamp =
          std::uint64_t(data_packet.packet_header.seconds) * 1E6 +
          std::uint64_t(data_packet.packet_header.nanoseconds) * 1E-3;

        result->header.seq = data_packet.data_header.sequence;

        result->resize(data_packet.data_header.point_count);

        for (unsigned int i = 0; i < data_packet.data_header.point_count; ++i)
        {
          DataPoint01 const & point = data_packet.data_points[i];
          PointCloudHVDIR::PointType& pc_point = result->points[i];

          pc_point.h = static_cast<double>(point.horizontal_angle) * 1E-4;
          pc_point.v = static_cast<double>(point.vertical_angle) * 1E-4;
          pc_point.d = static_cast<double>(point.range) * 1E-6;

          pc_point.intensity = point.intensity;

          /// This generator provides no ring.
          pc_point.ring = std::numeric_limits<uint16_t>::max();
        }

        return true;
      }
    };

    typedef VariadicPacketParser<PointCloudHVDIRPtr, DataPacket01> DataPacket01Parser;

  } // namespace client

} // namespace quanergy
#endif
