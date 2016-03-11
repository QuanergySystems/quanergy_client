/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/parsers/data_packet_parser_01.h>

namespace quanergy
{
  namespace client
  {
    DataPacketParser01::DataPacketParser01()
      : DataPacketParser()
    {
    }

    bool DataPacketParser01::validate(const std::vector<char>& packet)
    {
      const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());

      return (deserialize(h->packet_type) == 0x01
              && deserialize(h->version_major) == 0x00
              && deserialize(h->version_minor) == 0x01
              && deserialize(h->version_patch) == 0x00);
    }

    bool DataPacketParser01::parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result)
    {
      DataPacket01 data_packet;
      deserialize(packet.data(), data_packet);

      result.reset(new PointCloudHVDIR());

      // pcl pointcloud uses microseconds
      result->header.stamp =
          std::uint64_t(data_packet.packet_header.seconds) * 1E6 +
          std::uint64_t(data_packet.packet_header.nanoseconds) * 1E-3;

      result->header.seq = data_packet.data_header.sequence;
      result->header.frame_id = frame_id_;

      result->resize(data_packet.data_header.point_count);

      // intermediate angles and value
      double H = 0;
      double V = 0;
      double cosH = 0;

      for (unsigned int i = 0; i < data_packet.data_header.point_count; ++i)
      {
        DataPoint01 const & point = data_packet.data_points[i];
        PointCloudHVDIR::PointType& pc_point = result->points[i];

        H = static_cast<double>(point.horizontal_angle) * 1E-4;
        V = static_cast<double>(point.vertical_angle) * 1E-4;
        pc_point.d = static_cast<double>(point.range) * 1E-6;

        // convert to standard HVDIR
        cosH = std::cos(H);
        pc_point.h = std::atan2(std::sin(H), cosH * std::cos(V));
        pc_point.v = std::asin(cosH * std::sin(V));

        pc_point.intensity = point.intensity;

        /// This generator provides no ring.
        pc_point.ring = std::numeric_limits<uint16_t>::max();
      }

      return true;
    }
  } // namespace client

} // namespace quanergy
