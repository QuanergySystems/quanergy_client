/** \file pointcloud_generator_01.h
 *
 *  \brief Provide pointcloud generator functionality for data type 0x01.
 */

#ifndef QUANERGY_POINTCLOUD_GENERATOR_01_H
#define QUANERGY_POINTCLOUD_GENERATOR_01_H

#include <quanergy/client/pointcloud_types.h>

#include <quanergy/client/deserialize_01.h>
#include <quanergy/client/packet_parser.h>


namespace quanergy
{
  namespace client
  {

    /** \brief specialization for DataPacket01 */
    template <>
    struct PacketParser<PointCloudXYZIPtr, DataPacket01> 
      : PacketParserBase<PointCloudXYZIPtr>
    {
      PacketParser<PointCloudXYZIPtr, DataPacket01>(std::string const & frame_id)
        : PacketParserBase<PointCloudXYZIPtr>(frame_id)
      {
      }

      static bool match(std::uint8_t type)
      {
        return type == 0x01;
      }

      inline void parse(std::uint8_t type, const std::vector<char>& packet)
      {
        if (match(type))
        {
          // don't do the work unless someone is listening
          if (signal_ && signal_->num_slots() == 0)
            return;

          DataPacket01 data_packet;
          deserialize(packet.data(), data_packet);

          PointCloudXYZIPtr pc = PointCloudXYZIPtr(new PointCloudXYZI());

          pc->header.stamp = 
            std::uint64_t(data_packet.packet_header.seconds) * 1E9 + 
            std::uint64_t(data_packet.packet_header.nanoseconds);

          pc->header.seq = data_packet.data_header.sequence;

          pc->header.frame_id = frame_id_;

          pc->resize(data_packet.data_header.point_count);

          for (unsigned int i = 0; i < data_packet.data_header.point_count; ++i)
          {
            DataPoint01 const & point = data_packet.data_points[i];
            PointCloudXYZI::PointType& pc_point = pc->points[i];
            // get the distance to the XY plane
            double xy_distance = static_cast<double>(point.range) * 1E-6 *
              std::cos(static_cast<double>(point.vertical_angle) * 1E-4);
            // set x
            pc_point.x = static_cast<float>(xy_distance *
                                            std::cos(static_cast<double>(point.horizontal_angle) * 1E-4));
            // set y
            pc_point.y = static_cast<float>(xy_distance *
                                            std::sin(static_cast<double>(point.horizontal_angle) * 1E-4));
            // set z
            pc_point.z = static_cast<float>(static_cast<double>(point.range) * 1E-6 *
                                            std::sin(static_cast<double>(point.vertical_angle) * 1E-4));
            // set I
            pc_point.intensity = point.intensity;
          }

          // signal
          (*signal_)(pc);
        }
        else
          throw InvalidDataTypeError();
      }
    };

  } // namespace client

} // namespace quanergy
#endif
