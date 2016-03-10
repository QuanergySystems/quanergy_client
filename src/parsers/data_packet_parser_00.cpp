/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/parsers/data_packet_parser_00.h>

namespace quanergy
{
  namespace client
  {
    DataPacketParser00::DataPacketParser00()
      : DataPacketParserM8()
    {}

    bool DataPacketParser00::validate(const std::vector<char>& packet)
    {
      const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());

      return (deserialize(h->packet_type) == 0x00
              && deserialize(h->version_major) == 0x00
              && deserialize(h->version_minor) == 0x01
              && deserialize(h->version_patch) == 0x00);
    }

    bool DataPacketParser00::parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result)
    {
      DataPacket00 data_packet;
      deserialize(packet.data(), data_packet);
      return DataPacketParserM8::parse(data_packet.data_body, result);
    }
  } // namespace client

} // namespace quanergy
