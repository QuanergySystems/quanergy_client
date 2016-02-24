/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/parsers/data_packet_parser_failover.h>

namespace quanergy
{
  namespace client
  {
    DataPacketParserFailover::DataPacketParserFailover()
      : DataPacketParserM8()
    {
    }

    bool DataPacketParserFailover::validate(const std::vector<char>& packet)
    {
      const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());

      return (deserialize(h->signature) != SIGNATURE);
    }

    bool DataPacketParserFailover::parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result)
    {
      if (packet.size() != sizeof(M8DataPacket))
        throw SizeMismatchError();

      const M8DataPacket* data_packet = reinterpret_cast<const M8DataPacket*>(packet.data());
      return DataPacketParserM8::parse(*data_packet, result);
    }
  } // namespace client

} // namespace quanergy
