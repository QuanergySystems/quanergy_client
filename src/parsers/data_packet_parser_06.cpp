/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/parsers/data_packet_parser_06.h>

namespace quanergy
{
  namespace client
  {

    bool DataPacketParser06::validate(std::vector<char> const & packet)
    {
      const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());

      return (deserialize(h->packet_type) == 0x06
              && deserialize(h->version_major) == 0x00
              && deserialize(h->version_minor) == 0x01
              && deserialize(h->version_patch) == 0x00);

    }

    bool DataPacketParser06::parse(const std::vector<char>& packet, PointCloudHVDIRPtr & result)
    {
      bool retval = false;

      const M1DataHeader* h = reinterpret_cast<const M1DataHeader*>(packet.data()+sizeof(PacketHeader));

      if (deserialize(h->return_id) == 3)
      {
        retval = parse<3>(packet, result); 
      }
      else
      {
        retval = parse<1>(packet, result);
      }

      return retval;
    }

  } // namespace client

} // namespace quanergy
