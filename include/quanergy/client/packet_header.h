/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file deserialize.h
  * \brief Provide base deserialization functionality.
  */

#ifndef QUANERGY_CLIENT_DESERIALIZE_H
#define QUANERGY_CLIENT_DESERIALIZE_H

#include <cstdint>
#include <vector>
#include <algorithm>

#ifdef _MSC_VER
  #include <Winsock2.h>
#else
  #include <arpa/inet.h>
#endif

#include <quanergy/client/exceptions.h>

#include <iostream>

namespace quanergy
{
  namespace client
  {
    /// useful consts
    const std::uint32_t SIGNATURE     = 0x75bd7e97;

#pragma pack(push, 1)
    /** \brief Header shared by all messages */
    struct PacketHeader
    {
      std::uint32_t signature; // SIGNATURE
      std::uint32_t size;      // bytes
      std::uint32_t seconds;
      std::uint32_t nanoseconds;
      std::uint8_t  version_major;
      std::uint8_t  version_minor;
      std::uint8_t  version_patch;
      std::uint8_t  packet_type;
    };
#pragma pack(pop)

    /// convenience functions for deserialization of objects
    inline std::uint8_t deserialize(std::uint8_t net_char)
    {
      return net_char;
    }

    inline std::uint16_t deserialize(std::uint16_t net_short)
    {
      return ntohs(net_short);
    }

    inline std::uint32_t deserialize(std::uint32_t net_long)
    {
      return ntohl(net_long);
    }

    inline std::int8_t deserialize(std::int8_t net_char)
    {
      return net_char;
    }

    inline std::int16_t deserialize(std::int16_t net_short)
    {
      return ntohs(net_short);
    }

    inline std::int32_t deserialize(std::int32_t net_long)
    {
      return ntohl(net_long);
    }

    /** \brief deserialize function for header */
    inline void deserialize(const char* network_buffer, PacketHeader& object)
    {
      const PacketHeader& network_order = *reinterpret_cast<const PacketHeader*>(network_buffer);

      object.signature     = deserialize(network_order.signature);
      object.size          = deserialize(network_order.size);
      object.seconds       = deserialize(network_order.seconds);
      object.nanoseconds   = deserialize(network_order.nanoseconds);
      object.version_major = deserialize(network_order.version_major);
      object.version_minor = deserialize(network_order.version_minor);
      object.version_patch = deserialize(network_order.version_patch);
      object.packet_type   = deserialize(network_order.packet_type);
    }

    inline bool validateHeader(const PacketHeader& object)
    {
      if (deserialize(object.signature) != SIGNATURE)
      {
        std::cerr << "Invalid header signature: " << std::hex << std::showbase
                  << object.signature << std::dec << std::noshowbase << std::endl;

        return false;
      }

      return true;
    }

    inline std::size_t getPacketSize(const PacketHeader& object)
    {
      return deserialize(object.size);
    }

  } // namespace client

} // namespace quanergy

#endif
