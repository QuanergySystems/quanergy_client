/** \file deserialize.h
  * \brief provide base deserialization functionality
  *
  * Individual message types will need to specialize the functionality
  * provided here.
  */

#ifndef QUANERGY_DESERIALIZE_H
#define QUANERGY_DESERIALIZE_H

#include <cstdint>
#include <vector>
#include <arpa/inet.h>
#include <algorithm>

#include <quanergy/client/exceptions.h>

namespace quanergy
{
  namespace client
  {
    /// useful consts
    const std::uint32_t SIGNATURE     = 0x75bd7e97;
    const std::uint8_t  VERSION_MAJOR = 0;
    const std::uint8_t  VERSION_MINOR = 1;
    const std::uint8_t  VERSION_PATCH = 0;

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

    /** \brief interface for deserialize; to be specialized elsewhere */
    template<class T>
    inline void deserialize(const char* network_buffer, T& object) = delete;

    /** \brief deserialize specialization for header */
    template<>
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

  } // namespace client

} // namespace quanergy

#endif
