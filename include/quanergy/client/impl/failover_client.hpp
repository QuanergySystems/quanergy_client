/****************************************************************************
 **
 ** Copyright(C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/

#include <quanergy/client/failover_client.h>

namespace quanergy
{
  template <class... Types>
  FailoverClient<Types...>::FailoverClient(const std::string& host, const std::string& port)
    : Client<Types..., M8DataPacket>(host, port)
    , failover(false)
  {
  }

  template <class... Types>
  FailoverClient<Types...>::~FailoverClient()
  {
  }

  template <class... Types>
  void FailoverClient<Types...>::startDataRead()
  {
    if (failover)
    {
      // skip straight to body
      boost::asio::async_read(*read_socket_,
                boost::asio::buffer(buff_.data(), sizeof(M8DataPacket)),
                boost::bind(&FailoverClient<Types...>::handleReadBody, this,
                            boost::asio::placeholders::error));
    }
    else
    {
      Client<Types..., M8DataPacket>::startDataRead();
    }
  }

  template <class... Types>
  void FailoverClient<Types...>::handleReadHeader(const boost::system::error_code& error)
  {
    try
    {
      Client<Types..., M8DataPacket>::handleReadHeader(error);
    }
    catch (InvalidHeaderError)
    {
      std::cout << "Failover to old M8 Data" << std::endl;
      // failover
      failover = true;

      buff_.resize(sizeof(M8DataPacket));

      // read the rest of the packet
      boost::asio::async_read(*read_socket_,
                              boost::asio::buffer(buff_.data() + sizeof(DataPacketHeader),
                                                  sizeof(M8DataPacket) - sizeof(DataPacketHeader)),
                              boost::bind(&FailoverClient<Types...>::handleReadBody, this,
                                          boost::asio::placeholders::error));
    }
  }

  template <class... Types>
  void FailoverClient<Types...>::toPointCloud(const std::vector<char>& packet)
  {
    if (failover)
      point_cloud_generator_.toPointCloud(0xFF, packet);
    else
      Client<Types..., M8DataPacket>::toPointCloud(packet);
  }
}
