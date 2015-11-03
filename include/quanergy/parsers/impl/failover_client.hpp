/****************************************************************************
 **
 ** Copyright(C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/

#include <quanergy/parsers/failover_client.h>

namespace quanergy
{
  namespace client
  {

    template <class... TYPES>
    FailoverClient<TYPES...>::FailoverClient(std::string const & host, 
                                             std::string const & port, 
                                             std::string const & frame_id)
      : Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>(host, port, frame_id)
      , failover_(false)
    {
    }


    template <class... TYPES>
    FailoverClient<TYPES...>::~FailoverClient()
    {
    }


    template <class... TYPES>
    void FailoverClient<TYPES...>::startDataRead()
    {
      if (failover_)
      {
        // skip straight to body
        boost::asio::async_read(*read_socket_,
                                boost::asio::buffer(buff_.data(), sizeof(M8DataPacket)),
                                boost::bind(&FailoverClient<TYPES...>::handleReadBody, this,
                                            boost::asio::placeholders::error));
      }
      else
      {
        Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>::startDataRead();
      }
    }


    template <class... TYPES>
    void FailoverClient<TYPES...>::handleReadHeader(const boost::system::error_code& error)
    {
      try
      {
        Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>::handleReadHeader(error);
      }
      catch (InvalidHeaderError)
      {
        std::cout << "Failover to old M8 Data" << std::endl;
        // failover
        failover_ = true;

        buff_.resize(sizeof(M8DataPacket));

        // read the rest of the packet
        boost::asio::async_read(*read_socket_,
                                boost::asio::buffer(buff_.data() + sizeof(PacketHeader),
                                                    sizeof(M8DataPacket) - sizeof(PacketHeader)),
                                boost::bind(&FailoverClient<TYPES...>::handleReadBody, this,
                                            boost::asio::placeholders::error));
      }
    }

    template <class... TYPES>
    void FailoverClient<TYPES...>::parse(const std::vector<char>& packet)
    {
      if (failover_)
        parser_.parse(0xFF, packet);
      else
        Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>::parse(packet);
    }

  } // namespace client

} // namespace quanergy
