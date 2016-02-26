/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/client/failover_client.h>

#include <quanergy/client/m8_data_packet.h>

namespace quanergy
{
  namespace client
  {

    FailoverClient::FailoverClient(std::string const & host,
                                   std::string const & port,
                                   std::size_t max_queue_size)
      : SensorClient(host, port, max_queue_size)
    {
    }

    void FailoverClient::startDataRead()
    {
      if (failover_)
      {
        // skip straight to body
        boost::asio::async_read(*read_socket_,
                                boost::asio::buffer(buff_.data(), sizeof(M8DataPacket)),
                                boost::bind(&FailoverClient::handleReadBody, this,
                                            boost::asio::placeholders::error));
      }
      else
      {
        SensorClient::startDataRead();
      }
    }

    void FailoverClient::handleReadHeader(const boost::system::error_code& error)
    {
      try
      {
        SensorClient::handleReadHeader(error);
      }
      catch (InvalidHeaderError)
      {
        std::cout << "Failover to old M8 Data" << std::endl;
        // failover
        failover_ = true;

        buff_.resize(sizeof(M8DataPacket));

        // read the rest of the packet
        boost::asio::async_read(*read_socket_,
                                boost::asio::buffer(buff_.data() + sizeof(HeaderType),
                                                    sizeof(M8DataPacket) - sizeof(HeaderType)),
                                boost::bind(&FailoverClient::handleReadBody, this,
                                            boost::asio::placeholders::error));
      }
    }

  } // namespace client

} // namespace quanergy
