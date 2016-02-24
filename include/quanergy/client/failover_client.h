/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/


/** \file failover_client.h
 *
 *  \brief Provide fallback support for old m8 sensor data format which didn't have a header.
 */

#ifndef QUANERGY_CLIENT_FAILOVER_CLIENT_H
#define QUANERGY_CLIENT_FAILOVER_CLIENT_H

#include <quanergy/client/sensor_client.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {

    class DLLEXPORT FailoverClient : public SensorClient
    {
    public:

      FailoverClient(std::string const & host,
                     std::string const & port,
                     std::size_t max_queue_size = 100);

      virtual ~FailoverClient() = default;

    protected:

      virtual void startDataRead();
      virtual void handleReadHeader(const boost::system::error_code& error);

      using SensorClient::read_socket_;
      using SensorClient::buff_;

    private:
      /// variable for automatic packet failover to old m8 packet parsing
      bool failover_ = false;
    };

  } // namespace client

} // namespace quanergy

#endif
