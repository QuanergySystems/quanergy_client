/****************************************************************************
 **
 ** Copyright(C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/
#ifndef FAILOVER_CLIENT_H
#define FAILOVER_CLIENT_H

#include "deserialize_failover.h"
#include "pointcloud_generator_failover.h"
#include "client.h"

namespace quanergy
{
  template <class... Types>
  class FailoverClient : public Client<Types..., M8DataPacket>
  {
    public:
      /** \brief Constructor taking a host and port. */
      FailoverClient(const std::string& host, const std::string& port);

      virtual ~FailoverClient();

    protected:
      virtual void startDataRead();
      virtual void handleReadHeader(const boost::system::error_code& error);

      /** \brief converts packet to pointcloud and signals completion as needed */
      virtual void toPointCloud(const std::vector<char>& packet);

    private:
      /// variable for automatic packet failover to old m8 packet parsing
      bool failover;

      using Client<Types..., M8DataPacket>::read_socket_;
      using Client<Types..., M8DataPacket>::buff_;
      using Client<Types..., M8DataPacket>::point_cloud_generator_;
  };
}

#include "impl/failover_client.hpp"

#endif
