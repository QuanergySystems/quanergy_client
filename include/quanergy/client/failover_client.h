/****************************************************************************
 **
 ** Copyright(C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/

/** \file failover_client.h
 *
 *  \brief Provide fallback support for old m8 sensor data format.
 */

#ifndef FAILOVER_CLIENT_H
#define FAILOVER_CLIENT_H

#include <quanergy/client/pointcloud_types.h>

#include <quanergy/client/pointcloud_generator_failover.h>
#include <quanergy/client/client.h>

namespace quanergy
{
  namespace client
  {

    template <class... TYPES>
    class FailoverClient : public Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>
    {
    public:
      typedef std::shared_ptr<FailoverClient<TYPES...> > Ptr;

      /** \brief Constructor taking a host and port. */
      FailoverClient<TYPES...>(std::string const & host, std::string const & port, std::string const & frame_id);

      virtual ~FailoverClient();

    protected:

      virtual void startDataRead();
      virtual void handleReadHeader(const boost::system::error_code& error);

      /** \brief Converts packet to pointcloud and signals completion as needed. */
      virtual void parse(const std::vector<char>& packet);

    private:
      /// variable for automatic packet failover to old m8 packet parsing
      bool failover_;

      using Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>::read_socket_;
      using Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>::buff_;
      using Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>::parser_;
    };

  } // namespace client

} // namespace quanergy


#include <quanergy/client/impl/failover_client.hpp>

#endif
