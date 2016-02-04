/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/


/** \file failover_client.h
 *
 *  \brief Provide fallback support for old m8 sensor data format.
 */

#ifndef QUANERGY_PARSERS_FAILOVER_CLIENT_H
#define QUANERGY_PARSERS_FAILOVER_CLIENT_H

#include <quanergy/client/client.h>

#include <quanergy/common/pointcloud_types.h>
#include <quanergy/parsers/pointcloud_generator_failover.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {

    template <class... TYPES>
	class DLLEXPORT FailoverClient : public Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>
    {
    public:
      typedef std::shared_ptr<FailoverClient<TYPES...> > Ptr;

      FailoverClient(std::string const & host,
                     std::string const & port,
                     std::string const & frame_id = std::string(),
                     std::size_t max_queue_size = 100);

      virtual ~FailoverClient() = default;

    protected:

      virtual void startDataRead();
      virtual void handleReadHeader(const boost::system::error_code& error);

      /** \brief Converts packet to pointcloud and signals completion as needed. */
      virtual void parse(const std::vector<char>& packet);

    private:
      /// variable for automatic packet failover to old m8 packet parsing
      bool failover_ = false;

      using Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>::read_socket_;
      using Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>::buff_;
      using Client<PointCloudHVDIRPtr, TYPES..., M8DataPacket>::parser_;
    };

  } // namespace client

} // namespace quanergy


#include <quanergy/parsers/impl/failover_client.hpp>

#endif
