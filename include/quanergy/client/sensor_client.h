/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/


/** \file sensor_client.h
 *
 *  \brief Provide client for Quanergy sensor data.
 */

#ifndef QUANERGY_CLIENT_SENSOR_CLIENT_H
#define QUANERGY_CLIENT_SENSOR_CLIENT_H

#include <quanergy/client/packet_header.h>
#include <quanergy/client/tcp_client.h>

namespace quanergy
{
  namespace client
  {

    typedef TCPClient<PacketHeader> SensorClient;

  } // namespace client

} // namespace quanergy

#endif
