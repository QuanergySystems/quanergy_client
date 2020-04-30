/********************************************************************
 **                                                                **
 ** Copyright(C) 2020 Quanergy Systems. All Rights Reserved.       **
 ** Contact: http://www.quanergy.com                               **
 **                                                                **
 ********************************************************************/

/** \file http_client.h
 *
 *  \brief Provide basic http_client to get a response
 */

#ifndef QUANERGY_CLIENT_HTTP_CLIENT_H
#define QUANERGY_CLIENT_HTTP_CLIENT_H

// networking
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

// output to ostream
#include <ostream>

// exception library
#include <quanergy/client/exceptions.h>

namespace quanergy
{
  namespace client
  {
    /** \brief HTTPClient is a generic HTTP response receiver that outputs to an ostream
     */
    class HTTPClient
    {
    public:
      /** \brief Constructor taking a host and port.
       */
      HTTPClient(std::string const & host,
                 std::string const & port = "7780");

      // no default constructor
      HTTPClient() = delete;

      // noncopyable
      HTTPClient(const HTTPClient&) = delete;
      HTTPClient& operator=(const HTTPClient&) = delete;

      virtual ~HTTPClient() = default;

      void read(std::string target, std::ostream& response_body);

    private:

      boost::asio::io_service                       io_service_;
      boost::asio::ip::tcp::resolver::query         host_query_;
    };

  } // namespace client

} // namespace quanergy

#endif
