/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/client/http_client.h>

#include <iostream>

namespace quanergy
{
  namespace client
  {
    HTTPClient::HTTPClient(std::string const & host,
               std::string const & port)
      : host_query_(host, port)
    {
    }

    void HTTPClient::read(std::string target, std::ostream& response_body)
    {
      // Get a list of endpoints corresponding to the server name.
      boost::asio::ip::tcp::resolver resolver(io_service_);
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(host_query_);

      // Try each endpoint until we successfully establish a connection.
      boost::asio::ip::tcp::socket socket(io_service_);
      boost::asio::connect(socket, endpoint_iterator);

      // Form the request. We specify the "Connection: close" header so that the
      // server will close the socket after transmitting the response. This will
      // allow us to treat all data up until the EOF as the content.
      boost::asio::streambuf request;
      std::ostream request_stream(&request);
      request_stream << "GET " << target << " HTTP/1.0\r\n";
      request_stream << "Host: " << host_query_.host_name() << "\r\n";
      request_stream << "Accept: */*\r\n";
      request_stream << "Connection: close\r\n\r\n";

      // Send the request.
      boost::asio::write(socket, request);

      // Read the response status line. The response streambuf will automatically
      // grow to accommodate the entire line. The growth may be limited by passing
      // a maximum size to the streambuf constructor.
      boost::asio::streambuf response_buf;
      boost::asio::read_until(socket, response_buf, "\r\n");

      // Check that response is OK.
      std::istream response_stream(&response_buf);
      std::string http_version;
      response_stream >> http_version;

      unsigned int status_code;
      response_stream >> status_code;

      std::string status_message;
      std::getline(response_stream, status_message);

      if (!response_stream || http_version.substr(0, 5) != "HTTP/")
      {
        throw InvalidHTTPResponse();
      }

      // Status code 200 = OK
      if (status_code != 200)
      {
        throw HTTPResponseError(std::string("HTTP response returned error code: ") + std::to_string(status_code));
      }

      // Read the response headers, which are terminated by a blank line.
      boost::asio::read_until(socket, response_buf, "\r\n\r\n");
      std::string header;
      while (std::getline(response_stream, header) && header != "\r");

      // Write whatever content we already have to output.
      if (response_buf.size() > 0)
        response_body << &response_buf;

      // Read until EOF.
      boost::system::error_code error;
      while (boost::asio::read(socket, response_buf,
            boost::asio::transfer_at_least(1), error))
        response_body << &response_buf;
      if (error != boost::asio::error::eof)
        throw boost::system::system_error(error);
    }

  } // namespace client

} // namespace quanergy
