/****************************************************************************
 **
 ** Copyright(C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/

/** \file client.h
 *
 *  \brief Provide generalized (parameterized) sensor client service.
 */

#ifndef QUANERGY_CLIENT_H
#define QUANERGY_CLIENT_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <memory>

#include <boost/asio.hpp>
#include <boost/signals2.hpp>

#include <quanergy/client/packet_parser.h>

#include <quanergy/client/exceptions.h>
#include <quanergy/client/deserialize.h>


namespace quanergy
{
  template <class RESULT, class... TYPES>
  class Client
  {
  public:
    
    typedef RESULT Result;

    typedef std::shared_ptr<Client<RESULT, TYPES...> > Ptr;

    typedef boost::signals2::signal<void (RESULT const &)> Signal;

    typedef PacketParser<RESULT, TYPES...> Parser;

    /** \brief Constructor taking a host and port. */
    Client(std::string const & host, std::string const & port);

    virtual ~Client();

    /** \brief Connect a slot to the signal which will be emitted when a new RESULT is available */
    boost::signals2::connection connect(const typename Signal::slot_type& subscriber);

    /** \brief Starts processing the Quanergy packets */
    virtual void run();

    /** \brief Stops processing the Quanergy packets */
    virtual void stop();

  protected:

    /** \brief Asynchronously wait for connection. */
    virtual void startDataConnect();

    /** \brief Asynchronously read from socket. */
    virtual void startDataRead();

    /** \brief Handle read of packet header. */
    virtual void handleReadHeader(const boost::system::error_code& error);

    /** \brief Handle read of packet body. */
    virtual void handleReadBody(const boost::system::error_code& error);

    /** \brief Pulls packets off buffer queue and calls parse. */
    virtual void parsePackets();

    /** \brief Converts packet to RESULT and signals completion as needed. */
    virtual void parse(const std::vector<char>& packet);

    std::unique_ptr<boost::asio::ip::tcp::socket>       read_socket_;
    std::vector<char>                                   buff_;
    Parser                                              parser_;

  private:

    boost::asio::io_service                       io_service_;
    boost::asio::ip::tcp::resolver::query         host_query_;

    /// thread for running parsing
    std::unique_ptr<std::thread>              parse_thread_;

    std::queue<std::shared_ptr<std::vector<char>>> buff_queue_;
    std::mutex                  buff_queue_mutex_;
    std::condition_variable     buff_queue_conditional_;
    std::atomic_bool            kill_;

    std::shared_ptr<Signal>     signal_;
  };
}

#include <quanergy/client/impl/client.hpp>

#endif
