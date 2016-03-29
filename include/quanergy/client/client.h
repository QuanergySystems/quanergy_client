/********************************************************************
 **                                                                **
 ** Copyright(C) 2014-2015 Quanergy Systems. All Rights Reserved.  **
 ** Contact: http://www.quanergy.com                               **
 **                                                                **
 ********************************************************************/

/** \file client.h
 *
 *  \brief Provide generalized (parameterized) sensor client service.
 */

#ifndef QUANERGY_CLIENT_CLIENT_H
#define QUANERGY_CLIENT_CLIENT_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <memory>

#ifdef _MSC_VER
  #include <atomic>
#endif

// networking
#include <boost/asio.hpp>
// signals for output
#include <boost/signals2.hpp>

// interface for parsing
#include <quanergy/client/packet_parser.h>

// exception library
#include <quanergy/client/exceptions.h>

// basic deserialization functions
#include <quanergy/client/deserialize.h>


namespace quanergy
{
  namespace client
  {
    /** \brief Client class connects to a sensor and uses template specialization to appropriately parse the data
     *
     *  \tparam RESULT specifies the expected return type (type for the signal)
     *  \tparam TYPES... the data packet types that we expect to receive and want to be able to handle
     */
    template <class RESULT, class... TYPES>
    class Client
    {
    public:
    
      typedef RESULT Result;

      typedef std::shared_ptr<Client<RESULT, TYPES...> > Ptr;

      typedef boost::signals2::signal<void (RESULT const &)> Signal;

      typedef PacketParser<RESULT, TYPES...> Parser;

      /** \brief Constructor taking a host, port, queue size, and a string that differentiates instances of the client.
       *  parsers are not required to use the frame_id but some may use it to differentiate instances of the client
       */
      Client(std::string const & host,
             std::string const & port,
             std::string const & frame_id = std::string(),
             std::size_t max_queue_size = 100);

      // no default constructor
      Client() = delete;

      // noncopyable
      Client(const Client&) = delete;
      Client& operator=(const Client&) = delete;

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
      std::size_t max_queue_size_;
      std::mutex                  buff_queue_mutex_;
      std::condition_variable     buff_queue_conditional_;
      std::atomic<bool>           kill_; // std::atomic_bool lacks proper constructors in MSVC

      std::shared_ptr<Signal>     signal_;
    };

  } // namespace client

} // namespace quanergy

#include <quanergy/client/impl/client.hpp>

#endif
