/********************************************************************
 **                                                                **
 ** Copyright(C) 2014-2015 Quanergy Systems. All Rights Reserved.  **
 ** Contact: http://www.quanergy.com                               **
 **                                                                **
 ********************************************************************/

/** \file tcp_client.h
 *
 *  \brief Provide basic tcp_client to get packets
 */

#ifndef QUANERGY_CLIENT_TCP_CLIENT_H
#define QUANERGY_CLIENT_TCP_CLIENT_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <memory>
#include <atomic>

// networking
#include <boost/asio.hpp>
// signals for output
#include <boost/signals2.hpp>

// exception library
#include <quanergy/client/exceptions.h>

namespace quanergy
{
  namespace client
  {
    /** \brief TCPClient is a generic TCP data receiver that outputs packets based on header
     *  \tparam HEADER is the packet header type
     *  \attention The following two functions must be provided for HEADER type
     *             bool validateHeader(const HEADER&); // returns true if valid
     *             std::size_t getPacketSize(const HEADER&);  // returns the size of the full packet including header
     */
    template <class HEADER>
    class TCPClient
    {
    public:
      typedef std::shared_ptr<std::vector<char>> ResultType;
    
      /// give access to HEADER to derived classes
      typedef HEADER HeaderType;
      /// The packet is output on a signal
      typedef boost::signals2::signal<void (const ResultType&)> Signal;

      /** \brief Constructor taking a host, port, and queue size.
       */
      TCPClient(std::string const & host,
             std::string const & port,
             std::size_t max_queue_size = 100);

      // no default constructor
      TCPClient() = delete;

      // noncopyable
      TCPClient(const TCPClient&) = delete;
      TCPClient& operator=(const TCPClient&) = delete;

      virtual ~TCPClient();

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

      /** \brief Pulls packets off buffer queue and calls signal. */
      virtual void signalPackets();

      std::unique_ptr<boost::asio::ip::tcp::socket>       read_socket_;
      std::vector<char>                                   buff_;

    private:

      boost::asio::io_service                       io_service_;
      boost::asio::ip::tcp::resolver::query         host_query_;

      /// thread for running signals
      std::unique_ptr<std::thread> signal_thread_;

      std::queue<std::shared_ptr<std::vector<char>>> buff_queue_;
      std::size_t max_queue_size_;
      std::mutex                  buff_queue_mutex_;
      std::condition_variable     buff_queue_conditional_;
      std::atomic<bool>           kill_; // std::atomic_bool lacks proper constructors in MSVC

      Signal signal_;
    };

  } // namespace client

} // namespace quanergy

#include <quanergy/client/impl/tcp_client.hpp>

#endif
