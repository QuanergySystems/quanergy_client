/****************************************************************************
 **
 ** Copyright(C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/
#ifndef QUANERGY_CLIENT_TCP_CLIENT_HPP
#define QUANERGY_CLIENT_TCP_CLIENT_HPP

#include <quanergy/client/tcp_client.h>

#include <iostream>

namespace quanergy
{
  namespace client
  {

    template <class HEADER>
    TCPClient<HEADER>::TCPClient(std::string const & host,
                   std::string const & port,
                   std::size_t max_queue_size)
      : buff_(sizeof(HEADER))
      , host_query_(host, port)
      , max_queue_size_(max_queue_size)
      , kill_(true)
    {
      read_socket_.reset(new boost::asio::ip::tcp::socket(io_service_));
    }

    template <class HEADER>
    TCPClient<HEADER>::~TCPClient()
    {
      stop();
    }

    template <class HEADER>
    boost::signals2::connection TCPClient<HEADER>::connect(const typename Signal::slot_type& subscriber)
    {
      return signal_.connect(subscriber);
    }

    template <class HEADER>
    void TCPClient<HEADER>::run()
    {
      if (!kill_)
        return;

      kill_ = false;
      io_service_.reset();
      startDataConnect();

      // create thread for parsing
      std::exception_ptr eptr;
      signal_thread_.reset(new std::thread([this, &eptr]
                                          {
                                            try
                                            {
                                              signalPackets();
                                            }
                                            catch (...)
                                            {
                                              eptr = std::current_exception();
                                              stop();
                                            }
                                          }));

      // Add this thread to the pool to handle data
      try
      {
        io_service_.run();
      }
      catch (...)
      {
        eptr = std::current_exception();
        stop();
      }

      signal_thread_->join();
      signal_thread_.reset();

      if (eptr) std::rethrow_exception(eptr);
    }

    template <class HEADER>
    void TCPClient<HEADER>::stop()
    {
      kill_ = true;
      // close socket before stopping service to cancel async operations
      read_socket_->close();
      // guarantee we recognize the closed socket before stopping
      io_service_.run_one();
      io_service_.stop();

      // notify that we are killing
      buff_queue_conditional_.notify_one();
    }

    template <class HEADER>
    void TCPClient<HEADER>::startDataConnect()
    {
      std::cout << "Attempting to connect (" << host_query_.host_name()
                << ":" << host_query_.service_name() << ")..." << std::endl;
      boost::asio::ip::tcp::resolver resolver(io_service_);

      try
      {
        auto endpoint_iterator = resolver.resolve(host_query_);

        boost::asio::async_connect(*read_socket_, endpoint_iterator,
                                   [this](boost::system::error_code error, boost::asio::ip::tcp::resolver::iterator)
                                   {
                                     if (kill_)
                                     {
                                       return;
                                     }
                                     else if (error)
                                     {
                                       std::cerr << "Unable to bind to socket (" << host_query_.host_name()
                                                 << ":" << host_query_.service_name() << ")! "
                                                 << error.message() << std::endl;
                                       throw SocketBindError(error.message());
                                     }
                                     else
                                     {
                                       std::cout << "Connection established" << std::endl;
                                       startDataRead();
                                     }
                                   });
      }
      catch (boost::system::system_error& e)
      {
        std::cerr << "Unable to resolve host (" << host_query_.host_name()
                  << ":" << host_query_.service_name() << ")! "
                  << e.what() << std::endl;
        throw SocketBindError(e.what());
      }
    }

    template <class HEADER>
    void TCPClient<HEADER>::startDataRead()
    {
      boost::asio::async_read(*read_socket_,
                              boost::asio::buffer(buff_.data(), sizeof(HEADER)),
                              boost::bind(&TCPClient<HEADER>::handleReadHeader, this,
                                          boost::asio::placeholders::error));
    }

    template <class HEADER>
    void TCPClient<HEADER>::handleReadHeader(const boost::system::error_code& error)
    {
      if (kill_)
      {
        return;
      }
      else if (error)
      {
        std::cerr << "Error reading header: "
                  << error.message() << std::endl;
        throw SocketReadError(error.message());
      }
      else
      {
        HEADER* h = reinterpret_cast<HEADER*>(buff_.data());

        // validate
        if (validateHeader(*h))
        {
          std::size_t size = getPacketSize(*h);
          buff_.resize(size); // invalidates h pointer because of potential reallocate and move

          boost::asio::async_read(*read_socket_,
                                  boost::asio::buffer(buff_.data() + sizeof(HEADER),
                                                      size - sizeof(HEADER)),
                                  boost::bind(&TCPClient<HEADER>::handleReadBody, this,
                                              boost::asio::placeholders::error));
        }
        else
        {
          throw InvalidHeaderError();
        }
      }
    }

    template <class HEADER>
    void TCPClient<HEADER>::handleReadBody(const boost::system::error_code& error)
    {
      if (kill_)
      {
        return;
      }
      else if (error)
      {
        std::cerr << "Error reading body: "
                  << error.message() << std::endl;
        throw SocketReadError(error.message());
      }
      else
      {
        std::unique_lock<std::mutex> lk(buff_queue_mutex_);

        // copy into shared_ptr
        buff_queue_.push(std::make_shared<std::vector<char>>(buff_));

        if (buff_queue_.size() > max_queue_size_)
        {
          buff_queue_.pop();
          lk.unlock();
          std::cout << "Warning: Client dropped packet due to full buffer" << std::endl;
        }
        else
        {
          lk.unlock();
        }

        buff_queue_conditional_.notify_one();
      }

      // get ready to read again
      startDataRead();
    }

    template <class HEADER>
    void TCPClient<HEADER>::signalPackets()
    {
      // define condition to continue: something in buffer or kill flag set
      auto continue_condition = [this]{return (!buff_queue_.empty() || kill_);};

      for (;;)
      {
        std::unique_lock<std::mutex> lk(buff_queue_mutex_);
        // if queue is empty, wait for something to be in the queue
        if (!continue_condition())
          buff_queue_conditional_.wait(lk, continue_condition);

        if (kill_)
          return;

        auto packet = buff_queue_.front();
        buff_queue_.pop();
        lk.unlock();

        signal_(packet);
      }
    }

  } // namespace client

} // namespace quanergy

#endif
