/****************************************************************************
 **
 ** Copyright(C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/
#include <quanergy/client/client.h>

namespace quanergy
{
  namespace client
  {

    template <class RESULT, class... TYPES>
    Client<RESULT, TYPES...>::Client(std::string const & host, 
                                     std::string const & port,
                                     std::string const & frame_id)
      : buff_(sizeof(PacketHeader))
      , parser_(frame_id)
      , host_query_(host, port)
      , kill_(false)
      , signal_(new Signal)
    {
      read_socket_.reset(new boost::asio::ip::tcp::socket(io_service_));
      parser_.setSignal(signal_);
    }


    template <class RESULT, class... TYPES>
    Client<RESULT, TYPES...>::~Client()
    {
      stop();
    }


    template <class RESULT, class... TYPES>
    boost::signals2::connection Client<RESULT, TYPES...>::connect(const typename Signal::slot_type& subscriber)
    {
      return signal_->connect(subscriber);
    }


    template <class RESULT, class... TYPES>
    void Client<RESULT, TYPES...>::run()
    {
      kill_ = false;
      startDataConnect();

      // create thread for parsing
      std::exception_ptr eptr;
      parse_thread_.reset(new std::thread([this, &eptr]
                                          {
                                            try
                                            {
                                              parsePackets();
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

      parse_thread_->join();
      parse_thread_.reset();

      if (eptr) std::rethrow_exception(eptr);
    }

    template <class RESULT, class... TYPES>
    void Client<RESULT, TYPES...>::stop()
    {
      kill_ = true;
      io_service_.stop();

      // notify that we are killing
      buff_queue_conditional_.notify_one();
    }

    template <class RESULT, class... TYPES>
    void Client<RESULT, TYPES...>::startDataConnect()
    {
      std::cout << "Attempting to connect..." << std::endl;
      boost::asio::ip::tcp::resolver resolver(io_service_);

      auto endpoint_iterator = resolver.resolve(host_query_);

      boost::asio::async_connect(*read_socket_, endpoint_iterator,
                                 [this](boost::system::error_code error, boost::asio::ip::tcp::resolver::iterator)
                                 {
                                   if (error)
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


    template <class RESULT, class... TYPES>
    void Client<RESULT, TYPES...>::startDataRead()
    {
      boost::asio::async_read(*read_socket_,
                              boost::asio::buffer(buff_.data(), sizeof(PacketHeader)),
                              boost::bind(&Client::handleReadHeader, this,
                                          boost::asio::placeholders::error));
    }


    template <class RESULT, class... TYPES>
    void Client<RESULT, TYPES...>::handleReadHeader(const boost::system::error_code& error)
    {
      if (error)
      {
        std::cerr << "Error reading header: "
                  << error.message() << std::endl;
        throw SocketReadError(error.message());
      }
      else
      {
        PacketHeader* h =
          reinterpret_cast<PacketHeader*>(buff_.data());

        // check signature
        if (deserialize(h->signature) != SIGNATURE)
        {
          std::cerr << "Invalid header signature: " << std::hex << std::showbase
                    << h->signature << std::dec << std::noshowbase << std::endl;
          throw InvalidHeaderError();
        }
        else
        {
          auto size = deserialize(h->size);
          buff_.resize(size); // invalidates h pointer because of potential reallocate and move

          boost::asio::async_read(*read_socket_,
                                  boost::asio::buffer(buff_.data() + sizeof(PacketHeader),
                                                      size - sizeof(PacketHeader)),
                                  boost::bind(&Client::handleReadBody, this,
                                              boost::asio::placeholders::error));
        }
      }
    }


    template <class RESULT, class... TYPES>
    void Client<RESULT, TYPES...>::handleReadBody(const boost::system::error_code& error)
    {
      if (error)
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

        lk.unlock();
        buff_queue_conditional_.notify_one();
      }

      // get ready to read again
      startDataRead();
    }


    template <class RESULT, class... TYPES>
    void Client<RESULT, TYPES...>::parsePackets()
    {
      for (;;)
      {
        std::unique_lock<std::mutex> lk(buff_queue_mutex_);
        // wait for something in the queue
        buff_queue_conditional_.wait(lk, [this]{return (!buff_queue_.empty() || kill_);});

        if (kill_)
          return;

        auto packet = buff_queue_.front();
        buff_queue_.pop();
        lk.unlock();

        parse(*packet);
      }
    }


    template <class RESULT, class... TYPES>
    void Client<RESULT, TYPES...>::parse(const std::vector<char>& packet)
    {
      const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());
      parser_.parse(deserialize(h->packet_type), packet);
    }

  } // namespace client

} // namespace quanergy
