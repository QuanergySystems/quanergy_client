/****************************************************************************
 **
 ** Copyright(C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/
#include <quanergy/client/client.h>

namespace quanergy
{
  template <class... Types>
  Client<Types...>::Client(const std::string& host, const std::string& port)
    : buff_(sizeof(PacketHeader))
    , host_query_(host, port)
    , kill_(false)
    , cloud_signal_(new CloudSignal)
  {
    read_socket_.reset(new boost::asio::ip::tcp::socket(io_service_));
    point_cloud_generator_.setCloudSignal(cloud_signal_);
  }

  template <class... Types>
  Client<Types...>::~Client()
  {
    stop();
  }

  template <class... Types>
  boost::signals2::connection Client<Types...>::connect(const typename CloudSignal::slot_type& subscriber)
  {
    return cloud_signal_->connect(subscriber);
  }

  template <class... Types>
  void Client<Types...>::run()
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

  template <class... Types>
  void Client<Types...>::stop()
  {
    kill_ = true;
    io_service_.stop();

    // notify that we are killing
    buff_queue_conditional_.notify_one();
  }

  template <class... Types>
  void Client<Types...>::startDataConnect()
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

  template <class... Types>
  void Client<Types...>::startDataRead()
  {
    boost::asio::async_read(*read_socket_,
              boost::asio::buffer(buff_.data(), sizeof(PacketHeader)),
              boost::bind(&Client::handleReadHeader, this,
                          boost::asio::placeholders::error));
  }

  template <class... Types>
  void Client<Types...>::handleReadHeader(const boost::system::error_code& error)
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

  template <class... Types>
  void Client<Types...>::handleReadBody(const boost::system::error_code& error)
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

  template <class... Types>
  void Client<Types...>::parsePackets()
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

      toPointCloud(*packet);
    }
  }

  template <class... Types>
  void Client<Types...>::toPointCloud(const std::vector<char>& packet)
  {
    const PacketHeader* h = reinterpret_cast<const PacketHeader*>(packet.data());
    point_cloud_generator_.toPointCloud(deserialize(h->packet_type), packet);
  }
}
