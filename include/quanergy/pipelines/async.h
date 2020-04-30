/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file async_module.h
 *
 *  \brief makes signal asynchronous so calling thread can return
 */

#ifndef QUANERGY_ASYNC_H
#define QUANERGY_ASYNC_H

#include <boost/signals2.hpp>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>

namespace quanergy
{
  namespace pipeline
  {
    /** \brief AsyncModule makes it easy to move downstream processing on another thread
      */
    template <class Type>
    struct AsyncModule
    {
      using ResultType = Type;

      using Signal = boost::signals2::signal<void (const ResultType&)>;

      /** \brief constructor
       *  \param max_queue_size determines the queue size. It should be >= 2 and 2 is
       *         a good value as long as the consumer is keeping up but depending on
       *         the situation, a bigger value may be needed.
       */
      AsyncModule(std::size_t max_queue_size = 2)
        : max_queue_size_(max_queue_size)
      {
        // spin up new thread to handle inputs
        signal_thread_.reset(new std::thread([this]
                                             {
                                               try
                                               {
                                                 processInputs();
                                               }
                                               catch (...)
                                               {
                                                 exception_ = std::current_exception();
                                               }
                                             }));
      }

      ~AsyncModule()
      {
        {
          std::lock_guard<std::mutex> lk(input_queue_mutex_);
          kill_ = true;
        }
        input_queue_conditional_.notify_one();
        if (signal_thread_ && signal_thread_->joinable())
        {
          signal_thread_->join();
        }
      }

      boost::signals2::connection connect(const typename Signal::slot_type& subscriber)
      {
        return signal_.connect(subscriber);
      }

      void slot(const Type& input)
      {
        // if an exception was caught, send it up the chain
        if (exception_)
          std::rethrow_exception(exception_);

        std::unique_lock<std::mutex> lk(input_queue_mutex_);

        input_queue_.push(input);

        // while shouldn't be necessary but doesn't hurt just to be sure
        while (input_queue_.size() > max_queue_size_)
        {
          std::cerr << "Warning: AsyncModule dropped input due to full buffer" << std::endl;
          input_queue_.pop();
        }

        lk.unlock();
        input_queue_conditional_.notify_one();
      }

      void processInputs()
      {
        for (;;)
        {
          std::unique_lock<std::mutex> lk(input_queue_mutex_);
          // wait for something in the queue
          if (input_queue_.empty())
            input_queue_conditional_.wait(lk, [this]{return (!input_queue_.empty() || kill_);});

          if (kill_)
            return;

          Type item = input_queue_.front();
          input_queue_.pop();
          lk.unlock();

          signal_(item);
        }
      }

    private:
      /// new thread for signal
      std::unique_ptr<std::thread> signal_thread_;
      std::exception_ptr exception_;

      std::queue<Type>            input_queue_;
      std::size_t                 max_queue_size_;
      std::mutex                  input_queue_mutex_;
      std::condition_variable     input_queue_conditional_;
      std::atomic_bool            kill_ {false};

      Signal signal_;
    };

  } // namespace client

} // namespace quanergy

#endif
