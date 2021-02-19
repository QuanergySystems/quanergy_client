/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file packet_parser.h
 * \brief Provide base packet parser functionality.
 *
 * Individual message types will need to specialize the functionality provided here.
 */

#ifndef QUANERGY_CLIENT_PACKET_PARSER_H
#define QUANERGY_CLIENT_PACKET_PARSER_H

#pragma warning ( disable : 4275 )

#include <memory>
#include <chrono>

#include <boost/signals2.hpp>

#include <quanergy/client/exceptions.h>

namespace
{
	using namespace std::chrono_literals;
}

namespace quanergy
{
  namespace client
  {
    template <class PARSER>
    struct PacketParserModule : public PARSER
    {
      PacketParserModule() = default;

      /// signal type
      typedef boost::signals2::signal<void (const typename PARSER::ResultType&)> Signal;
      /** \brief Connect a slot to the signal which will be emitted when a new RESULT is available */
      boost::signals2::connection connect(const typename Signal::slot_type& subscriber)
      {
        return signal_.connect(subscriber);
      }

	  void slot(const std::shared_ptr<std::vector<char>>& packet)
	  {
	    // don't do the work unless someone is listening
	    if (signal_.num_slots() == 0)
		    return;

	    if (PARSER::validateParse(*packet, result))
	    {
            last_data_time_ = std::chrono::steady_clock::now();
		    signal_(result);
	    }
	    else
	    {
		    //Starting from this point the driver will have parse_timeout_sec_ to get a new full cloud without failing out
		    if (last_data_time_ == min_time_)
		    {
                last_data_time_ = std::chrono::steady_clock::now();
		    }
		    else if ((std::chrono::steady_clock::now() - last_data_time_) > parse_timeout_sec_)
		    {
			    //Reset clock for next try
                last_data_time_ = min_time_;
			    throw ParseTimeoutError();
		    }
	    }
	  }

      protected:
		static constexpr auto min_time_ = std::chrono::time_point<std::chrono::steady_clock>::min();
		static constexpr auto parse_timeout_sec_ = 500s;
		std::chrono::time_point<std::chrono::steady_clock> last_data_time_ = min_time_;

        /// Signal that gets fired whenever a result is ready.
        Signal signal_;

        /// result to pass to parse function
        typename PARSER::ResultType result;
    };

    /** \brief base class for packet parsers */
    template <class RESULT>
    struct PacketParserBase
    {
      PacketParserBase() = default;

      /** \brief check packet validity and parse if a match
       *  \return true if result updated; false otherwise
       *  \throws InvalidPacketError if not a valid packet
       */
      inline virtual bool validateParse(const std::vector<char>& packet, RESULT& result)
      {
        if (validate(packet))
          return parse(packet, result);
        else
          throw InvalidPacketError();
      }

      /** \brief check packet validity
       *  \return true if valid, false otherwise
       */
      virtual bool validate(const std::vector<char>& packet) = 0;
      /** \brief parse packet and update result
       *  \return true if result updated; false otherwise
       *          (some parsers may require multiple packets before updating result)
       */
      virtual bool parse(const std::vector<char>& packet, RESULT& result) = 0;
    };

  } // namespace client

} // namespace quanergy
#endif
