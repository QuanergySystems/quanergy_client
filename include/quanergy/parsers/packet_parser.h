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

#include <memory>

#include <boost/signals2.hpp>

#include <quanergy/client/exceptions.h>

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
          signal_(result);
      }

      protected:
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

    /** \brief VariadicPacketParer takes a list of parsers and iterates through them. */
    template <class RESULT, class... PARSERS>
    struct VariadicPacketParser : public PacketParserBase<RESULT>
    {
      typedef RESULT ResultType;

      VariadicPacketParser() = default;

      /** \brief provide access to the individual parsers */
      template <std::size_t I>
      auto get() -> decltype(std::get<I>(std::tuple<PARSERS...>()))&
      {
        return std::get<I>(parsers);
      }

      /** \brief iterate through parsers to find first match and parse */
      inline virtual bool validateParse(const std::vector<char>& packet, RESULT& result)
      {
        return parse<0>(packet, result);
      }

      /** \brief iterate through parsers to see if there is a match */
      inline virtual bool validate(const std::vector<char> &packet)
      {
        return validate<0>(packet);
      }

      /** \brief parse using validateParse but catch throw */
      inline virtual bool parse(const std::vector<char> &packet, RESULT &result)
      {
        try
        {
          return validateParse(packet, result);
        }
        catch (InvalidPacketError())
        {
          return false;
        }
      }

    private:
      /// didn't find matching parser if I >= number of parsers, end of recursion
      template<std::size_t I = 0>
      inline typename std::enable_if<I >= sizeof...(PARSERS), bool>::type parse(const std::vector<char>&, RESULT&)
      {
        throw InvalidPacketError();
      }

      /// find parser and recurse if I < number of parsers
      template<std::size_t I = 0>
      inline typename std::enable_if<I < sizeof...(PARSERS), bool>::type parse(const std::vector<char>& packet, RESULT& result)
      {
        if (std::get<I>(parsers).validate(packet))
          return std::get<I>(parsers).parse(packet, result);
        else
          return parse<I+1>(packet, result);
      }

      /// didn't find matching parser if I >= number of parsers, end of recursion
      template<std::size_t I = 0>
      inline typename std::enable_if<I >= sizeof...(PARSERS), bool>::type validate(const std::vector<char>&)
      {
        return false;
      }

      /// find parser and recurse if I < number of parsers
      template<std::size_t I = 0>
      inline typename std::enable_if<I < sizeof...(PARSERS), bool>::type validate(const std::vector<char>& packet)
      {
        if (std::get<I>(parsers).validate(packet))
          return true;
        else
          return validate<I+1>(packet);
      }

      std::tuple<PARSERS...> parsers;
    };

  } // namespace client

} // namespace quanergy
#endif
