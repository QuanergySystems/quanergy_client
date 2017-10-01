/****************************************************************
**                                                            **
**  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
**  Contact: http://www.quanergy.com                          **
**                                                            **
**  Robert Kriener - Robert.Kriener@TMEIC.com          **
**  This class was extracted from packet_parser.h             **
**  It was updated to be compliant with VS2013                **
**                                                            **
****************************************************************/

/** \file variadic_packet_parser.h
* \brief provides generic parser which is capable of combining multiple parsers and iterating them.
*
*/

#pragma once

#include <quanergy/parsers/packet_parser.h>

/** \brief VariadicPacketParer takes a list of parsers and iterates through them. */
namespace quanergy
{
  namespace client
  {
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
        //Start iteration from end of tuple to start (required for VS2013 compatibility)
        return parse<sizeof...(PARSERS)-1>(packet, result);
      }

      /** \brief iterate through parsers to see if there is a match */
      inline virtual bool validate(const std::vector<char> &packet)
      {
        //Start iteration from end of tuple to start (required for VS2013 compatibility)
        return validate<sizeof...(PARSERS)-1>(packet);
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
      /// last attempt to find matching parser at I==0, if it fails throw an excp
      template<std::size_t I = 0>
      inline typename std::enable_if<I == 0, bool>::type parse(const std::vector<char>& packet, RESULT& result)
      {
        if (std::get<I>(parsers).validate(packet))
          return std::get<I>(parsers).parse(packet, result);
        else
          throw InvalidPacketError();
      }

      /// find parser and recurse if I != 0
      template<std::size_t I = 0>
      inline typename std::enable_if < I != 0, bool>::type parse(const std::vector<char>& packet, RESULT& result)
      {
        if (std::get<I>(parsers).validate(packet))
          return std::get<I>(parsers).parse(packet, result);
        else
          return parse<I - 1>(packet, result);
      }

      /// last attempt to find a match at I==0 if it fails return false
      template<std::size_t I = 0>
      inline typename std::enable_if<I == 0, bool>::type validate(const std::vector<char>& packet)
      {
        if (std::get<I>(parsers).validate(packet))
          return true;
        else
          return false;
      }

      /// find parser and recurse if I !=0
      template<std::size_t I = 0>
      inline typename std::enable_if < I != 0, bool>::type validate(const std::vector<char>& packet)
      {
        if (std::get<I>(parsers).validate(packet))
          return true;
        else
          return validate<I - 1>(packet);
      }

      std::tuple<PARSERS...> parsers;
    };
  };
}
