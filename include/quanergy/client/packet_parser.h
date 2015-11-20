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

#ifndef QUANERGY_CLIENT_PACKET_PARSER_GENERATOR_H
#define QUANERGY_CLIENT_PACKET_PARSER_GENERATOR_H

#include <memory>
#include <string>

#include <boost/signals2.hpp>

namespace quanergy
{
  namespace client
  {
    /** \brief base class for packet parsers, providing the signal and frame id */
    template <class RESULT>
    struct PacketParserBase
    {
      PacketParserBase<RESULT>(std::string const & frame_id)
        : frame_id_(frame_id) {}

      /// signal type
      typedef boost::signals2::signal<void (RESULT const &)> Signal;

      virtual void setSignal(std::shared_ptr<Signal> const & signal)
      {
        signal_ = signal;
      }

    protected:
      /// Signal that gets fired whenever a result is ready.
      std::shared_ptr<Signal> signal_;

      /// Coordinate frame ID
      std::string frame_id_;
    };


    /** \brief Recursive case for the PacketParer variadic template. */
    template <class RESULT, class TYPE, class... TYPES>
    struct PacketParser : public PacketParserBase<RESULT>
    {
      using typename PacketParserBase<RESULT>::Signal;

      PacketParser<RESULT, TYPE, TYPES...>(std::string const & frame_id)
        : PacketParserBase<RESULT>(frame_id)
        , type_parser_(frame_id)
        , types_parser_(frame_id) {}

      virtual void setSignal(std::shared_ptr<Signal> const & signal)
      {
        type_parser_.setSignal(signal);
        types_parser_.setSignal(signal);
      }

      inline void parse(std::uint8_t type, const std::vector<char>& packet)
      {
        if (type_parser_.match(type))
          type_parser_.parse(type, packet);
        else
          types_parser_.parse(type, packet);
      }

    private:

      // Packet parser for this type.
      PacketParser<RESULT, TYPE> type_parser_;
      // The rest.
      PacketParser<RESULT, TYPES...> types_parser_;
    };


    /** \brief Force specialization of the single type case.  */
    template <class RESULT, class TYPE>
    struct PacketParser<RESULT, TYPE> : public PacketParserBase<RESULT>
    {
      PacketParser(std::string const &) = delete;
      static bool match(std::uint8_t type) = delete;
      inline void parse(std::uint8_t type, const std::vector<char>& packet) = delete;
    };

  } // namespace client

} // namespace quanergy
#endif
