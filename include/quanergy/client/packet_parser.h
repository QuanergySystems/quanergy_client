/** \file packet_parser.h
  * \brief Provide base packet parser functionality.
  *
  * Individual message types will need to specialize the functionality provided here.
  */

#ifndef QUANERGY_PACKET_PARSER_GENERATOR_H
#define QUANERGY_PACKET_PARSER_GENERATOR_H

#include <memory>
#include <boost/signals2.hpp>

#include <quanergy/client/deserialize.h>

namespace quanergy
{
  template <class RESULT>
  struct PacketParserBase
  {
    /// signal type
    typedef boost::signals2::signal<void (RESULT const &)> Signal;

    virtual void setSignal(std::shared_ptr<Signal> const & signal)
    {
      signal_ = signal;
    }

  protected:
    /// Signal that gets fired whenever a result is ready.
    std::shared_ptr<Signal> signal_;
  };


  /** \brief Recursive case for the PacketParer variadic template. */

  template <class RESULT, class TYPE, class... TYPES>
  struct PacketParser : public PacketParserBase<RESULT>
  {
    // @TODO: Why is this necessary? It's in the base class.
    typedef boost::signals2::signal<void (const RESULT&)> Signal;

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
    static bool match(std::uint8_t type) = delete;
    inline void parse(std::uint8_t type, const std::vector<char>& packet) = delete;
  };
}
#endif
