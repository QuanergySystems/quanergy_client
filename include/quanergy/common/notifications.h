/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

 /** \file notifications.h
   * \brief Define classes for handling notifications.
   *
   */

#ifndef QUANERGY_NOTIFICATIONS_H
#define QUANERGY_NOTIFICATIONS_H

#include <iostream>
#include <sstream>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  /** \brief severity level of the notification */
  enum class NotificationLevel { Trace, Debug, Info, Warn, Error };

  /** \brief string buffer for notifier */
  class DLLEXPORT NotifierBuf : public std::stringbuf
  {
  public:
    NotifierBuf() = default;

    /** \brief called on flush allowing us to forward to sink_ */
    virtual int sync() override;

    /** \brief set the downstream sink */
    void SetSink(std::ostream* sink) {sink_ = sink;}

  protected:
    /** \brief sink to stream to on flush */
    std::ostream* sink_ = nullptr;
  };

  /** \brief ostream for notifier */
  class DLLEXPORT NotifierStream : public std::ostream
  {
  public:
    NotifierStream() : std::ostream(&buf_) {}

    /** \brief set the downstream sink */
    void SetSink(std::ostream* sink) {buf_.SetSink(sink);}

  protected:
    /** \brief the notification buffer for this stream */
    NotifierBuf buf_;
  };

  /** \brief class to keep the various streams 
   *  by default, error streams to cerr and info and warn stream to cout
  */
  class DLLEXPORT Notifier
  {
  public:
    Notifier();

    /** \brief set the downstream sink for one or more streams */
    void SetSinks(std::ostream* sink, NotificationLevel minLevel = NotificationLevel::Trace, NotificationLevel maxLevel = NotificationLevel::Error);
    /** \brief set the downstream sink for one stream */
    void SetSink(std::ostream* sink, NotificationLevel level);

    /** \brief clear the downstream sink for one or more streams */
    void ClearSinks(NotificationLevel minLevel = NotificationLevel::Trace, NotificationLevel maxLevel = NotificationLevel::Error);
    /** \brief clear the downstream sink for one stream */
    void ClearSink(NotificationLevel level);

    /** \brief the error stream */
    NotifierStream error;
    /** \brief the warn stream */
    NotifierStream warn;
    /** \brief the info stream */
    NotifierStream info;
    /** \brief the debug stream */
    NotifierStream debug;
    /** \brief the info stream */
    NotifierStream trace;
  };

  static Notifier log;

} // namespace quanergy

#endif