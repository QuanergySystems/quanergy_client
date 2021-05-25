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
    /** \brief constructor
      * \param name notifier name printed in output
      * \param level notifier level printed in output
      */
    NotifierBuf(std::string name, NotificationLevel level);

    /** \brief called on flush allowing us to forward to sink_ */
    virtual int sync() override;

    /** \brief set the downstream sink */
    void SetSink(std::ostream* sink) { sink_ = sink; }

    /** \brief enable formatting on output (time, level, name) */
    void EnableFormatting(bool enable = true) { add_formatting_ = enable; }

  protected:
    /** \brief sink to stream to on flush */
    std::ostream* sink_ = nullptr;

    /** \brief flag determining whether to include time, level, and name */
    bool add_formatting_ = true;

    /** \brief name used for formatting */
    std::string name_;

    /** \brief level string used for formatting */
    std::string level_string_;
  };

  /** \brief ostream for notifier */
  class DLLEXPORT NotifierStream : public std::ostream
  {
  public:
    /** \brief constructor
      * \param name notifier name printed in output
      * \param level notifier level printed in output
      */
    NotifierStream(std::string name, NotificationLevel level);

    /** \brief set the downstream sink */
    void SetSink(std::ostream* sink) { buf_.SetSink(sink); }

    /** \brief enable formatting on output (time, level, name) */
    void EnableFormatting(bool enable = true) { buf_.EnableFormatting(enable); }

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
    /** \brief constructor
      * \param name notifier name printed in output
      */
    Notifier(std::string name);

    /** \brief set the downstream sink for one or more streams */
    void SetSinks(std::ostream* sink, NotificationLevel minLevel = NotificationLevel::Trace, NotificationLevel maxLevel = NotificationLevel::Error);
    /** \brief set the downstream sink for one stream */
    void SetSink(std::ostream* sink, NotificationLevel level);

    /** \brief clear the downstream sink for one or more streams */
    void ClearSinks(NotificationLevel minLevel = NotificationLevel::Trace, NotificationLevel maxLevel = NotificationLevel::Error);
    /** \brief clear the downstream sink for one stream */
    void ClearSink(NotificationLevel level);

    /** \brief enable formatting on output (time, level, name) */
    void EnableFormatting(bool enable = true);

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

  static Notifier log {"QuanergyClient"};

} // namespace quanergy

#endif