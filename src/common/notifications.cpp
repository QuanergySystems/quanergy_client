/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/common/notifications.h>

#include <chrono>
#include <iomanip>

using namespace quanergy;

static inline bool checkLevel(NotificationLevel level, NotificationLevel min, NotificationLevel max)
{
  return (level >= min && level <= max);
}

static std::string getTime()
{
  auto now = std::chrono::system_clock::now();
  auto itt = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&itt), "%Y-%m-%d %H:%M:%S");
  return ss.str();
}

NotifierBuf::NotifierBuf(std::string name, NotificationLevel level)
: name_(name)
{
  switch (level)
  {
    case NotificationLevel::Trace:
      level_string_ = "Trace";
      break;
    case NotificationLevel::Debug:
      level_string_ = "Debug";
      break;
    case NotificationLevel::Info:
      level_string_ = "Info";
      break;
    case NotificationLevel::Warn:
      level_string_ = "Warn";
      break;
    case NotificationLevel::Error:
      level_string_ = "Error";
      break;
    default:
      throw std::runtime_error("NotifierBuf invalid notification level");
  }
}

int NotifierBuf::sync()
{
  if (sink_ && !this->str().empty())
  {
    if (add_formatting_)
    {
      (*sink_) << getTime() << " [" << level_string_ << "] " << name_ << ": " << this->str();
    }
    else
    {
      (*sink_) << this->str();
    }
  }

  this->str("");
  return 0;
}

NotifierStream::NotifierStream(std::string name, NotificationLevel level)
: buf_(name, level)
{
  this->rdbuf(&buf_);
}

Notifier::Notifier(std::string name)
: error(name, NotificationLevel::Error)
, warn(name, NotificationLevel::Warn)
, info(name, NotificationLevel::Info)
, debug(name, NotificationLevel::Debug)
, trace(name, NotificationLevel::Trace)
{
  // default to Info & Warn on cout and Error on cerr
  SetSinks(&std::cout, NotificationLevel::Info, NotificationLevel::Warn);
  SetSink(&std::cerr, NotificationLevel::Error);
}

void Notifier::SetSinks(std::ostream* sink, NotificationLevel minLevel, NotificationLevel maxLevel)
{
  if (checkLevel(NotificationLevel::Trace, minLevel, maxLevel))
  {
    trace.SetSink(sink);
  }
  if (checkLevel(NotificationLevel::Debug, minLevel, maxLevel))
  {
    debug.SetSink(sink);
  }
  if (checkLevel(NotificationLevel::Info, minLevel, maxLevel))
  {
    info.SetSink(sink);
  }
  if (checkLevel(NotificationLevel::Warn, minLevel, maxLevel))
  {
    warn.SetSink(sink);
  }
  if (checkLevel(NotificationLevel::Error, minLevel, maxLevel))
  {
    error.SetSink(sink);
  }
}

void Notifier::SetSink(std::ostream* sink, NotificationLevel level)
{
  SetSinks(sink, level, level);
}

void Notifier::ClearSinks(NotificationLevel minLevel, NotificationLevel maxLevel)
{
  SetSinks(nullptr, minLevel, maxLevel);
}

void Notifier::ClearSink(NotificationLevel level)
{
  ClearSinks(level, level);
}

void Notifier::EnableFormatting(bool enable)
{
  trace.EnableFormatting(enable);
  debug.EnableFormatting(enable);
  info.EnableFormatting(enable);
  warn.EnableFormatting(enable);
  error.EnableFormatting(enable);
}