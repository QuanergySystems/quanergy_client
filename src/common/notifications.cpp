/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/common/notifications.h>

using namespace quanergy;

static inline bool checkLevel(NotificationLevel level, NotificationLevel min, NotificationLevel max)
{
  return (level >= min && level <= max);
}

int NotifierBuf::sync()
{
	// NotificationType type = NotificationType::Info;
  // notification_signal_(type, this->str());
  if (sink_ && !this->str().empty())
    (*sink_) << this->str();
    
  this->str("");
  return 0;
}

Notifier::Notifier()
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