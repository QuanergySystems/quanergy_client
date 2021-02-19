/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/common/notifications.h>

using namespace quanergy;

boost::signals2::connection INotify::connect(const typename NotificationSignal::slot_type& subscriber)
{
    return notification_signal_.connect(subscriber);
}

// Try to find a NotificationType in the message, if one is not found, the input type is not changed
bool FindNotificationTag(const std::string& str, NotificationType& type)
{
	if (str.find("[trace]") != std::string::npos ||
		str.find("[Trace]") != std::string::npos ||
		str.find("[TRACE]") != std::string::npos)
	{
		type = NotificationType::Trace;
	}
	else if (str.find("[debug]") != std::string::npos ||
		str.find("[Debug]") != std::string::npos ||
		str.find("[DEBUG]") != std::string::npos)
	{
		type = NotificationType::Debug;
	}
	else if (str.find("[info]") != std::string::npos ||
		str.find("[Info]") != std::string::npos ||
		str.find("[INFO]") != std::string::npos)
	{
		type = NotificationType::Info;
	}
	else if (str.find("[warn]") != std::string::npos ||
		str.find("[warning]") != std::string::npos ||
		str.find("[Warn]") != std::string::npos ||
		str.find("[Warning]") != std::string::npos ||
		str.find("[WARN]") != std::string::npos ||
		str.find("[WARNING]") != std::string::npos)
	{
		type = NotificationType::Warning;
	}
	else if (str.find("[error]") != std::string::npos ||
		str.find("[Error]") != std::string::npos ||
		str.find("[ERROR]") != std::string::npos)
	{
		type = NotificationType::Error;
	}
	else if (str.find("[critical]") != std::string::npos ||
		str.find("[Critical]") != std::string::npos ||
		str.find("[CRITICAL]") != std::string::npos)
	{
		type = NotificationType::Critical;
	}
	else
	{
		return false;
	}
		
	return true;
}

int InfoBuf::sync()
{
	NotificationType type = NotificationType::Info;
	FindNotificationTag(this->str(), type);
    // do something with this->str() here
    notification_signal_(type, this->str());
    // (optionally clear buffer afterwards)
    this->str("");
    return 0;
}

int ErrorBuf::sync()
{
	NotificationType type = NotificationType::Error;
	FindNotificationTag(this->str(), type);
    // do something with this->str() here
    notification_signal_(type, this->str());
    // (optionally clear buffer afterwards)
    this->str("");
    return 0;
}

InfoBuf InfoStream::buf;
InfoStream::InfoStream() : std::ostream(&buf)
{}

boost::signals2::connection InfoStream::connect(const typename NotificationSignal::slot_type& subscriber)
{
    return buf.connect(subscriber);
}

ErrorBuf ErrorStream::buf;
ErrorStream::ErrorStream() : std::ostream(&buf)
{}

boost::signals2::connection ErrorStream::connect(const typename NotificationSignal::slot_type& subscriber)
{
    return buf.connect(subscriber);
}
