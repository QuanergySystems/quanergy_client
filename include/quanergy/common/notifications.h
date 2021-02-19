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

// signals for output
#include <boost/signals2.hpp>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
    /** \brief severity level of the notification */
    enum class NotificationType { Trace, Debug, Info, Warning, Error, Critical };

    /// Notifications (error messages) are output on a different signal
    typedef boost::signals2::signal<void(const NotificationType&, const std::string&)> NotificationSignal;

    class DLLEXPORT INotify
    {
    public:
        /** \brief Connect a slot to the signal which will be for outputting messages and notifications */
        boost::signals2::connection connect(const typename NotificationSignal::slot_type& subscriber);

    protected:
        NotificationSignal notification_signal_;
    };

    class DLLEXPORT InfoBuf : public std::stringbuf, public INotify
    {
    public:
        InfoBuf() = default;
        virtual int sync() override;
    };

    class DLLEXPORT ErrorBuf : public std::stringbuf, public INotify
    {
    public:
        ErrorBuf() = default;
        virtual int sync() override;
    };

    class DLLEXPORT InfoStream : public std::ostream
    {
    public:
        InfoStream();
        
        /** \brief Connect a slot to the signal which will be for outputting messages and notifications */
        static boost::signals2::connection connect(const typename NotificationSignal::slot_type& subscriber);

    protected:
        static InfoBuf buf;
    };

    class DLLEXPORT ErrorStream : public std::ostream
    {
    public:
        ErrorStream();

        /** \brief Connect a slot to the signal which will be for outputting messages and notifications */
        static boost::signals2::connection connect(const typename NotificationSignal::slot_type& subscriber);

    protected:
        static ErrorBuf buf;
    };

    static InfoStream qout;
    static ErrorStream qerr;

} // namespace quanergy

#endif