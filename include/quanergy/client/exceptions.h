/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file exceptions.h
  * \brief Define specific sensor client run-time exceptions.
  *
  */

#ifndef QUANERGY_CLIENT_EXCEPTIONS_H
#define QUANERGY_CLIENT_EXCEPTIONS_H

#include <stdexcept>

namespace quanergy
{
  namespace client
  {

    /** \brief error binding to socket */
    struct SocketBindError : public std::runtime_error
    {
      explicit SocketBindError(const std::string& message)
        : std::runtime_error(message) {}
    };

    /** \brief error reading from socket */
    struct SocketReadError : public std::runtime_error
    {
      explicit SocketReadError(const std::string& message)
        : std::runtime_error(message) {}
    };

    /** \brief error parsing header */
    struct InvalidHeaderError : public std::exception
    {
      virtual const char* what() const throw() { return "Invalid header"; }
    };

    /** \brief packet size doesn't match data description */
    struct SizeMismatchError : public std::exception
    {
      virtual const char* what() const throw() { return "Packet sizes don't match"; }
    };

    /** \brief Invalid packet */
    struct InvalidPacketError : public std::exception
    {
      virtual const char* what() const throw() { return "Invalid packet"; }
    };

    /** \brief Invalid data type in header; no parser available */
    struct InvalidDataTypeError : public std::exception
    {
      virtual const char* what() const throw() { return "Invalid data type"; }
    };

    /** \brief Invalid version for type */
    struct InvalidDataVersionError : public std::exception
    {
      virtual const char* what() const throw() { return "Invalid data version"; }
    };

    /** \brief Firmware versions on sensor don't match */
    struct FirmwareVersionMismatchError : public std::exception
    {
      virtual const char* what() const throw() { return "Firmware version mismatch"; }
    };
    
    /** \brief Firmware watchdog is not receiving an internal signal */
    struct FirmwareWatchdogViolationError : public std::exception
    {
      virtual const char* what() const throw() { return "Firmware watchdog violation"; }
    };

    /** \brief Firmware unknown error
     * Raised for status bits that are not presently assigned.
     */
    struct FirmwareUnknownError : public std::exception
    {
      virtual const char* what() const throw() { return "Firmware unknown error"; }
    };

    /** \brief Degress per cloud must be 360 or less */
    struct InvalidDegreesPerCloud : public std::exception
    {
      virtual const char* what() const throw() { return "Invalid degrees per cloud"; }
    };

    /** \brief Return selction must be less than M8_NUM_RETURNS */
    struct InvalidReturnSelection : public std::exception
    {
      virtual const char* what() const throw() { return "Invalid return selection"; }
    };

    /** \brief Return ID coming from sensor doesn't match requested Return ID */
    struct ReturnIDMismatchError : public std::exception
    {
      virtual const char* what() const throw() { return "Return ID mismatch"; }
    };



  } // namespace client

} // namespace quanergy

#endif
