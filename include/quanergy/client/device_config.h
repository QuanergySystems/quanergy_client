/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_SENSOR_PIPELINE_CONFIG_H
#define QUANERGY_CLIENT_SENSOR_PIPELINE_CONFIG_H

#include <memory>
#include <chrono>

// networking
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>

//#include <boost/optional.hpp>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace client
  {
    namespace SettingsAPI
    {
      enum class Command { Invalid, Set, Reset, Reboot, Query, Status, DeviceInfo};

      /** \brief File is the base class for XML requests and responses to/from the sensor. It is best to use the derived classes externally.
       */
      class DLLEXPORT File : protected boost::property_tree::ptree
      {
        using boost::property_tree::ptree::ptree;

      public:
        typedef std::unique_ptr<File> UPtr;
        typedef std::unique_ptr<const File> ConstUPtr;
        typedef std::shared_ptr<File> Ptr;
        typedef std::shared_ptr<const File> ConstPtr;

        File();
        File(Command command);

        virtual ~File() = default;

        /// \brief get the integer status code of the entire file (response only) if one exists
        int status() const { return status_; }

        /// \brief get a descriptive string corresponding to the current status
        std::string status_str() const;

        /// \brief get the type of the file (command or response type) as a Command enum
        Command type() const { return command_; }

        /* \brief clear the file and read new contents in from a stream, then attempt to parse the contents
         * \param stream is the input stream to read from (XML)
         * \return true if successfully read in and parsed the contents, false o.w.
         */
        bool from_stream(std::istream& stream);

        /* \brief output the current contents of this file into an output stream
         * \param stream a reference to the stream to output the current contents into
         */
        void to_stream(std::ostream& stream) const;

      protected:
        /* \brief attempt to parse the header of the file (responses) to validate the contents
         * \return true if successfully parsed the header, false o.w.
         */
        virtual bool parse_header();

        /* \brief attempt to get the value at the requested path in the ptree contents
         * \param path is the path of the ptree (full path) to attempt to obtain the value from
         * \param value is a reference to a variable which will be written into only if value is successfully retrieved
         * \return true if value was read from the ptree, false o.w.
         */
        template <typename T>
        bool try_get(std::string path, T& value) const;

        /* \brief attempt to get the value at the requested path in the ptree contents
         * \param path is the path of the ptree (full path) to attempt to obtain the value from
         * \return a boost optional which will be filled out if the value was read successfully, empty o.w.
         */
        template <typename T>
        boost::optional<T> try_get(std::string path) const;

        int status_ = 0;
        Command command_ = Command::Invalid;
        std::chrono::time_point<
          std::chrono::steady_clock>  message_timestamp_;
      };

      /** \brief SetRequest is the means by which to set device parameters. Call it's functions with appropriate values to fill out the underlying file
       */
      class DLLEXPORT SetRequest : public File
      {
      public:
        typedef std::shared_ptr<SetRequest> Ptr;
        typedef std::shared_ptr<const SetRequest> ConstPtr;
        typedef std::unique_ptr<SetRequest> UPtr;
        typedef std::unique_ptr<const SetRequest> ConstUPtr;

        SetRequest();

        SetRequest& set_range(int index, int value);
        SetRequest& set_intensity(int index, int value);
        SetRequest& set_range(std::vector<int> values);
        SetRequest& set_intensity(std::vector<int> values);
        SetRequest& set_invertPPS(int value);
        SetRequest& set_nmeaBaud(int value);
        SetRequest& set_invertSerial(int value);
        SetRequest& set_returnSelect(int value);
        SetRequest& set_led(bool value);
        SetRequest& set_blockedCapDetect(bool value);
        SetRequest& set_disconnectSpeed(std::string value);
        SetRequest& set_lowPowerSpinup(bool value);
        SetRequest& set_frameRate(int value);
        SetRequest& set_fovCenter(float value);
        SetRequest& set_fovWidth(float value);
        SetRequest& set_networkType(std::string value);
        SetRequest& set_ipAddress(std::string value);
        SetRequest& set_gateway(std::string value);
        SetRequest& set_netmask(std::string value);
        SetRequest& set_broadcast(std::string value);
        SetRequest& set_ntpOption(bool value);
        SetRequest& set_ntp_tp(std::string value);
        SetRequest& set_ntp_ip_addr(std::string value);
      };

      /** \brief SetOrQueryResponse is the response to a set or query command, it's underlying file will be read to obtain the requested values
       */
      class DLLEXPORT SetOrQueryResponse : public File
      {
      public:
        typedef std::shared_ptr<SetOrQueryResponse> Ptr;
        typedef std::shared_ptr<const SetOrQueryResponse> ConstPtr;
        typedef std::unique_ptr<SetOrQueryResponse> UPtr;
        typedef std::unique_ptr<const SetOrQueryResponse> ConstUPtr;

        SetOrQueryResponse();

        bool get_range(int index, int& value) const;
        bool get_intensity(int index, int& value) const;
        bool get_range(std::vector<int>& values) const;
        bool get_intensity(std::vector<int>& values) const;
        bool get_invertPPS(int& value) const;
        bool get_nmeaBaud(int& value) const;
        bool get_invertSerial(int& value) const;
        bool get_returnSelect(int& value) const;
        bool get_led(bool& value) const;
        bool get_blockedCapDetect(bool& value) const;
        bool get_disconnectSpeed(std::string& value) const;
        bool get_lowPowerSpinup(bool& value) const;
        bool get_frameRate(int& value) const;
        bool get_fovCenter(float& value) const;
        bool get_fovWidth(float& value) const;
        bool get_networkType(std::string& value) const;
        bool get_ipAddress(std::string& value) const;
        bool get_gateway(std::string& value) const;
        bool get_netmask(std::string& value) const;
        bool get_broadcast(std::string& value) const;
        bool get_ntpOption(bool& value) const;
        bool get_ntp_tp(std::string& value) const;
        bool get_ntp_ip_addr(std::string& value) const;

        boost::optional<int> get_range(int index) const;
        boost::optional<int> get_intensity(int index) const;
        boost::optional<std::vector<int>> get_range() const;
        boost::optional<std::vector<int>> get_intensity() const;
        boost::optional<int> get_invertPPS() const;
        boost::optional<int> get_nmeaBaud() const;
        boost::optional<int> get_invertSerial() const;
        boost::optional<int> get_returnSelect() const;
        boost::optional<bool> get_led() const;
        boost::optional<bool> get_blockedCapDetect() const;
        boost::optional<std::string> get_disconnectSpeed() const;
        boost::optional<bool> get_lowPowerSpinup() const;
        boost::optional<int> get_frameRate() const;
        boost::optional<float> get_fovCenter() const;
        boost::optional<float> get_fovWidth() const;
        boost::optional<std::string> get_networkType() const;
        boost::optional<std::string> get_ipAddress() const;
        boost::optional<std::string> get_gateway() const;
        boost::optional<std::string> get_netmask() const;
        boost::optional<std::string> get_broadcast() const;
        boost::optional<bool> get_ntpOption() const;
        boost::optional<std::string> get_ntp_tp() const;
        boost::optional<std::string> get_ntp_ip_addr() const;

        bool copy_from(const SetOrQueryResponse& response);

      private:
        const int num_beams_ = 8;
      };

      /** \brief DeviceInfoResponse is the response to a device into request, it's underlying file will be read to obtain the requested values
       */
      class DLLEXPORT DeviceInfoResponse : public File
      {
      public:
        typedef std::shared_ptr<DeviceInfoResponse> Ptr;
        typedef std::shared_ptr<const DeviceInfoResponse> ConstPtr;
        typedef std::unique_ptr<DeviceInfoResponse> UPtr;
        typedef std::unique_ptr<const DeviceInfoResponse> ConstUPtr;

        DeviceInfoResponse();

        bool get_revision(std::string& value) const;
        bool get_top_version(std::string& value) const;
        bool get_base_release(std::string& value) const;
        bool get_model(std::string& value) const;
        bool get_revision_number(std::string& value) const;
        bool get_linux_OS(std::string& value) const;
        bool get_ps_code(std::string& value) const;
        bool get_linux_driver(std::string& value) const;
        bool get_bist(std::string& value) const;
        bool get_scripts(std::string& value) const;
        bool get_webserver(std::string& value) const;
        bool get_pl(std::string& value) const;
        bool get_mac_address(std::string& value) const;
        bool get_serial_number(std::string& value) const;

        bool get_calibration_version(double& value) const;
        bool get_encoder_amplitude(double& value) const;
        bool get_encoder_phase(double& value) const;
        bool get_number_beams(int& value) const;
        bool get_vertical_angles(std::vector<double>& value) const;

        boost::optional<std::string> get_revision() const;
        boost::optional<std::string> get_top_version() const;
        boost::optional<std::string> get_base_release() const;
        boost::optional<std::string> get_model() const;
        boost::optional<std::string> get_revision_number() const;
        boost::optional<std::string> get_linux_OS() const;
        boost::optional<std::string> get_ps_code() const;
        boost::optional<std::string> get_linux_driver() const;
        boost::optional<std::string> get_bist() const;
        boost::optional<std::string> get_scripts() const;
        boost::optional<std::string> get_webserver() const;
        boost::optional<std::string> get_pl() const;
        boost::optional<std::string> get_mac_address() const;
        boost::optional<std::string> get_serial_number() const;

        boost::optional<double> get_calibration_version() const;
        boost::optional<double> get_encoder_amplitude() const;
        boost::optional<double> get_encoder_phase() const;
        boost::optional<int> get_number_beams() const;
        boost::optional<std::vector<double>> get_vertical_angles() const;
      };

      /** \brief DeviceStatusResponse is the response to a status request, it's underlying file will be read to obtain the requested values
       */
      class DLLEXPORT DeviceStatusResponse : public File
      {
      public:
        typedef std::shared_ptr<DeviceStatusResponse> Ptr;
        typedef std::shared_ptr<const DeviceStatusResponse> ConstPtr;
        typedef std::unique_ptr<DeviceStatusResponse> UPtr;
        typedef std::unique_ptr<const DeviceStatusResponse> ConstUPtr;

        DeviceStatusResponse();

        bool get_gps_status(std::string& value) const;
        bool get_frameRate_status(double& value) const;
        bool get_sensor_temp_status(double& value) const;
        bool get_watchdog_status(int& value) const;
        bool get_motor_init_failure_status(int& value) const;
        bool get_motor_velocity_failure_status(int& value) const;
        bool get_blocked_cap_status(std::string& value) const;
        bool get_version_mismatch_error_status(int& value) const;
        bool get_laser_overtemp_error_status(int& value) const;
        bool get_cpu_overtemp_error_status(int& value) const;
        bool get_seconds_status(std::uint64_t& value) const;
        bool get_nanoseconds_status(std::uint64_t& value) const;

        boost::optional<std::string> get_gps_status() const;
        boost::optional<double> get_frameRate_status() const;
        boost::optional<double> get_sensor_temp_status() const;
        boost::optional<int> get_watchdog_status() const;
        boost::optional<int> get_motor_init_failure_status() const;
        boost::optional<int> get_motor_velocity_failure_status() const;
        boost::optional<std::string> get_blocked_cap_status() const;
        boost::optional<int> get_version_mismatch_error_status() const;
        boost::optional<int> get_laser_overtemp_error_status() const;
        boost::optional<int> get_cpu_overtemp_error_status() const;
        boost::optional<std::uint64_t> get_seconds_status() const;
        boost::optional<std::uint64_t> get_nanoseconds_status() const;

        bool is_stale() const;

      protected:
        static const std::uint32_t      status_stale_milliseconds_ = 500;
      };
    }

    /** \brief DeviceConfig provides a facility for retrieving and setting device config information for Quanergy sensors.
     */
    class DLLEXPORT DeviceConfig
    {
      using tcp = boost::asio::ip::tcp;

    public:
      /** \brief The constructor connects to the sensor and retrieves the initial information.
       *  \param host is the hostname or IP address of the sensor (the port is predefined)
       */
      DeviceConfig(const std::string& host);

      /* \brief Connects to the sensor and retrieves the initial information, caches basic information based on sensor type 
       * \return true on success, false o.w.
       */
      bool connect();

      /* \brief Connects to the sensor and retrieves the initial information, caches basic information based on sensor type
       * \param host is the hostname or IP address of the sensor (the port is predefined)
       * \return true on success, false o.w.
       */
      bool connect(std::string host);

      /* \brief Checks if the DeviceConfig class is currently connected to the sensor              
       * \return true on success, false o.w.
       */
      bool is_connected();

      /* \brief Disconnects from the sensor and closes the socket
       * \return true on success, false o.w.
       */
      bool disconnect();
  
      /* \brief request parameters from the sensor and if successful, get a pointer to the response. (Cached values will be updated on Set)
       * \return a pointer to a valid response if values are available, nullptr o.w.
      */
      SettingsAPI::SetOrQueryResponse::ConstPtr get_parameters();

      /* \brief request device info from the sensor and if successful, get a pointer to the response.
       * \return a pointer to a valid response if values are available, nullptr o.w.
      */
      SettingsAPI::DeviceInfoResponse::ConstPtr get_device_info();

      /* \brief request status from the sensor and if successful, get a pointer to the response.
       * \return a pointer to a valid response if values are available, nullptr o.w.
      */
      SettingsAPI::DeviceStatusResponse::ConstPtr get_status();

      /* \brief this is the preferred method by which to obtain a set request for filling out device parameters
       * \return a reference to an internally held set request so that the request can be filled out
      */
      SettingsAPI::SetRequest& new_set_request();

      /* \brief this method MUST be called in order to actually send the set request once filled out
       * \param cmd is a reference to the set request [ usage: send_set_command(new_set_request().set_netmask("255.255.255.0")); ]
       * \return true on success, false o.w.
       */
      bool send_set_request(SettingsAPI::SetRequest& cmd);

      /* \brief send request to rest parameters to factory default
       * \return true on success, false o.w.
       */
      bool reset_parameters();

      /* \brief send request to reboot the sensor
       * \return true on success, false o.w.
       */
      bool reboot_device();

      /// \brief get the model; empty string indicates an error loading the device info
      const std::string& model() const { return model_; }

      /// \brief get the amplitude if it was available in the device info
      const boost::optional<double>& amplitude() const { return amplitude_; }

      /// \brief get the encoder calibration phase if it was available in the device info
      const boost::optional<double>& phase() const { return phase_; }

      /// \brief get the vertical angles; empty means they were not available
      const std::vector<double>& verticalAngles() const { return vertical_angles_; }

      /// \brief get whether the sensor is an m-series sensor or not
      const bool& is_m_series() const { return m_series_; }

      /// \brief get whether the sensor supports the additional settings API calls
      bool supports_api_calls() const { return supports_api_; }
    
    private:

      /* \brief connect to the sensor on the HTTP port and request "basic" information using the non-API calls
       * \return true on success, false o.w.
       */
      bool get_basic_device_info();

      /// \brief send a query to the sensor to get all parameters at their most recent values
      void refresh_parameters();

      /* \brief actually perform the send and recieve of the request/response
       * \param request is a unique pointer to the request to be sent
       * \return a unique pointer to a valid response if successful, nullptr o.w.
       */
      SettingsAPI::File::UPtr send_request_get_response(SettingsAPI::File::UPtr request);

      std::string                host_;
      static const int           port_ = 5153;

      boost::asio::io_service    io_context_;
      tcp::socket                socket_;
      SettingsAPI::
        SetRequest::UPtr         set_request_;
      SettingsAPI::
        SetOrQueryResponse::Ptr  parameters_;
      bool                       parameters_fresh_ = false;
      bool                       supports_api_ = false;

      // From DeviceInfo class 
      const std::string          device_info_path_{ "/PSIA/System/deviceInfo" };

      std::string                model_;
      boost::optional<double>    amplitude_;
      boost::optional<double>    phase_;
      std::vector<double>        vertical_angles_;
      bool                       m_series_ = false;
    };
  }
}

#endif
