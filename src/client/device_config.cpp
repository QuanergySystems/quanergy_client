/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/client/device_config.h>

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <quanergy/client/device_info.h>

#include <quanergy/common/notifications.h>

// HTTP client to get calibration info from sensors
#include <quanergy/client/http_client.h>

#include <quanergy/parsers/data_packet_parser_m_series.h>

namespace quanergy
{
  namespace client
  {
    // PATH                       | Default   | Range {min, max}
    // --------------------------------------------------------------------------------------------------------------------------------
    // Ring.Range0                  0,          {0, 3000} cm
    // Ring.Intensity0              0,          {0, 255}
    //
    // Settings.InvertPPS           0,          {0, 1}
    // Settings.NmeaBaud            5,          {0, 12}
    // SettingsInvertSerial         0,          {0, 1}
    // Settings.returnselect        0,          {0, 3}
    // Settings.LED                 true,       {false, true}
    // Settings.BlockedCapDetect    true,       {false, true}
    // Settings.DisconnectSpeed     "fast",     {"fast", "slow"}
    // Motor.LowPowerSpinup         false,      {false, true}
    // Motor.FrameRate              10,         {?, ?} Hz - TicksPerTenth
    // FOV.fovcenter                180.0,      {0.0, 360.0} degrees
    // FOV.fovwidth                 360.0,      {0.0, 360.0} degrees
    // Network.type                 "dhcp",     {"dhcp", "static"}
    // Network.ipaddress            null,       <x.x.x.x> IPv4 formatted IP
    // Network.gateway              null,       <x.x.x.x> IPv4 formatted IP-Gateway
    // Network.netmask              null,       <x.x.x.x> IPv4 formatted IP-Netmask
    // Network.broadcast            null,       <x.x.x.x> IPv4 formatted IP
    // Network.NTPOption            false,      {false, true}
    // NTP.ntp_tp                   "default",  {"default", "local"}
    // NTP.ntp_ip_addr              null,       <x.x.x.x> IPv4 formatted IP
    //
    // DeviceInfo
    // -----------------------------------------------------------------------------------------------
    // REVISION
    // TOP_VER
    // BASE_REL
    // DEVICE
    // REV_NUM
    // LINUX
    // PS_CODE
    // LINUX_DRIVER
    // BIST
    // SCRIPTS
    // WEBSERVER
    // PL
    // MACAddress
    // SerialNumber
    //
    // calibration.<xmlattr>.version
    // calibration.amplitude
    // calibration.phase
    //
    // calibration.lasers.<xmlattr>.number
    // calibration.lasers.laser.<xmlattr>.id
    // calibration.lasers.laser.v
    //
    // Response querying
    //
    // Additional Info
    //  <value> The value that was supplied by the user and is echoed back to the user.
    //
    //  <min> and <max> indicate the minimum and maximum values that are considered
    //  valid for numeric settings.
    //
    //  <option> lists possible options that are valid for non-numeric settings, including
    //  booleans.
    
    namespace SettingsAPI
    {
      std::string to_string(Command command)
      {
        switch (command)
        {
        case Command::DeviceInfo:
          return "DeviceInfo";
        case Command::Set:
          return "Set";
        case Command::Reset:
          return "Reset";
        case Command::Reboot:
          return "Reboot";
        case Command::Query:
          return "Query";
        case Command::Status:
          return "Status";
        case Command::Invalid:
        default:
          return "Invalid";
        }
      }
      
      Command from_string(std::string command)
      {
        if (command == "DeviceInfo")
          return Command::DeviceInfo;
        else if (command == "Set")
          return Command::Set;
        else if (command == "Reset")
          return Command::Reset;
        else if (command == "Reboot")
          return Command::Reboot;
        else if (command == "Query")
          return Command::Query;
        else if (command == "Status")
          return Command::Status;
        else
          return Command::Invalid;
      }
      
      bool check_status(int status, std::string& errMsg)
      {
        /*
          OK_CODE				200  The entire request was accepted cleanly without problems.
          ACCEPTED_CODE			202  Request was accepted and answered.
          BAD_REQUEST_CODE		400  The requested setting value is invalid.
          NOT_FOUND_CODE		404  The requested setting key is does not exist in the system. The user may
          							  have misspelled the name. The setting will be ignored in the sensor.
          NOT_ACCEPTABLE		406  The command that was given doesn't match one of the commands above.
          PARTIAL_FAILURE_CODE	460  In contrast to OK_CODE, this means some, but not all, requests were
          							  not successful. Those ones that did not work are tagged with a status
          							  code indicating how it doesn't work.
          INTERNAL_SERVER_ERROR	500  The sensor has some problem reading or writing the settings.
        */
        
        switch (status)
        {
        case -2:
          errMsg = "Parse error";
          break;
        case -1:
          errMsg = "Status not read from XML";
          break;
        case 0:
          errMsg = "Command has no status";
          break;
        //   200: Okay. Query command was processed successfully.
        case 200:
          errMsg = "OK";
          return true;
        //   202: Accepted. Set command succeeded and the parameter updated with the
        //        requested value
        case 202:
          errMsg = "OK";
          return true;
        //   400: Bad Request. Set command failed because the provided parameter value was
        //        invalid.For example, setting fovwidth to a value of - 90 would not be valid.
        case 400:
          errMsg = "400: Bad Request.";
          break;
        //   404 : Not Found. Query command failed due to not being found. 
        //         OR Set command failed due to not being found, that is, because the
        //         setting name is invalid.
        case 404:
          errMsg = "404 : Not Found.";
          break;
        //   406 : Command not recognized
        case 406:
          errMsg = "406 : Command not recognized.";
          break;
        //   460 : Partial Failure. At least one setting failed to be set or queried.
        case 460:
          errMsg = "460 : Partial Failure.";
          break;
        //   500 : Internal Server Error. Sensor has some problem reading or writing the settings.
        case 500:
          errMsg = "500 : Internal Server Error.";
          break;
        default:
          errMsg = "Status not recognized";
        }
        return false;
      }
      
      bool validate_reponse(const File& response, Command command_type)
      {
        // Validate that the requested type matches the actual type.
        // In the case of Set and Query, treat those interchangably. 
        if (((response.type() == Command::Set || response.type() == Command::Query) &&
          !(command_type == Command::Set || command_type == Command::Query)) ||
          response.type() != command_type)
        {
          qout << "[Warning] Response command not same as type requested. Command: " << to_string(response.type())
            << "; Requested Type: " << to_string(command_type) << std::endl;
          return false;
        }
        
        return true;
      }
      
      File::File() 
        : boost::property_tree::ptree() 
      {}
      
      File::File(Command command)
        : boost::property_tree::ptree()
        , command_(command)
      {
        // Put the correct command header string at the top of the ptree
        this->put("SettingsAPI.Command", to_string(command));
      }
      
      bool File::from_stream(std::istream& stream)
      {
        this->clear();
        boost::property_tree::xml_parser::read_xml(stream, static_cast<boost::property_tree::ptree&>(*this));
        if (!parse_header())
        {
          qout << "[Error] Failed to parse xml header for SettingsAPI::File!" << std::endl;
          return false;
        }
        message_timestamp_ = std::chrono::steady_clock::now();
        return true;
      }
      
      void File::to_stream(std::ostream& stream) const
      {
        boost::property_tree::xml_parser::write_xml(stream, static_cast<const boost::property_tree::ptree&>(*this));
      }
      
      std::string File::status_str() const
      { 
        std::string msg;  
        check_status(status_, msg);
        return msg; 
      }
      
      bool File::parse_header()
      {
        // Check for the response node
        auto response_node = this->get_child_optional("SettingsAPI.Response");
        if (!response_node)
        {
          qout << "[Warning] Response node not found! File is not a response!" << std::endl;
          status_ = -2;
          return false;
        }
        
        // Get the actual command from the response
        boost::optional<std::string> command = response_node->get_optional<std::string>("Command");
        if (!command)
        {
          qout << "[Warning] Response command not found!" << std::endl;
          status_ = -2;
          return false;
        }
        
        // Validate that the command is a "known" type
        Command read_command = from_string(command.value());
        if (read_command == Command::Invalid)
        {
          qout << "[Warning] Response command unknown. Command: " << command << std::endl;
          status_ = -2;
          return false;
        }
        
        // If this file was built with a command and it is not the same as the response type, that's bad
        if (command_ != Command::Invalid && command_ != read_command)
        {
          qout << "[Warning] File was built with command type: " << to_string(command_) 
            << " but the response type was: " << command << std::endl;
          status_ = -2;
          return false;
        }
        
        // Adopt the read_command
        command_ = read_command;
        
        // Then get and check the status to make sure that the response is okay
        std::string errMsg;
        status_ = response_node->get("Status", -1);
        if (!check_status(status_, errMsg) &&
          !(status_ == 460 && command_ == Command::Set)) // Allow partial failure in the case of set.
        {
          qout << "[Warning] check_status for response header returned an error status: " << errMsg << std::endl;
          return false;
        }
        
        return true;
      }
      
      template <typename T>
      bool File::try_get(std::string path, T& value) const
      {
        boost::optional<T> optional_value = try_get<T>(path);
        
        if (!optional_value)
          return false;
        
        value = *optional_value;
        return true;
      }
      
      template <typename T>
      boost::optional<T> File::try_get(std::string path) const
      {
        Command command = command_;
        // This should never fire, because parse_header should set command_ 
        if (command == Command::Invalid)
        { // FIRST!  Try to read the command type to know HOW to parse it...
          
          // Get the actual command from the response
          std::string command_str = this->get<std::string>("SettingsAPI.Response.Command", "");
          
          // Validate that the command is a "known" type
          command = from_string(command_str);
        }
        
        // If the command type is STILL invalid, that's a problem
        if (command == Command::Invalid)
        {
          qout << "[Error] Failed to parse command type... can't parse value." << std::endl;
          return boost::optional<T>();
        }
        
        // If the command type is NOT Set or Query, then try to get the contents directly
        if (command != Command::Set && command != Command::Query)
        {
          boost::optional<T> optional_val = this->get_optional<T>(path);
          if (!optional_val)
          {
            qout << "[Warning] The item was not able to be obtained from the path: "
              << path << std::endl;
            return boost::optional<T>();
          }
          return optional_val;
        }
        
        // If we make it here, we are trying to parse a Set or Query command, so check the status of the item itself
        
        // Try to get the requested node from the ptree at the path given
        auto child_node = this->get_child_optional(path);
        if (!child_node)
        { // No child was found, print error and return fail.
          qout << "[Warning] Requested item: " << path << " not found in response!" << std::endl;
          return boost::optional<T>();
        }
        
        // Try to get the status from the ptree tag
        std::string errMsg;
        boost::optional<T> optional_value = child_node->get_optional<T>("value");
        if (check_status(child_node->get("<xmlattr>.status", -1), errMsg))
        { // If check status returns OK, then read the value and return success.
          return optional_value;
        }
        
        // Add the error message to the output
        qout << "[Info] check_status for variable: [" << path << "] returned an error status: " << errMsg << std::endl;
        
        // Try to get the value
        if (!optional_value)
        {
          qout << "[Debug] Value was not able to be obtained from the path: "
            << path << ".value" << std::endl;
          return boost::optional<T>();
        }
        
        // If the value was obtained, try to discern why it wasn't okay. 
        
        // Get the min/max (if possible)
        boost::optional<T> min = child_node->get_optional<T>("min");
        boost::optional<T> max = child_node->get_optional<T>("max");
        
        // If the min/max were obtained and the value is outside the min / max; publish error
        if (min && max)
        {
          if ((optional_value.value() < min.value() || optional_value.value() > max.value()))
          {
            qout << "[Debug] Value: " << optional_value.value()
              << " was outside the allowable range: {"
              << min.value() << ", " << max.value() << "}" << std::endl;
            return boost::optional<T>();
          }
        }
        else
        {
          // get options, if available
          std::vector<T> options;
          for (auto& node : *child_node)
          {
            if (node.first == "option")
            {
              options.push_back(node.second.get_value<T>());
            }
          }
          
          bool found = false;
          for (auto& op : options)
          {
            if (op == optional_value.value())
            {
              found = true;
              break;
            }
          }
          
          if (!found)
          {
            qout << "[Debug] Value: " << optional_value.value()
              << " was not a valid option: {";
            
            for (auto& op : options)
              qout << op << ",";
            
            qout << "}" << std::endl;
            
            return boost::optional<T>();
          }
        }
        
        qout << "[Debug] Error unknown..." << std::endl;
        return boost::optional<T>();
      }
      
      SetRequest::SetRequest()
        : File(Command::Set)
      {}
      
      SetRequest& SetRequest::set_range(int index, int value)
      {
        const std::string num = boost::lexical_cast<std::string>(index);
        std::string range_param = std::string("SettingsAPI.Data.Ring.Range").append(num);
        this->put(range_param, value);
        return *this;
      }
      
      SetRequest& SetRequest::set_intensity(int index, int value)
      {
        const std::string num = boost::lexical_cast<std::string>(index);
        std::string intensity_param = std::string("SettingsAPI.Data.Ring.Intensity").append(num);
        this->put(intensity_param, value);
        return *this;
      }
      
      SetRequest& SetRequest::set_range(std::vector<int> value)
      {
        for (int i = 0; i < value.size(); i++)
          set_range(i, value[i]);
        
        return *this;
      }
      
      SetRequest& SetRequest::set_intensity(std::vector<int> value)
      {
        for (int i = 0; i < value.size(); i++)
          set_intensity(i, value[i]);
        
        return *this;
      }
      
      SetRequest& SetRequest::set_invertPPS(int value)
      {
        this->put("SettingsAPI.Data.Settings.InvertPPS", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_nmeaBaud(int value)
      {
        this->put("SettingsAPI.Data.Settings.NmeaBaud", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_invertSerial(int value)
      {
        this->put("SettingsAPI.Data.SettingsInvertSerial", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_returnSelect(int value)
      {
        this->put("SettingsAPI.Data.Settings.returnselect", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_led(bool value)
      {
        this->put("SettingsAPI.Data.Settings.LED", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_blockedCapDetect(bool value)
      {
        this->put("SettingsAPI.Data.Settings.BlockedCapDetect", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_disconnectSpeed(std::string value)
      {
        this->put("SettingsAPI.Data.Settings.DisconnectSpeed", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_lowPowerSpinup(bool value)
      {
        this->put("SettingsAPI.Data.Motor.LowPowerSpinup", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_frameRate(int value)
      {
        this->put("SettingsAPI.Data.Motor.FrameRate", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_fovCenter(float value)
      {
        this->put("SettingsAPI.Data.FOV.fovcenter", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_fovWidth(float value)
      {
        this->put("SettingsAPI.Data.FOV.fovwidth", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_networkType(std::string value)
      {
        this->put("SettingsAPI.Data.Network.type", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_ipAddress(std::string value)
      {
        this->put("SettingsAPI.Data.Network.ipaddress", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_gateway(std::string value)
      {
        this->put("SettingsAPI.Data.Network.gateway", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_netmask(std::string value)
      {
        this->put("SettingsAPI.Data.Network.netmask", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_broadcast(std::string value)
      {
        this->put("SettingsAPI.Data.Network.broadcast", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_ntpOption(bool value)
      {
        this->put("SettingsAPI.Data.Network.NTPOption", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_ntp_tp(std::string value)
      {
        this->put("SettingsAPI.Data.NTP.ntp_tp", value);
        return *this;
      }
      
      SetRequest& SetRequest::set_ntp_ip_addr(std::string value)
      {
        this->put("SettingsAPI.Data.NTP.ntp_ip_addr", value);
        return *this;
      }
      
      SetOrQueryResponse::SetOrQueryResponse()
        : File()
      { }
      
      bool SetOrQueryResponse::get_range(int index, int& value) const
      {
        const std::string num = boost::lexical_cast<std::string>(index);
        std::string param = std::string("SettingsAPI.Data.Ring.Range").append(num);
        return try_get(param, value);
      }
      
      bool SetOrQueryResponse::get_intensity(int index, int& value) const
      {
        const std::string num = boost::lexical_cast<std::string>(index);
        std::string param = std::string("SettingsAPI.Data.Ring.Intensity").append(num);
        return try_get(param, value);
      }
      
      bool SetOrQueryResponse::get_range(std::vector<int>& value) const
      {
        bool success = true;
        if (value.size() < num_beams_)
          value.resize(num_beams_, 0);
        
        for (int i = 0; i < num_beams_; i++)
          success &= get_range(i, value[i]);
        
        return success;
      }
      
      bool SetOrQueryResponse::get_intensity(std::vector<int>& value) const
      {
        bool success = true;
        if (value.size() < num_beams_)
          value.resize(num_beams_, 0);
        
        for (int i = 0; i < num_beams_; i++)
          success &= get_intensity(i, value[i]);
        
        return success;
      }
      
      bool SetOrQueryResponse::get_invertPPS(int& value) const
      {
        return try_get("SettingsAPI.Data.Settings.InvertPPS", value);
      }
      
      bool SetOrQueryResponse::get_nmeaBaud(int& value) const
      {
        return try_get("SettingsAPI.Data.Settings.NmeaBaud", value);
      }
      
      bool SetOrQueryResponse::get_invertSerial(int& value) const
      {
        return try_get("SettingsAPI.Data.Settings.InvertSerial", value);
      }
      
      bool SetOrQueryResponse::get_returnSelect(int& value) const
      {
        return try_get("SettingsAPI.Data.Settings.returnselect", value);
      }
      
      bool SetOrQueryResponse::get_led(bool& value) const
      {
        return try_get("SettingsAPI.Data.Settings.LED", value);
      }
      
      bool SetOrQueryResponse::get_blockedCapDetect(bool& value) const
      {
        return try_get("SettingsAPI.Data.Settings.BlockedCapDetect", value);
      }
      
      bool SetOrQueryResponse::get_disconnectSpeed(std::string& value) const
      {
        return try_get("SettingsAPI.Data.Settings.DisconnectSpeed", value);
      }
      
      bool SetOrQueryResponse::get_lowPowerSpinup(bool& value) const
      {
        return try_get("SettingsAPI.Data.Motor.LowPowerSpinup", value);
      }
      
      bool SetOrQueryResponse::get_frameRate(int& value) const
      {
        return try_get("SettingsAPI.Data.Motor.FrameRate", value);
      }
      
      bool SetOrQueryResponse::get_fovCenter(float& value) const
      {
        return try_get("SettingsAPI.Data.FOV.fovcenter", value);
      }
      
      bool SetOrQueryResponse::get_fovWidth(float& value) const
      {
        return try_get("SettingsAPI.Data.FOV.fovwidth", value);
      }
      
      bool SetOrQueryResponse::get_networkType(std::string& value) const
      {
        return try_get("SettingsAPI.Data.Network.type", value);
      }
      
      bool SetOrQueryResponse::get_ipAddress(std::string& value) const
      {
        return try_get("SettingsAPI.Data.Network.ipaddress", value);
      }
      
      bool SetOrQueryResponse::get_gateway(std::string& value) const
      {
        return try_get("SettingsAPI.Data.Network.gateway", value);
      }
      
      bool SetOrQueryResponse::get_netmask(std::string& value) const
      {
        return try_get("SettingsAPI.Data.Network.netmask", value);
      }
      
      bool SetOrQueryResponse::get_broadcast(std::string& value) const
      {
        return try_get("SettingsAPI.Data.Network.broadcast", value);
      }
      
      bool SetOrQueryResponse::get_ntpOption(bool& value) const
      {
        return try_get("SettingsAPI.Data.Network.NTPOption", value);
      }
      
      bool SetOrQueryResponse::get_ntp_tp(std::string& value) const
      {
        return try_get("SettingsAPI.Data.NTP.ntp_tp", value);
      }
      
      bool SetOrQueryResponse::get_ntp_ip_addr(std::string& value) const
      {
        return try_get("SettingsAPI.Data.NTP.ntp_ip_addr", value);
      }
      
      boost::optional<int> SetOrQueryResponse::get_range(int index) const
      {
        const std::string num = boost::lexical_cast<std::string>(index);
        std::string param = std::string("SettingsAPI.Data.Ring.Range").append(num);
        return try_get<int>(param);
      }
      
      boost::optional<int> SetOrQueryResponse::get_intensity(int index) const
      {
        const std::string num = boost::lexical_cast<std::string>(index);
        std::string param = std::string("SettingsAPI.Data.Ring.Intensity").append(num);
        return try_get<int>(param);
      }
      
      boost::optional<std::vector<int>> SetOrQueryResponse::get_range() const
      {
        boost::optional<std::vector<int>> optional_value;
        std::vector<int>value = std::vector<int>(num_beams_, 0);
        bool success = true;
        for (int i = 0; i < num_beams_; i++)
        {
          success &= get_range(i, value[i]);
        }
        
        if (success)
          optional_value.emplace(value);
        
        return optional_value;
      }
      
      boost::optional<std::vector<int>> SetOrQueryResponse::get_intensity() const
      {
        boost::optional<std::vector<int>> optional_value;
        std::vector<int>value = std::vector<int>(num_beams_, 0);
        bool success = true;
        for (int i = 0; i < num_beams_; i++)
        {
          success &= get_intensity(i, value[i]);
        }
        
        if (success)
          optional_value.emplace(value);
        
        return optional_value;
      }
      
      boost::optional<int> SetOrQueryResponse::get_invertPPS() const
      {
        return try_get<int>("SettingsAPI.Data.Settings.InvertPPS");
      }
      
      boost::optional<int> SetOrQueryResponse::get_nmeaBaud() const
      {
        return try_get<int>("SettingsAPI.Data.Settings.NmeaBaud");
      }
      
      boost::optional<int> SetOrQueryResponse::get_invertSerial() const
      {
        return try_get<int>("SettingsAPI.Data.Settings.InvertSerial");
      }
      
      boost::optional<int> SetOrQueryResponse::get_returnSelect() const
      {
        return try_get<int>("SettingsAPI.Data.Settings.returnselect");
      }
      
      boost::optional<bool> SetOrQueryResponse::get_led() const
      {
        return try_get<bool>("SettingsAPI.Data.Settings.LED");
      }
      
      boost::optional<bool> SetOrQueryResponse::get_blockedCapDetect() const
      {
        return try_get<bool>("SettingsAPI.Data.Settings.BlockedCapDetect");
      }
      
      boost::optional<std::string> SetOrQueryResponse::get_disconnectSpeed() const
      {
        return try_get<std::string>("SettingsAPI.Data.Settings.DisconnectSpeed");
      }
      
      boost::optional<bool> SetOrQueryResponse::get_lowPowerSpinup() const
      {
        return try_get<bool>("SettingsAPI.Data.Motor.LowPowerSpinup");
      }
      
      boost::optional<int> SetOrQueryResponse::get_frameRate() const
      {
        return try_get<int>("SettingsAPI.Data.Motor.FrameRate");
      }
      
      boost::optional<float> SetOrQueryResponse::get_fovCenter() const
      {
        return try_get<float>("SettingsAPI.Data.FOV.fovcenter");
      }
      
      boost::optional<float> SetOrQueryResponse::get_fovWidth() const
      {
        return try_get<float>("SettingsAPI.Data.FOV.fovwidth");
      }
      
      boost::optional<std::string> SetOrQueryResponse::get_networkType() const
      {
        return try_get<std::string>("SettingsAPI.Data.Network.type");
      }
      
      boost::optional<std::string> SetOrQueryResponse::get_ipAddress() const
      {
        return try_get<std::string>("SettingsAPI.Data.Network.ipaddress");
      }
      
      boost::optional<std::string> SetOrQueryResponse::get_gateway() const
      {
        return try_get<std::string>("SettingsAPI.Data.Network.gateway");
      }
      
      boost::optional<std::string> SetOrQueryResponse::get_netmask() const
      {
        return try_get<std::string>("SettingsAPI.Data.Network.netmask");
      }
      
      boost::optional<std::string> SetOrQueryResponse::get_broadcast() const
      {
        return try_get<std::string>("SettingsAPI.Data.Network.broadcast");
      }
      
      boost::optional<bool> SetOrQueryResponse::get_ntpOption() const
      {
        return try_get<bool>("SettingsAPI.Data.Network.NTPOption");
      }
      
      boost::optional<std::string> SetOrQueryResponse::get_ntp_tp() const
      {
        return try_get<std::string>("SettingsAPI.Data.NTP.ntp_tp");
      }
      
      boost::optional<std::string> SetOrQueryResponse::get_ntp_ip_addr() const
      {
        return try_get<std::string>("SettingsAPI.Data.NTP.ntp_ip_addr");
      }
      
      bool SetOrQueryResponse::copy_from(const SetOrQueryResponse& response)
      {
        // We are copying from another SetOrQueryResponse to this one
        // first, grab the data section of the ptree so we can iterate it
        auto data = response.get_child_optional("SettingsAPI.Data");
        if (!data)
        {
          qout << "[Debug] Response is missing the data section, nothing to copy from!" << std::endl;
          return false;
        }
        
        int items_copied = 0;
        // Iterate the data section, checking each node and adding it into this ptree if it is okay.
        for (auto& node : *data)
        { // Iterate the "section" nodes
          for (auto& innerNode : node.second)
          { // Iterate the "item" nodes
            std::string path = "SettingsAPI.Data." + node.first + "." + innerNode.first;
            int status = innerNode.second.get<int>("<xmlattr>.status", -1);
            if (check_status(status, std::string()))
            { // If the status was OK, then copy the status and value
              auto value = innerNode.second.get_optional<std::string>("value");
              if (value)
              { // Copy the contents if the value exists!
                this->put(path + ".<xmlattr>.status", status);
                this->put(path + ".value", *value);
                items_copied++;
              } // if value exists
            } // if status ok
            else
            {// If the status check fails, then publish a warning, and don't copy
              qout << "[Debug] Item: " << path
                << " had status: " << status
                << " and was not copied!" << std::endl;
            } // if inner node
          } // for outer node in inner node
        } // for node in data
        
        return (items_copied > 0);
      }
      
      DeviceInfoResponse::DeviceInfoResponse()
        : File()
      { }
      
      bool DeviceInfoResponse::get_revision(std::string& value) const
      {
        return try_get("SettingsAPI.Data.REVISION", value);
      }
      
      bool DeviceInfoResponse::get_top_version(std::string& value) const
      {
        return try_get("SettingsAPI.Data.TOP_VER", value);
      }
      
      bool DeviceInfoResponse::get_base_release(std::string& value) const
      {
        return try_get("SettingsAPI.Data.BASE_REL", value);
      }
      
      bool DeviceInfoResponse::get_model (std::string& value) const
      {
        return try_get("SettingsAPI.Data.DEVICE", value);
      }
      
      bool DeviceInfoResponse::get_revision_number(std::string& value) const
      {
        return try_get("SettingsAPI.Data.REV_NUM", value);
      }
      
      bool DeviceInfoResponse::get_linux_OS(std::string& value) const
      {
        return try_get("SettingsAPI.Data.LINUX", value);
      }
      
      bool DeviceInfoResponse::get_ps_code(std::string& value) const
      {
        return try_get("SettingsAPI.Data.PS_CODE", value);
      }
      
      bool DeviceInfoResponse::get_linux_driver(std::string& value) const
      {
        return try_get("SettingsAPI.Data.LINUX_DRIVER", value);
      }
      
      bool DeviceInfoResponse::get_bist(std::string& value) const
      {
        return try_get("SettingsAPI.Data.BIST", value);
      }
      
      bool DeviceInfoResponse::get_scripts(std::string& value) const
      {
        return try_get("SettingsAPI.Data.SCRIPTS", value);
      }
      
      bool DeviceInfoResponse::get_webserver(std::string& value) const
      {
        return try_get("SettingsAPI.Data.WEBSERVER", value);
      }
      
      bool DeviceInfoResponse::get_pl(std::string& value) const
      {
        return try_get("SettingsAPI.Data.PL", value);
      }
      
      bool DeviceInfoResponse::get_mac_address(std::string& value) const
      {
        return try_get("SettingsAPI.Data.MACAddress", value);
      }
      
      bool DeviceInfoResponse::get_serial_number(std::string& value) const
      {
        return try_get("SettingsAPI.Data.SerialNumber", value);
      }
      
      bool DeviceInfoResponse::get_calibration_version(double& value) const
      {
        return try_get("SettingsAPI.Data.calibration.<xmlattr>.version", value);
      }
      
      bool DeviceInfoResponse::get_encoder_amplitude(double& value) const
      {
        return try_get("SettingsAPI.Data.calibration.encoder.amplitude", value);
      }
      
      bool DeviceInfoResponse::get_encoder_phase(double& value) const
      {
        return try_get("SettingsAPI.Data.calibration.encoder.phase", value);
      }
      
      bool DeviceInfoResponse::get_number_beams(int& value) const
      {
        return try_get("SettingsAPI.Data.calibration.lasers.<xmlattr>.number", value);
      }
      
      bool DeviceInfoResponse::get_vertical_angles(std::vector<double>& value) const
      {
        auto laser_data = this->get_child_optional("SettingsAPI.Data.calibration.lasers");
        if (laser_data)
        {
          value.resize(laser_data->get<unsigned int>("<xmlattr>.number"), 0.0);
          for (auto& laser : *laser_data)
          {
            if (laser.first == "laser")
            {
              auto id = laser.second.get<unsigned int>("<xmlattr>.id");
              if (id < value.size())
              {
                value[id] = laser.second.get<double>("v");
              } // if id valid
            } // if laser
          } // for laser
          return true;
        } // if laser data
        
        return false;
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_revision() const
      {
        return try_get<std::string>("SettingsAPI.Data.REVISION");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_top_version() const
      {
        return try_get<std::string>("SettingsAPI.Data.TOP_VER");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_base_release() const
      {
        return try_get<std::string>("SettingsAPI.Data.BASE_REL");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_model() const
      {
        return try_get<std::string>("SettingsAPI.Data.DEVICE");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_revision_number() const
      {
        return try_get<std::string>("SettingsAPI.Data.REV_NUM");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_linux_OS() const
      {
        return try_get<std::string>("SettingsAPI.Data.LINUX");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_ps_code() const
      {
        return try_get<std::string>("SettingsAPI.Data.PS_CODE");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_linux_driver() const
      {
        return try_get<std::string>("SettingsAPI.Data.LINUX_DRIVER");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_bist() const
      {
        return try_get<std::string>("SettingsAPI.Data.BIST");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_scripts() const
      {
        return try_get<std::string>("SettingsAPI.Data.SCRIPTS");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_webserver() const
      {
        return try_get<std::string>("SettingsAPI.Data.WEBSERVER");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_pl() const
      {
        return try_get<std::string>("SettingsAPI.Data.PL");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_mac_address() const
      {
        return try_get<std::string>("SettingsAPI.Data.MACAddress");
      }
      
      boost::optional<std::string> DeviceInfoResponse::get_serial_number() const
      {
        return try_get<std::string>("SettingsAPI.Data.SerialNumber");
      }
      
      boost::optional<double> DeviceInfoResponse::get_calibration_version() const
      {
        return try_get<double>("SettingsAPI.Data.calibration.<xmlattr>.version");
      }
      
      boost::optional<double> DeviceInfoResponse::get_encoder_amplitude() const
      {
        return try_get<double>("SettingsAPI.Data.calibration.encoder.amplitude");
      }
      
      boost::optional<double> DeviceInfoResponse::get_encoder_phase() const
      {
        return try_get<double>("SettingsAPI.Data.calibration.encoder.phase");
      }
      
      boost::optional<int> DeviceInfoResponse::get_number_beams() const
      {
        return try_get<int>("SettingsAPI.Data.calibration.lasers.<xmlattr>.number");
      }
      
      boost::optional<std::vector<double>> DeviceInfoResponse::get_vertical_angles() const
      {
        boost::optional<std::vector<double>> optional_value;
        auto laser_data = this->get_child_optional("SettingsAPI.Data.calibration.lasers");
        if (laser_data)
        {
          std::vector<double> value = 
            std::vector<double>(laser_data->get<unsigned int>("<xmlattr>.number"), 0.0);
          for (auto& laser : *laser_data)
          {
            if (laser.first == "laser")
            {
              auto id = laser.second.get<unsigned int>("<xmlattr>.id");
              if (id < value.size())
              {
                value[id] = laser.second.get<double>("v");
              } // if id valid
            } // if laser
          } // for laser
          optional_value.emplace(value);
        } // if laser data
        
        return optional_value;
      }
      
      DeviceStatusResponse::DeviceStatusResponse()
        : File()
      { }
      
      bool DeviceStatusResponse::get_gps_status(std::string& value) const
      {
        return try_get("SettingsAPI.Data.GPSStatus", value);
      }
      
      bool DeviceStatusResponse::get_frameRate_status(double& value) const
      {
        return try_get("SettingsAPI.Data.FrameRate", value);
      }
      
      bool DeviceStatusResponse::get_sensor_temp_status(double& value) const
      {
        return try_get("SettingsAPI.Data.SensorTemperature", value);
      }
      
      bool DeviceStatusResponse::get_watchdog_status(int& value) const
      {
        return try_get("SettingsAPI.Data.WatchdogStatus", value);
      }
      
      bool DeviceStatusResponse::get_motor_init_failure_status(int& value) const
      {
        return try_get("SettingsAPI.Data.MotorInitializationFailure", value);
      }
      
      bool DeviceStatusResponse::get_motor_velocity_failure_status(int& value) const
      {
        return try_get("SettingsAPI.Data.InsufficientMotorVelocityFailure", value);
      }
      
      bool DeviceStatusResponse::get_blocked_cap_status(std::string& value) const
      {
        return try_get("SettingsAPI.Data.BlockedCap", value);
      }
      
      bool DeviceStatusResponse::get_version_mismatch_error_status(int& value) const
      {
        return try_get("SettingsAPI.Data.VersionMismatchError", value);
      }
      
      bool DeviceStatusResponse::get_laser_overtemp_error_status(int& value) const
      {
        return try_get("SettingsAPI.Data.LaserOvertempError", value);
      }
      
      bool DeviceStatusResponse::get_cpu_overtemp_error_status(int& value) const
      {
        return try_get("SettingsAPI.Data.CPUOvertempError", value);
      }
      
      bool DeviceStatusResponse::get_seconds_status(std::uint64_t& value) const
      {
        return try_get("SettingsAPI.Data.Second", value);
      }
      
      bool DeviceStatusResponse::get_nanoseconds_status(std::uint64_t& value) const
      {
        return try_get("SettingsAPI.Data.Nanoseconds", value);
      }
      
      boost::optional<std::string> DeviceStatusResponse::get_gps_status() const
      {
        return try_get<std::string>("SettingsAPI.Data.GPSStatus");
      }
      
      boost::optional<double> DeviceStatusResponse::get_frameRate_status() const
      {
        return try_get<double>("SettingsAPI.Data.FrameRate");
      }
      
      boost::optional<double> DeviceStatusResponse::get_sensor_temp_status() const
      {
        return try_get<double>("SettingsAPI.Data.SensorTemperature");
      }
      
      boost::optional<int> DeviceStatusResponse::get_watchdog_status() const
      {
        return try_get<int>("SettingsAPI.Data.WatchdogStatus");
      }
      
      boost::optional<int> DeviceStatusResponse::get_motor_init_failure_status() const
      {
        return try_get<int>("SettingsAPI.Data.MotorInitializationFailure");
      }
      
      boost::optional<int> DeviceStatusResponse::get_motor_velocity_failure_status() const
      {
        return try_get<int>("SettingsAPI.Data.InsufficientMotorVelocityFailure");
      }
      
      boost::optional<std::string> DeviceStatusResponse::get_blocked_cap_status() const
      {
        return try_get<std::string>("SettingsAPI.Data.BlockedCap");
      }
      
      boost::optional<int> DeviceStatusResponse::get_version_mismatch_error_status() const
      {
        return try_get<int>("SettingsAPI.Data.VersionMismatchError");
      }
      
      boost::optional<int> DeviceStatusResponse::get_laser_overtemp_error_status() const
      {
        return try_get<int>("SettingsAPI.Data.LaserOvertempError");
      }
      
      boost::optional<int> DeviceStatusResponse::get_cpu_overtemp_error_status() const
      {
        return try_get<int>("SettingsAPI.Data.CPUOvertempError");
      }
      
      boost::optional<std::uint64_t> DeviceStatusResponse::get_seconds_status() const
      {
        return try_get<std::uint64_t>("SettingsAPI.Data.Second");
      }
      
      boost::optional<std::uint64_t> DeviceStatusResponse::get_nanoseconds_status() const
      {
        return try_get<std::uint64_t>("SettingsAPI.Data.Nanoseconds");
      }
      
      bool DeviceStatusResponse::is_stale() const
      {
        return (std::chrono::steady_clock::now() - message_timestamp_) > std::chrono::milliseconds(status_stale_milliseconds_);
      }
    }
    
    DeviceConfig::DeviceConfig(const std::string& host)
      : host_(host)
      , socket_(io_context_)
      , set_request_(new SettingsAPI::SetRequest())
    {
      connect();
    }
    
    bool DeviceConfig::connect(std::string host) 
    { 
      host_ = host; 
      return connect(); 
    }
    
    bool DeviceConfig::connect()
    {
      // Disallow connecting if already connected
      if (is_connected())
      {
        qout << "[Warning] Connection to host: "
          << host_ << " port: " << port_
          << " is already open! Please call disconnect() before reconnecting!" << std::endl;
        return false; //error
      }
      
      // Try to get the basic device info from the sensor
      if (!get_basic_device_info())
      {
        qerr << "[Error] Call to DeviceInfo failed! Unable to determine model! " << std::endl;
        return false; //error
      }
      
      // Reset the ASIO io_context for good measure.
      io_context_.reset();
      
      // Make a host query for finding applicable endpoints
      tcp::resolver::query host_query_(host_, std::to_string(port_));
      
      // Get a list of endpoints corresponding to the server name.
      boost::asio::ip::tcp::resolver resolver(io_context_);
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(host_query_);
      
      // Try each endpoint until we successfully establish a connection.
      socket_ = boost::asio::ip::tcp::socket(io_context_);
      boost::asio::connect(socket_, endpoint_iterator);
      
      // If the socket did not open, we failed to connect, return false;
      if (!socket_.is_open())
      {
        qout << "[Warning] Failed to open TCP socket connection to host: "
          << host_ << " port: " << port_ << std::endl;
        return false; //error
      }
      
      // Refresh parameters on connect if the device supports the API
      if (supports_api_)
      {
        refresh_parameters();
      }
      
      return true;
    }
    
    bool DeviceConfig::is_connected()
    {
      return socket_.is_open();
    }
    
    bool DeviceConfig::disconnect()
    { 
      bool success = true;
      if (is_connected())
      {
        boost::system::error_code error;
        socket_.shutdown(socket_.shutdown_both, error);
        if (error)
        {
          success = false;
          qout << "[Warning] Error in shutting down socket: " << error.message() << std::endl;
        }
        error = boost::system::error_code();
        socket_.close(error);
        if (error)
        {
          success = false;
          qout << "[Warning] Error in closing socket: " << error.message() << std::endl;
        }
      }
      return success;
    }
    
    SettingsAPI::SetOrQueryResponse::ConstPtr DeviceConfig::get_parameters()
    {
      if (!parameters_ || !parameters_fresh_)
      {
        refresh_parameters();
      }
      
      // Return a full copy of the parameters
      parameters_fresh_ = false;
      return parameters_;
    }
    
    SettingsAPI::DeviceInfoResponse::ConstPtr DeviceConfig::get_device_info()
    {
      if (auto response = send_request_get_response(
        std::make_unique<SettingsAPI::File>(SettingsAPI::Command::DeviceInfo)
      ))
      {
        return SettingsAPI::DeviceInfoResponse::Ptr(
          static_cast<SettingsAPI::DeviceInfoResponse*>(response.release()));
      }
      // If no response, we failed
      return nullptr;
    }
    
    SettingsAPI::DeviceStatusResponse::ConstPtr DeviceConfig::get_status()
    {
      if (auto response = send_request_get_response(
        std::make_unique<SettingsAPI::File>(SettingsAPI::Command::Status)
      ))
      {
        return SettingsAPI::DeviceStatusResponse::Ptr(
          static_cast<SettingsAPI::DeviceStatusResponse*>(response.release()));
      }
      // If no response, we failed
      return nullptr;
    }
    
    bool DeviceConfig::reset_parameters()
    {
      if (auto response = send_request_get_response(
        std::make_unique<SettingsAPI::File>(SettingsAPI::Command::Reset)
      ))
      {
        return true;
      }
      // If no response, we failed
      return false;
    }
    
    bool DeviceConfig::reboot_device()
    {
      if (auto response = send_request_get_response(
        std::make_unique<SettingsAPI::File>(SettingsAPI::Command::Reboot)
      ))
      {
        return true;
      }
      // If no response, we failed
      return false;
    }
    
    SettingsAPI::SetRequest& DeviceConfig::new_set_request()
    {
      // Start from scratch, make a new set command
      set_request_.reset(new SettingsAPI::SetRequest());
      return *set_request_;
    }
    
    bool DeviceConfig::send_set_request(SettingsAPI::SetRequest& cmd)
    {
      if (auto response = send_request_get_response(
        std::make_unique<SettingsAPI::SetRequest>(cmd)
      ))
      {
        if (!parameters_)
        {
          // If we don't have a complete set of parameters yet, get one
          refresh_parameters();
        }
        else
        {
          // If we already have parameters, then update them with the results of the set response
          //SettingsAPI::SetOrQueryResponse::UPtr setResponse(
          //	static_cast<SettingsAPI::SetOrQueryResponse*>(response.release()));
          //parameters_fresh_ = parameters_->copy_from(*setResponse);
          parameters_fresh_ = parameters_->copy_from(static_cast<SettingsAPI::SetOrQueryResponse&>(*response));
        }
        
        // Return success = parameters refreshed OK
        return parameters_fresh_;
      }
      
      // If no response, we failed
      return false;
    }
    
    bool DeviceConfig::get_basic_device_info()
    {
      HTTPClient http_client(host_);
      std::stringstream device_info_stream;
      
      // get deviceInfo from sensor for calibration
      quanergy::qout << "[Debug] Attempting to get device info from " << host_ << std::endl;
      http_client.read(device_info_path_, device_info_stream);
      boost::property_tree::ptree device_info_tree;
      boost::property_tree::read_xml(device_info_stream, device_info_tree);
      
      // get model
      model_ = device_info_tree.get<std::string>("DeviceInfo.model");
      
      auto calibration_data = device_info_tree.get_child_optional("DeviceInfo.calibration");
      if (calibration_data)
      {
        // get encoder cal values
        amplitude_ = calibration_data->get_optional<double>("encoder.amplitude");
        phase_ = calibration_data->get_optional<double>("encoder.phase");
        
        // get vertical angles, if available
        auto laser_data = calibration_data->get_child_optional("lasers");
        if (laser_data)
        {
          vertical_angles_.resize(laser_data->get<unsigned int>("<xmlattr>.number"), 0.);
          
          for (auto& laser : *laser_data)
          {
            if (laser.first == "laser")
            {
              auto id = laser.second.get<unsigned int>("<xmlattr>.id");
              if (id < vertical_angles_.size())
              {
                vertical_angles_[id] = laser.second.get<double>("v");
              } // if id valid
              
            } // if laser
            
          } // for laser
          
        } // if laser data
        
      } // if cal data
      quanergy::qout << "[Debug] ... complete." << std::endl;
      
      // Try to discern what type of sensor this is and set some settings accordingly
      if (model_.rfind("M1", 0) == 0)
      {
        m_series_ = true;
        if (!vertical_angles_.empty())
          qerr << "[Critical] M1 sensor found with vertical angles but none are expected" << std::endl;
        vertical_angles_ = std::vector<double>(0.0);
      }
      else if (model_.rfind("M8", 0) == 0)
      {
        m_series_ = true;
        if (model_ == "M8-PRIME")
          supports_api_ = true;
          
        if (vertical_angles_.empty())
        {
          qout << "[Warning] No vertical angle calibration information available on sensor, proceeding with M8 defaults" << std::endl;
          
          vertical_angles_ = std::vector<double>(quanergy::client::M8_VERTICAL_ANGLES,
            quanergy::client::M8_VERTICAL_ANGLES + quanergy::client::M_SERIES_NUM_LASERS);
        }
      }
      else if (model_.rfind("MQ8", 0) == 0)
      {
        m_series_ = true;
        if (vertical_angles_.empty())
        {
          // all MQ sensors should have vertical angles
          //throw quanergy::client::InvalidVerticalAngles("MQ sensor found with no vertical angles on the sensor");
          qout << "[Warning] MQ sensor found with no vertical angles on the sensor" << std::endl;
          return false;
        }
      }
      else
      {
        qout << "[Warning] Model type: " << model_ << "Not recognized by DeviceConfig!" << std::endl;
        return false;
      }
      
      return true;
    }
    
    void DeviceConfig::refresh_parameters()
    {
      // Send a new query response to get new parameters
      if (auto response = send_request_get_response(
        std::make_unique<SettingsAPI::File>(SettingsAPI::Command::Query)
      ))
      {
        parameters_ = SettingsAPI::SetOrQueryResponse::UPtr(
          static_cast<SettingsAPI::SetOrQueryResponse*>(response.release()));
        parameters_fresh_ = true;
      }
    }
    
    SettingsAPI::File::UPtr 
      DeviceConfig::send_request_get_response(SettingsAPI::File::UPtr request)
    {
      using namespace SettingsAPI;
      
      // If the device does not support the api, we cannot use it!
      if (!supports_api_) return false;
      
      // If the socket did not open, we failed to connect, return false;
      if (!socket_.is_open())
      {
        qout << "[Warning] Attempting to get send request but TCP socket connection to host: "
          << host_ << " port: " << port_ << " is closed!" << std::endl;
        return nullptr; //error
      }
      
      // Form the request. 
      boost::asio::streambuf request_buf;
      std::ostream request_stream(&request_buf);
      
      // Get the request type for check and comparison
      Command requestType = request->type();
      
      // Can't send an Invalid command... so bail
      if (requestType == Command::Invalid)
      {
        qout << "[Warning] Command type was Invalid! Can't send an invalid request!" << std::endl;
        return nullptr;
      }
      
      // Convert the request to bytes
      request->to_stream(request_stream);
      
      // Send the command.
      boost::asio::write(socket_, request_buf);
      
      // Read the response 
      boost::asio::streambuf response_buf;
      boost::asio::read_until(socket_, response_buf, "</SettingsAPI>");
      
      // Check that response is OK. Make a stream to read from 
      std::istream response_stream(&response_buf);
      
      // Make a SettingsAPI::File to read the response into
      SettingsAPI::File::UPtr response = std::make_unique<SettingsAPI::File>();
      
      // Read the response from the stream
      if (!response->from_stream(response_stream))
      {
        qout << "[Warning] Failed to get " << to_string(requestType)
          << " response from stream!" << std::endl;
        return nullptr;
      }
      
      // Validate that the response received was the expected one
      if (!validate_reponse(*response, requestType))
      {
        qout << "[Warning] Response for " << to_string(requestType)
          << " command failed. See reason above." << std::endl;
        return nullptr;
      }
      
      return response;
    }
  }
}