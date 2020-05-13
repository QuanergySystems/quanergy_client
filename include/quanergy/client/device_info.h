/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <boost/optional.hpp>

#include <vector>

namespace quanergy
{
  namespace client
  {
    /** \brief DeviceInfo provides a facility for retrieving device info information from Quanergy sensors.
     */
    class DeviceInfo
    {
    public:
      /** \brief The constructor connects to the sensor and retrieves the information.
       *  \param host is the hostname or IP address of the sensor
       */
      DeviceInfo(const std::string& host);

      /// \brief get the model; empty string indicates an error loading the device info */
      const std::string& model() const { return model_; }

      /// \brief get the amplitude if it was available in the device info
      const boost::optional<double>& amplitude() const { return amplitude_; }

      /// \brief get the encoder calibration phase if it was available in the device info
      const boost::optional<double>& phase() const { return phase_; }

      /// \brief get the vertical angles; empty means they were not available
      const std::vector<double>& verticalAngles() const { return vertical_angles_; }

    private:
      const std::string device_info_path_ {"/PSIA/System/deviceInfo"};

      std::string model_;
      boost::optional<double> amplitude_;
      boost::optional<double> phase_;
      std::vector<double> vertical_angles_;
    };
  }
}

