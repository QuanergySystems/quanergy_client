/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef ENCODER_ANGLE_CALIBRATION_H_
#define ENCODER_ANGLE_CALIBRATION_H_

#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include <boost/signals2.hpp>

#include <quanergy/common/point_hvdir.h>
#include <quanergy/common/pointcloud_types.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
  #define TYPENAME
#else
  #define DLLEXPORT
  #define TYPENAME typename
#endif

namespace quanergy
{
  namespace calibration
  {
    /**
     * @brief This class calculates the error in the encoder angles and returns
     * the amplitude and phase shift of the sine function modeling the error in
     * the encoder angles for the M8 Sensor. This class can also be provided
     * with the sine error parameters and apply the calibration to incoming
     * points.
     */
    struct EncoderAngleCalibration
    {
      /** The type of the container that contains the encoder
       * angles */
      using AngleContainer = std::vector<double>;

      /** This type holds the parameters of the sine wave.
       * The amplitude is the first element of the pair,
       * the phase is the second element. */
      using SineParameters = std::pair<double, double>;

      /** This type holds the parameters of a 2d line */
      using LineParameters = std::pair<double, double>;

      /** 
       * @brief Short-hand for point types
       */
      using Ptr = std::shared_ptr<EncoderAngleCalibration>;

      /** 
       * @brief Result type for class
       */
      using ResultType = PointCloudHVDIRPtr;

      /** 
       * @brief Signal type
       */
      using Signal =  boost::signals2::signal<void (const ResultType&)>;

      /** The firing rate of the LiDAR, in Hz */
      static const double FIRING_RATE;

      /** Maximum number of radians allowed between subsequent encoder angles.
       * This value determines if we've the sensor has dropped a packet */
      static const double RADIANS_PER_ENCODER_COUNT;

      /** Once the motor has reached stead-state, the number of encoder counts per
       * revolution should be roughly the firing rate divided by the frame rate.
       * This number is how many counts the current revolution can be within the
       * theoretical steady-state number of encoder counts. */
      static const int ENCODER_COUNT_TOLERANCE;

      /** Minimum number of encoder angles to qualify full revolution at
       * steady-state motor speed */
      static const int MIN_ENCODER_ANGLES_PER_REV;

      /** Maximum number of encoder angles to qualify full revolution at
       * steady-state motor speed */
      static const int MAX_ENCODER_ANGLES_PER_REV;

      /** Allowable tolerance within pi for an endpoint-angle in a revolution to
       * be considered near pi */
      static const double PI_TOLERANCE;

    public:

      /**
       * @brief Empty constructor
       */
      EncoderAngleCalibration();

      /**
       * @brief Empty destructor
       */
      ~EncoderAngleCalibration();

      /** 
       * @brief Adds subscriber to be called after this classes functionality is
       * done.
       * 
       * @param subscriber[in] Subscriber to be called.
       * 
       * @return connection between this class and subscriber
       */
      boost::signals2::connection connect(const TYPENAME Signal::slot_type& subscriber);

      /** 
       * @brief Slot to be connected as a subscriber to another process. If
       * calibration is not complete, this function will add the point cloud
       * argument to the cloud to be used for calibration. If calibration is
       * complete, this function will apply the calibration and call the next
       * subscriber.
       * 
       * @param pc[in] Point cloud to be processed.
       */
      void slot(PointCloudHVDIRPtr const & pc);

      /** 
       * @brief Function to manually set calibration parameters. Calling this
       * function will disable the automatic calibration and subsequent calls to
       * slot will apply the calibration and call the subscriber.
       * 
       * @param amplitude[in] Amplitude of sinusoid error
       * @param phase[in] Phase of sinusoidal error
       */
      void setParams(double amplitude, double phase);

      /** 
       * @brief Function to set frame rate. Value is in frames per second. If
       * not set, the default is 10.
       * 
       * @param frame_rate[in] frame rate.
       */
      void setFrameRate(double frame_rate);

      /**
       * @brief Function to calculate the sinusoidal error of the horizontal
       * encoder values. This function is called once a full revolution of the
       * encoder is captured.
       *
       * @param encoder_angles[in] Encoder angles
       * @param debugging[in] Flag enabling debugging output
       *
       * @return Tuple with first element as the amplitude of the sinusoid and
       * second element as the phase offset of the sinusoid.
       */
      SineParameters calculate(const std::vector<double>& encoder_angles,
                               bool debugging = false);

      /**
       * @brief Function to create line representing the expected encoder
       * values. Unwrapped encoder values should be linear with respect to 
       * time-index. This line is used to determine the error.
       *
       * @param encoder_angles Angles to fit line to
       * @returns slope of the line
       */
      double fitLine(const AngleContainer& encoder_angles);

      /**
       * @brief Function to determine sinusoidal parameters from a signal. Once
       * we've created a sinusoid of the encoder error, we use this function to
       * determine the parameters of the sinusoid.
       *
       * @param sinusoid_values[in] Sinusoid signal
       *
       * @return tuple where first element is amplitude of sinusoid and second
       * elemement is phase offset of sinusoid.
       */
      SineParameters findSinusoidParameters(const AngleContainer& sine_signal);

    private:

      /** 
       * @brief Function to check if hvdir_pts_ has any gaps due to dropped
       * packets
       * 
       * @return True if HVDIR points are complelte with no dropped, false
       * otherwise. 
       */
      bool checkComplete();

      /** 
       * @brief Function to process encoder angles in separate thread 
       * 
       * @param encoder_angles[in] Angles to be processed
       */
      void processAngles(AngleContainer encoder_angles);

      /**
       * @brief Translate angle values so they are *not* contained within -pi
       *and
       * pi.
       *
       * @param encoder_angles[in] Encoder angles to be unwrapped.
       * @returns Unwrapped encoder angles
       */
      AngleContainer unwrapEncoderAngles(const AngleContainer& encoder_angles);

      /** 
       * @brief Applies calibration. Applies calibration in place.
       * 
       * @param cloud_ptr[in] Point cloud calibration will be applied to.
       */
      void applyCalibration(PointCloudHVDIRPtr const & cloud_ptr);

      /** 
       * @brief Flag indicating this class will output debugging output.
       */
      bool debugging_ = false;

      /** Signal object to notify next slot */
      Signal signal_;

      /** HVDIR points to be used for calibration */
      PointCloudHVDIR hvdir_pts_;

      /** thread to calibrate encoder error */
      std::thread processing_thread_;

      /** Flag indicating that we've started a full revolution of h angles from
       * -pi to pi */
      std::atomic_bool started_full_rev_;

      /** Flag indicating calibration is complete. Used to withstand spurious
       * wakeups from condition variable */
      std::atomic_bool calibration_complete_;

      /** Calculated amplitude */
      double amplitude_ = 0.;

      /** Calculated phase (radians) */
      double phase_ = 0.;

      /** Frame rate of M8 sensor */
      double frame_rate_ = 10.;

    };

  } /* calibration */
} /* quanergy */

#endif /* ENCODER_ANGLE_CALIBRATION_H_ */
