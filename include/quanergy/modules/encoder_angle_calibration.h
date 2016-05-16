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
#include <future>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

#include <boost/signals2.hpp>

#include <quanergy/common/point_hvdir.h>
#include <quanergy/common/pointcloud_types.h>
#include <quanergy/common/angle.h>

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

      /** Moving average period to use when smoothing error signal, in encoder
       * counts */
      static const int MOV_AVG_PERIOD;

      /** Tolerance for identifying when phase values have converged without
       * outliers */
      static const double PHASE_CONVERGENCE_THRESHOLD;

      /** Amplitude threshold dictating if calculating the encoder offset is
       * appropriate. If an encoder calibration returns an amplitude below this
       * value, this class indicates that calibration is complete and applies no
       * calibration to outgoing points. */
      static const double AMPLITUDE_THRESHOLD;

    public:

      /** 
       * @brief Constructor
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
       * @param[in] subscriber Subscriber to be called.
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
       * @param[in] pc Point cloud to be processed.
       */
      void slot(PointCloudHVDIRPtr const & pc);

      /** 
       * @brief Sets this class to only calculate the error parameters and not
       * apply the calibration. This mode is for when the caller wants to look
       * at multiple calculations for the amplitude and phase.
       */
      void calibrateOnly();

      /**
       * @brief Sets number of valid calibrations to be collected before
       *averaging. Validity is determined by change in phase between current
       *sample and previous sample
       *
       * @param[in] num_cals Number of calibrations
       */
      void setRequiredNumSamples(double num_cals);

      /** 
       * @brief Function to manually set calibration parameters. Calling this
       * function will disable the automatic calibration and subsequent calls to
       * slot will apply the calibration and call the subscriber.
       * 
       * @param[in] amplitude Amplitude of sinusoid error
       * @param[in] phase Phase of sinusoidal error
       */
      void setParams(double amplitude, double phase);

      /** 
       * @brief Function to set frame rate. Value is in frames per second. If
       * not set, the default is 10.
       * 
       * @param[in] frame_rate frame rate.
       */
      void setFrameRate(double frame_rate);

      /**
       * @brief Sets timeout for calculating the calibration. If the timeout
       * expires this class will throw an exception.
       *
       * @param[in] timeout Timeout.
       */
      template<typename Rep, typename Period>
      void setTimeout(const std::chrono::duration<Rep,Period>& timeout);

      /**
       * @brief Function to calculate the sinusoidal error of the horizontal
       * encoder values. This function is called once a full revolution of the
       * encoder is captured.
       *
       * @param[in] encoder_angles Encoder angles
       *
       * @return Tuple with first element as the amplitude of the sinusoid and
       * second element as the phase offset of the sinusoid.
       */
      SineParameters calculate(const std::vector<double>& encoder_angles);

    private:
      /**
       * @brief Function to create line representing the expected encoder
       * values. Unwrapped encoder values should be linear with respect to 
       * time-index. This line is used to determine the error.
       *
       * @param[in] encoder_angles Angles to fit line to
       * @returns slope of the line
       */
      static double fitLine(const AngleContainer& encoder_angles);

      /**
       * @brief Function to determine sinusoidal parameters from a signal. Once
       * we've created a sinusoid of the encoder error, we use this function to
       * determine the parameters of the sinusoid.
       *
       * @param[in] sinusoid_values Sinusoid signal
       * @param[in] clockwise Whether the motor is spinning clockwise.
       * If false, motor is spinning counter-clockwise.
       *
       * @return tuple where first element is amplitude of sinusoid and second
       * elemement is phase offset of sinusoid.
       */
      static SineParameters findSinusoidParameters(const AngleContainer& sine_signal, bool clockwise);

      /**
       * @brief Translate angle values so they are *not* contained within -pi
       *and
       * pi.
       *
       * @param[in] encoder_angles Encoder angles to be unwrapped.
       * @returns Unwrapped encoder angles
       */
      static AngleContainer unwrapEncoderAngles(const AngleContainer& encoder_angles);

      /** 
       * @brief Applies moving average in place
       * 
       * @param[inout] encoder_angles Signal to be filtered.
       * @param[in] period Period to apply averaging over, in container elements
       *
       */
      static AngleContainer movingAvgFilter(const AngleContainer& encoder_angles, int period);

      /** 
       * @brief Function to check if hvdir_pts_ has any gaps due to dropped
       * packets
       * 
       * @return True if HVDIR points are complelte with no dropped, false
       * otherwise. 
       */
      bool checkComplete() const;

      /** 
       * @brief Function to process encoder angles in thread pool
       */
      void processAngles();

      /** 
       * @brief Applies calibration. Applies calibration in place.
       * 
       * @param[in] cloud_ptr Point cloud calibration will be applied to.
       */
      void applyCalibration(PointCloudHVDIRPtr const & cloud_ptr) const;

      /** Signal object to notify next slot */
      Signal signal_;

      /** HVDIR points to be used for calibration */
      PointCloudHVDIR hvdir_pts_;

      /** thread pool futures */
      std::vector<std::future<void>> futures_;

      /** Mutex for period_queue_ */
      mutable std::mutex queue_mutex_;

      /** Condition variable for waiting on empty queue_mutex_ */
      std::condition_variable nonempty_condition_;

      /** queue for encoder angles vectors to be stored. Each element in the
       * queue contains a vector of encoder angles for a full period */
      std::queue<AngleContainer> period_queue_;

      /** Flag indicating that we've started a full revolution of h angles from
       * -pi to pi */
      std::atomic<bool> started_full_rev_;

      /** Flag indicating calibration is complete. Used to withstand spurious
       * wakeups from condition variable */
      std::atomic<bool> calibration_complete_;

      /** Calculated amplitude */
      double amplitude_ = 0.;

      /** Calculated phase (radians) */
      double phase_ = 0.;

      /** Frame rate of M8 sensor */
      double frame_rate_ = 10.;

      /** Flag indicating that we're outputting calibration results constantly
       * and never applying calibration. This mode is used when the user wants
       * to analyze the calibration results */
      bool run_forever_ = false;

      /** mutex around writing to file */
      mutable std::mutex file_mutex_;

      /** number of encoder calibrations to run before before averaging
       * amplitude and phase values and reporting these to user */
      std::atomic<int> required_samples_;

      /** number of calibrations which have currently been processed. Used to
       * check against required_samples_ */
      std::atomic<int> num_valid_samples_;

      /** mutex for containers holding amplitude_values_ and phase_values_ */
      mutable std::mutex container_mutex_;

      /** Vector to hold amplitude values. This is added to as valid calibration
       * samples are calculated and used to eventually calculate the average */
      std::vector<double> amplitude_values_;

      /** Special averaging container to calculate average of phase angles and
       * handle wrapping. */
      quanergy::common::AngleAverager<double> phase_averager_;

      /** Last phase angle processed by thread pool */
      double last_phase_ = 0.;

      /** Flag indicating calibration has started */
      bool started_calibration_ = false;

      /** time when calibration started */
      std::chrono::time_point<std::chrono::system_clock> time_started_;

      /** Timeout to collect enough calibrations. Defaulted is 60 seconds */
      std::chrono::seconds timeout_ = std::chrono::seconds(60);

      /** flag indicating whether or not the first calibration has been
       * performed. This flag gets set to false after the first run */
      std::atomic<bool> first_run_;

      /** Number of calibrations which have occurred */
      std::atomic<int> calibration_count_;

    };

  } /* calibration */
} /* quanergy */

#endif /* ENCODER_ANGLE_CALIBRATION_H_ */
