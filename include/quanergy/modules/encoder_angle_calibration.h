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
       * 
       * @param run_forever [in] Specifies whether this module should output
       * calibration results for its lifetime rather than apply calibration
       * after a successful calibration is found.
       */
      EncoderAngleCalibration(bool run_forever = false);

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
       * @brief Sets number of valid calibrations to be collected before
       *averaging. Validity is determined by change in phase between current
       *sample and previous sample
       *
       * @param num_cals[in] Number of calibrations
       */
      void setRequiredNumSamples(double num_cals);

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
       * @brief Sets timeout for calculating calibration.
       * 
       * @param[in] timeout Timeout in seconds.
       */
      void setTimeout(int timeout);

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
       * @brief Function to process encoder angles in thread pool
       */
      void processAngles();

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
       * @brief Applies moving average in place
       * 
       * @param encoder_angles[inout] Signal to be filtered.
       * @param period[in] Period to apply averaging over, in container elements
       *
       */
      AngleContainer movingAvgFilter(const AngleContainer& encoder_angles, int period);

      /** 
       * @brief Flag indicating this class will output debugging output.
       */
      bool debugging_ = false;

      /** Signal object to notify next slot */
      Signal signal_;

      /** HVDIR points to be used for calibration */
      PointCloudHVDIR hvdir_pts_;

      /** thread to calibrate encoder error */
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

      /** Flag indicating that we're outputting calibration results constantly
       * and never applying calibration. This mode is used when the user wants
       * to analyze the calibration results */
      const bool run_forever_ = false;

      /** mutex around writing to file */
      mutable std::mutex file_mutex_;

      /** number of encoder calibrations to run before before averaging
       * amplitude and phase values and reporting these to user */
      std::atomic_int required_samples_;

      /** number of calibrations which have current been processed. Used to
       * check against total_cal_samples_ */
      std::atomic_int num_valid_samples_;

      /** mutex for containers holding amplitude_values_ and phase_values_ */
      mutable std::mutex container_mutex_;

      /** Vector to hold amplitude values. This is added to as valid calibration
       * samples are calculated and used to eventually calculate the average */
      std::vector<double> amplitude_values_;

      /** Vector to hold phase values. This is added to as valid calibration
       * samples are calculated and used to eventually calculate the average */
      std::vector<double> phase_values_;

      /** Flag indicating calibration has started */
      bool started_calibration_ = false;

      /** time when calibration started */
      std::chrono::time_point<std::chrono::system_clock> time_started_;

      /** Timeout in seconds */
      int timeout_ = 60;

    };

  } /* calibration */
} /* quanergy */

#endif /* ENCODER_ANGLE_CALIBRATION_H_ */
