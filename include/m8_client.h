/****************************************************************************
 **
 ** Copyright (C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/
#ifndef M8_CLIENT_H
#define M8_CLIENT_H

#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>

/** Class M8Client used to connect to Quanergy Systems M8 LiDAR
  * Implements pcl::Grabber
  *
  * \note M8 A samples may display a ring of low intensity data near the origin that is not
  * valid data. This class provides a ring filter to remove that ring if it is present. The
  * ring filter range and intensity thresholds are combined to filter out only points that
  * are both within the range and of lower intensity.
  */
class M8Client : public pcl::Grabber, private boost::noncopyable
{
  public:
    /// The output is a PCL point cloud of type pcl::PointXYZI
    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
    /// Const shared pointer
    typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
    /// Shared pointer
    typedef boost::shared_ptr<PointCloud> PointCloudPtr;

    /** \brief Signal calllback used for a 360 degree cloud
      * Represents multiple corrected packets from the Quanergy M8.
      */
    typedef void (cloud_signal_callback) (const PointCloudConstPtr&);

    /** \brief Constructor taking a specified IP/port.
      * \param[in] ip IP Address that should be used to listen for M8 packets
      * \param[in] port TCP Port that should be used to listen for M8 packets
      */
    M8Client (const boost::asio::ip::address& ip, const unsigned short port = M8_DATA_PORT);

    /** \brief virtual Destructor inherited from the pcl::Grabber interface. It never throws. */
    virtual ~M8Client () throw ();

    /** \brief Starts processing the Quanergy M8 packets, either from the network or PCAP file. */
    virtual void start ();

    /** \brief Stops processing the Quanergy M8 packets, either from the network or PCAP file */
    virtual void stop ();

    /** \brief Obtains the name of this I/O grabber
     *  \return The name of the grabber
     */
    virtual std::string getName () const;

    /** \brief Check if the grabber is still running.
     *  \return TRUE if the grabber is running, FALSE otherwise
     */
    virtual bool isRunning () const;

    /** \brief Returns the number of frames per second.
     */
    virtual float getFramesPerSecond () const;

    /** \brief For ring filtering: Returns the minimum range filter threshold for the given beam, in meters */
    float getRingFilterMinimumRangeThreshold (const unsigned int laser_beam) const;

    /** \brief For ring filtering: Set the minimum range filter threshold for the given beam, in meters
      * This value is in meters. Defaults to 1.0
      */
    void setRingFilterMinimumRangeThreshold (const unsigned int laser_beam, const float min_threshold);

    /** \brief For ring filtering: Returns the minimum intensity filter threshold for the given beam */
    unsigned char getRingFilterMinimumIntensityThreshold (const unsigned int laser_beam) const;

    /** \brief For ring filtering: Set the minimum intensity filter threshold for the given beam, in meters
      * This value is an integer between 0-255. Defaults to 0
      */
    void setRingFilterMinimumIntensityThreshold (const unsigned int laser_beam, const unsigned char min_threshold);

    /** \brief For ring filtering: Gets the minimum range filter thresholds for all 8 beams, in meters */
    void getRingFilterMinimumRangeThresholds (float min_threshold[8]) const;

    /** \brief For ring filtering: Sets the minimum range filter thresholds for all 8 beams, in meters */
    void setRingFilterMinimumRangeThresholds (const float min_threshold[8]);

    /** \brief For ring filtering: Gets the minimum intensity filter thresholds for all 8 beams, in meters */
    void getRingFilterMinimumIntensityThresholds (unsigned char min_threshold[8]) const;

    /** \brief For ring filtering: Sets the minimum intensity filter thresholds for all 8 beams, in meters */
    void setRingFilterMinimumIntensityThresholds (const unsigned char min_threshold[8]);

  private:
    /// Default TCP port for the M8 sensor
    static const int M8_DATA_PORT = 4141;
    /// Default angles
    static const int M8_NUM_ROT_ANGLES = 10400;
    /// Default number of firings per TCP packet
    static const int M8_FIRING_PER_PKT = 50;
    /// Size of TCP packet
    static const int M8_PACKET_BYTES = 6612;
    /// Ultimately M8 would be a multiecho LiDAR, for now only the first echo is available
    static const int M8_NUM_RETURNS = 3;
    /// The total number of lasers on the M8 Sensor
    static const int M8_NUM_LASERS = 8;
    /// Vertical angles
    static const double M8_VERTICAL_ANGLES[];

#pragma pack(push, 1)
      /// \brief structure that holds the sensor firing output
      struct M8FiringData
      {
        std::uint16_t position;
        std::uint16_t padding;
        std::uint32_t returns_distances[M8_NUM_RETURNS][M8_NUM_LASERS];   // 1 cm resolution.
        std::uint8_t  returns_intensities[M8_NUM_RETURNS][M8_NUM_LASERS]; // 255 indicates saturation
        std::uint8_t  returns_status[M8_NUM_LASERS];                      // 0 for now
      }; // 132 bytes

      /// \brief structure that holds multiple sensor firings and gets sent in the TCP packet
      struct M8DataPacket
      {
        M8FiringData  data[M8_FIRING_PER_PKT];
        std::uint32_t seconds;     // seconds from Jan 1 1970
        std::uint32_t nanoseconds; // fractional seconds turned to nanoseconds
        std::uint16_t version;     // API version number
        std::uint16_t status;      // 0: good, 1: Sensor SW/FW mismatch
      }; // 6612 bytes
#pragma pack(pop)

    /// function used as a callback for the thread that enqueus encoming data in the queue
    void enqueueM8Packet (const unsigned char *data,
                         const std::size_t& bytes_received);
    /// function used as a callback for the socket reading thread
    void read ();
    /// processes the TCP packets
    void processM8Packets ();
    /// transposes the point cloud
    void organizeCloud (PointCloudPtr &current_xyzi);
    /// converts TCP packets to PCL point clouds
    void toPointClouds (M8DataPacket *data_packet);
    /// Fire current cloud
    void fireCurrentCloud ();
    /** Convert from range and angles to cartesian
      * \param[in] range range in meter
      * \param[in] cos_hz_angle cosine of horizontal angle
      * \param[in] sin_hz_angle sine of horizontal angle
      * \param[in] cos_vt_angle cosine of vertical angle
      * \param[in] sin_vt_angle sine of vertical angle
      * \param[out] point point in cartesian coordinates
      */
    void computeXYZ (const double range,
                     const double cos_hz_angle, const double sin_hz_angle,
                     const double cos_vt_angle, const double sin_vt_angle,
                     pcl::PointXYZI& point);
    /// sensor IP address
    boost::asio::ip::address ip_address_;
    /// TCP port
    unsigned int port_;
    /// TCP socket to the sensor, using for reading
    boost::asio::ip::tcp::socket *read_socket_;
    /// TCP socket reading service
    boost::asio::io_service read_socket_service_;
    /// TCP end point
    boost::asio::ip::tcp::endpoint tcp_listener_endpoint_;
    /// SynchronizedQueue is a thread-safe access queue
    pcl::SynchronizedQueue<unsigned char *> data_queue_;
    /// lookup table for cosinus
    std::vector<double> cos_lookup_table_;
    /// lookup table for sinus
    std::vector<double> sin_lookup_table_;
    double cos_vertical_angles_[M8_NUM_LASERS];
    double sin_vertical_angles_[M8_NUM_LASERS];
    /// queue consuming thread
    boost::thread *queue_consumer_thread_;
    /// packet reading thread
    boost::thread *read_packet_thread_;
    /// termination condition
    bool terminate_read_packet_thread_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > current_cloud_;
    /// signal that gets fired whenever we collect a scan
    boost::signals2::signal<cloud_signal_callback>* cloud_signal_;
    /// last accounted for azimuth angle
    double last_azimuth_;
    /// cloud height
    int cloud_height_;
    /// number of dropped packets
    int dropped_packets_;
    /// global packet counter
    uint32_t packet_counter_;
    /// global cloud counter
    uint32_t cloud_counter_;
    /// minimum range threshold
    float min_range_threshold_;
    /// maximum range threshold
    float max_range_threshold_;
    /// For ring filter: range filter thresholds
    float ring_filter_range_[M8_NUM_LASERS];
    /// For ring filter: intensity filter thresholds
    unsigned char ring_filter_intensity_[M8_NUM_LASERS];
    /// number of proccessed frames over the previous second
    int frames_per_second_;
    /// current moment timestamp
    boost::chrono::high_resolution_clock::time_point curr_time_;
    /// previous second timestamp
    boost::chrono::high_resolution_clock::time_point prev_time_;
    /// duration of time between current time and previous second
    boost::chrono::duration<double> time_span_;
};

#endif // M8_CLIENT_H
