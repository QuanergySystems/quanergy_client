/****************************************************************************
 **
 ** Copyright (C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include "m8_client.h"
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <typeinfo>
#include <boost/circular_buffer.hpp>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

#define SHOW_FPS 0
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
  do \
{ \
  static unsigned count = 0;\
  static double last = getTime ();\
  double now = getTime (); \
  ++count; \
  if (now - last >= 1.0) \
{ \
  std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
  count = 0; \
  last = now; \
  } \
  }while(false)
#else
#define FPS_CALC(_WHAT_) \
  do \
{ \
  }while(false)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class SimpleM8Viewer
{
  public:
    typedef PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleM8Viewer (Grabber& grabber)
      : cloud_viewer_ (new PCLVisualizer ("PCL M8 Cloud"))
      , grabber_ (grabber)
      , clouds_ (25)
    {}

    void cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      clouds_.push_back (cloud);
    }

    void keyboard_callback (const KeyboardEvent& event, void* /*cookie*/)
    {
      if (event.keyUp ())
        return;
    }

    void
    mouse_callback (const MouseEvent& mouse_event,
                    void* /*cookie*/)
    {
      if (mouse_event.getType () == MouseEvent::MouseButtonPress &&
          mouse_event.getButton () == MouseEvent::LeftButton)
        cout << mouse_event.getX () << " , " << mouse_event.getY () << endl;
    }

    void
    run ()
    {
#if (PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 7 && PCL_REVISION_VERSION <= 2)
      cloud_viewer_->addCoordinateSystem (3.0);
#else
      cloud_viewer_->addCoordinateSystem (3.0, "global");
#endif
      cloud_viewer_->setBackgroundColor (0, 0, 0);
      cloud_viewer_->initCameraParameters ();
      cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
      cloud_viewer_->setCameraClipDistances (0.0, 50.0);
      boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
            &SimpleM8Viewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (
            cloud_cb);

      grabber_.start ();


      while (!cloud_viewer_->wasStopped ())
      {
        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())
        {
          if (!clouds_.empty()) {
            cloud_ = clouds_.front();
            clouds_.pop_front();
          }
          cloud_mutex_.unlock ();
        }

        if (cloud_)
        {
          FPS_CALC("drawing cloud");
          PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler (cloud_,"intensity");

          if (!cloud_viewer_->updatePointCloud<pcl::PointXYZI> (cloud_, color_handler, "M8"))
            cloud_viewer_->addPointCloud<pcl::PointXYZI> (cloud_, "M8");

          cloud_viewer_->spinOnce ();

          cloud_.reset();
        }

        if (!grabber_.isRunning ())
          cloud_viewer_->spin ();

        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();

      cloud_connection.disconnect ();
    }

    boost::shared_ptr<PCLVisualizer> cloud_viewer_;

    Grabber& grabber_;
    boost::mutex cloud_mutex_;

    CloudConstPtr cloud_;
    boost::circular_buffer<CloudConstPtr> clouds_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0]
      << " -ip 10.0.0.2 -port 1234 [-h | --help] [-color N/I/Z]" << std::endl
      << "-ip IP address of the sensor  10.0.0.2[default]" << std::endl
      << "-port TCP port used by the sensor 4141[default]" << std::endl
      << "-range minimum range threshold for beam 0" << std::endl
      << "-intensity minimum intensity threshold for beam 0" << std::endl;
  std::cout << "\t-h | --help : shows this help and exit" << std::endl;
  return;
}

int
main (int argc, char ** argv)
{
  if (argc < 2 || find_switch (argc, argv, "-h") || find_switch (argc, argv, "--help"))
  {
    usage (argv);
    return (0);
  }

  std::string ip ("10.0.0.2");
  int port = 4141;
  double range = 0;
  double intensity = 0;

  parse_argument (argc, argv, "-ip", ip);
  parse_argument (argc, argv, "-port", port);
  parse_argument (argc, argv, "-range", range);
  parse_argument (argc, argv, "-intensity", intensity);

  M8Client* grabber = new M8Client(boost::asio::ip::address::from_string(ip), port);

  float ranges[8] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  grabber->setRingFilterMinimumRangeThresholds(ranges);
  grabber->setRingFilterMinimumRangeThreshold(0, range);
  unsigned char intensities[8] = {1, 2, 2, 3, 3, 4, 2, 1};
  grabber->setRingFilterMinimumIntensityThresholds(intensities);
  grabber->setRingFilterMinimumIntensityThreshold(0, intensity);

  SimpleM8Viewer v (*grabber);
  v.run ();

  return (0);
}
