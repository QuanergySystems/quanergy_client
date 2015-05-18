/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, The MITRE Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Nizar Sallem <nizar.sallem@quanergy.com>
 */
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
template<typename PointType>
class SimpleM8Viewer
{
  public:
    typedef PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleM8Viewer (Grabber& grabber,
                    PointCloudColorHandler<PointType>* handler = NULL)
      : cloud_viewer_ (new PCLVisualizer ("PCL M8 Cloud"))
      , grabber_ (grabber)
      , clouds_ (25)
      , handler_ (handler)
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

      if (handler_) 
      {
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
            handler_->setInputCloud (cloud_);
            if (!cloud_viewer_->updatePointCloud (cloud_, *handler_, "M8"))
              cloud_viewer_->addPointCloud (cloud_, *handler_, "M8");

            cloud_viewer_->spinOnce ();

            cloud_.reset();
          }

          if (!grabber_.isRunning ())
            cloud_viewer_->spin ();

          boost::this_thread::sleep (boost::posix_time::microseconds (100));
        }
      } 
      else 
      {
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
            if (!cloud_viewer_->updatePointCloud<pcl::PointXYZI> (cloud_, "M8"))
              cloud_viewer_->addPointCloud<pcl::PointXYZI> (cloud_, "M8");

            cloud_viewer_->spinOnce ();

            cloud_.reset();
          }

          if (!grabber_.isRunning ())
            cloud_viewer_->spin ();

          boost::this_thread::sleep (boost::posix_time::microseconds (100));
        }
      }
      grabber_.stop ();

      cloud_connection.disconnect ();
    }

    boost::shared_ptr<PCLVisualizer> cloud_viewer_;

    Grabber& grabber_;
    boost::mutex cloud_mutex_;

    CloudConstPtr cloud_;
    boost::circular_buffer<CloudConstPtr> clouds_;
    PointCloudColorHandler<PointType> *handler_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0]
	    << " -ip 10.0.0.2 -port 1234 [-h | --help] [-color N/I/Z]" << std::endl
	    << "-ip IP address of the sensor  10.0.0.2[default]" << std::endl
	    << "-port TCP port used by the sensor 4141[default]" << std::endl
	    << "-color specifies the color handler" << std::endl
    	    << "\t N: solid color[default]" << std::endl
	    << "\t I: intensity field is colored" << std::endl
            << "\t Z: z field is colored" << std::endl;
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

  char color ('N');
  std::string ip ("10.0.0.2");
  int port = 4141;

  parse_argument (argc, argv, "-ip", ip);
  parse_argument (argc, argv, "-port", port);
  parse_argument (argc, argv, "-color", color);

  M8Client* grabber = new M8Client(boost::asio::ip::address::from_string(ip), port);

  std::cout << "viewer coloring: ";
  switch (color)
  {
    case 'Z':
    {
      std::cout << "Z axis" << std::endl;
      PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("z");
      
      SimpleM8Viewer<pcl::PointXYZI> v (*grabber, &color_handler);
      v.run ();
    } break;
    case 'I':
    {
      std::cout << "intensity field" << std::endl;
      PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");
      
      SimpleM8Viewer<pcl::PointXYZI> v (*grabber, &color_handler);
      v.run ();
    } break;
    case 'N':
    {
      std::cout << "solid color" << std::endl;
      SimpleM8Viewer<pcl::PointXYZI> v (*grabber);
      v.run ();
    } break;
  }

  return (0);
}
