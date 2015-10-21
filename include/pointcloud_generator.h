/** \file pointcloud_generator.h
  * \brief provide base pointcloud generator functionality
  *
  * Individual message types will need to specialize the functionality provided here.
  */

#ifndef QUANERGY_POINTCLOUD_GENERATOR_H
#define QUANERGY_POINTCLOUD_GENERATOR_H

#include "deserialize.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/signals2.hpp>

namespace quanergy
{
  struct PointCloudGeneratorBase
  {
    /// The output is a PCL point cloud of type pcl::PointXYZI
    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

    /// signal type
    typedef boost::signals2::signal<void (const PointCloud::ConstPtr&)> CloudSignal;

    virtual void setCloudSignal(std::shared_ptr<CloudSignal> cloud_signal)
    {
      cloud_signal_ = cloud_signal;
    }

  protected:
    /// signal that gets fired whenever a cloud is ready
    std::shared_ptr<CloudSignal> cloud_signal_;
  };

  /** \brief main PointCloudGenerator template recursively defines the type lookup*/
  template <class T, class... OtherTypes>
  struct PointCloudGenerator : public PointCloudGeneratorBase
  {
    virtual void setCloudSignal(std::shared_ptr<CloudSignal> cloud_signal)
    {
      t_gen_.setCloudSignal(cloud_signal);
      o_gen_.setCloudSignal(cloud_signal);
    }

    inline void toPointCloud(std::uint8_t type, const std::vector<char>& packet)
    {
      if (t_gen_.match(type))
        t_gen_.toPointCloud(type, packet);
      else
        o_gen_.toPointCloud(type, packet);
    }

  private:
    PointCloudGenerator<T> t_gen_;
    PointCloudGenerator<OtherTypes...> o_gen_;
  };

  /** \brief forces specialization of any single type TypeFinder */
  template <class T>
  struct PointCloudGenerator<T> : public PointCloudGeneratorBase
  {
    static bool match(std::uint8_t type) = delete;
    inline void toPointCloud(std::uint8_t type, const std::vector<char>& packet) = delete;
  };
}
#endif
