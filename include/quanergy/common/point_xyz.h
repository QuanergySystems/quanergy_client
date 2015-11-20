/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file
 *
 *  Point Cloud Library point structures for cartesian data.
 *
 */

#ifndef QUANERGY_COMMON_POINT_XYZ_H
#define QUANERGY_COMMON_POINT_XYZ_H

#include <ostream>

#include <pcl/point_types.h>


namespace quanergy
{
  struct EIGEN_ALIGN16 PointXYZ
  {
    PCL_ADD_POINT4D;

    PointXYZ (const PointXYZ& p) 
      : x(p.x)
      , y(p.y)
      , z(p.z)
    {
      data[3] = 1.f;
    }

    PointXYZ ()
      : x(0.0f)
      , y(0.0f)
      , z(0.0f)
    {
      data[3] = 1.f;
    }

    PointXYZ (float _x, float _y, float _z)
      : x(_x)
      , y(_y)   
      , z(_z)
    {
      data[3] = 1.f;
    }

    inline float const & operator[](std::size_t index) const
    {
      return data[index];
    }

    inline float & operator[](std::size_t index)
    {
      return data[index];
    }

    operator Eigen::Vector3f() 
    {
      return Eigen::Vector3f(x, y, z);
    }

    friend std::ostream& operator<< (std::ostream &out, const PointXYZ &point);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PointXYZ operator+(PointXYZ const &, float);
  PointXYZ operator-(PointXYZ const &, float);
  PointXYZ operator*(PointXYZ const &, float);
  PointXYZ operator/(PointXYZ const &, float);
  PointXYZ operator+(PointXYZ const &, PointXYZ const &);
  PointXYZ operator-(PointXYZ const &, PointXYZ const &);

  PointXYZ operator-(PointXYZ const &);

  float norm(PointXYZ const & p);        // magnitude
  //! if norm is zero, this give invalid value
  PointXYZ normalize(PointXYZ const &);
  float squaredNorm(PointXYZ const &);
  float dot(PointXYZ const &, PointXYZ const &);
  PointXYZ cross(PointXYZ const &, PointXYZ const &);

} // namespace quanergy


inline std::ostream& operator<< (std::ostream &out, const quanergy::PointXYZ &point)
{
  out << point.x << " " << point.y << " " << point.z;
  return out;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(quanergy::PointXYZ,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z))


#endif
