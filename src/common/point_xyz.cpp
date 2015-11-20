/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/common/point_xyz.h>

#include <cmath>

namespace quanergy
{

PointXYZ operator+(PointXYZ const & lhs, float alpha)
{
  return PointXYZ(lhs.x+alpha, lhs.y+alpha, lhs.z+alpha);
}


PointXYZ operator-(PointXYZ const & lhs, float alpha)
{
  return PointXYZ(lhs.x-alpha, lhs.y-alpha, lhs.z-alpha);
}


PointXYZ operator*(PointXYZ const & lhs, float alpha)
{
  return PointXYZ(lhs.x*alpha, lhs.y*alpha, lhs.z*alpha);
}


PointXYZ operator/(PointXYZ const & lhs, float alpha)
{
  return PointXYZ(lhs.x/alpha, lhs.y/alpha, lhs.z/alpha);
}


PointXYZ operator+(PointXYZ const & lhs, PointXYZ const & rhs)
{
  return PointXYZ(lhs.x+rhs.x, lhs.y+rhs.y, lhs.z+rhs.z);
}


PointXYZ operator-(PointXYZ const & lhs, PointXYZ const & rhs)
{
  return PointXYZ(lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z);
}

PointXYZ operator-(PointXYZ const & p)
{
  return PointXYZ(-p.x, -p.y, -p.z);
}


float norm(PointXYZ const & p)
{
  return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


PointXYZ normalize(PointXYZ const & p)
{
  float norm = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);

  float s = 1.0f / norm;

  return p * s;
}


float squaredNorm(PointXYZ const & p)
{
  return (p.x*p.x + p.y*p.y + p.z*p.z);
}


float dot (PointXYZ const & a, PointXYZ const & b)
{
  return (a.x*b.x + a.y*b.y + a.z*b.z);
}


PointXYZ cross(PointXYZ const & a, PointXYZ const & b)
{
  return PointXYZ(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

} // namespace quanergy
