/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file
 *
 *  Point Cloud Library point structures for polar data with intensity and ring.
 *
 */

#ifndef QUANERGY_COMMON_POINT_HVDIR_H
#define QUANERGY_COMMON_POINT_HVDIR_H

#include <pcl/point_types.h>

namespace quanergy
{

#if (PCL_MAJOR_VERSION==1 && PCL_MINOR_VERSION == 7 && PCL_REVISION_VERSION < 2)
  #define PCL_ADD_EIGEN_MAPS_POINT4D \
  inline Eigen::Map<Eigen::Vector3f> getVector3fMap () { return (Eigen::Vector3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap () const { return (Eigen::Vector3f::Map (data)); } \
  inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap () { return (Eigen::Vector4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4fMap () const { return (Eigen::Vector4f::MapAligned (data)); } \
  inline Eigen::Map<Eigen::Array3f> getArray3fMap () { return (Eigen::Array3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Array3f> getArray3fMap () const { return (Eigen::Array3f::Map (data)); } \
  inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap () { return (Eigen::Array4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap () const { return (Eigen::Array4f::MapAligned (data)); }
#endif

#define PCL_ADD_UNION_POINT4D_HVD       \
  union EIGEN_ALIGN16 {                 \
    float data[4];                      \
    struct {                            \
      float h;                          \
      float v;                          \
      float d;                          \
    };                                  \
  };

#define PCL_ADD_POINT4D_HVD \
  PCL_ADD_UNION_POINT4D_HVD \
  PCL_ADD_EIGEN_MAPS_POINT4D

  /** Polar coordinate, including intensity and ring number. */
  struct PointHVDIR
  {
    PCL_ADD_POINT4D_HVD;                    // quad-word HVD
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;


} // namespace quanergy

POINT_CLOUD_REGISTER_POINT_STRUCT(quanergy::PointHVDIR,
                                  (float, h, h)
                                  (float, v, v)
                                  (float, d, d)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))



#endif // __QUANERGY_POINT_HVDIR_H
