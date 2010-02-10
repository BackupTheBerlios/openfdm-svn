/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_CoordinateSystem_H
#define OpenFDM_CoordinateSystem_H

#include <iosfwd>
#include "Vector.h"
#include "Plane.h"
#include "Transform.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Rotation.h"
#include "Inertia.h"

namespace OpenFDM {

class CoordinateSystem {
public:
  CoordinateSystem() :
    mPosition(Vector3::zeros()),
    mOrientation(Quaternion::unit())
  { }
  explicit CoordinateSystem(const Vector3& position) :
    mPosition(position),
    mOrientation(Quaternion::unit())
  { }
  explicit CoordinateSystem(const Quaternion& orientation) :
    mPosition(Vector3::zeros()),
    mOrientation(orientation)
  { }
  explicit CoordinateSystem(const Rotation& orientation) :
    mPosition(Vector3::zeros()),
    mOrientation(orientation)
  { }
  CoordinateSystem(const Vector3& position, const Quaternion& orientation) :
    mPosition(position),
    mOrientation(orientation)
  { }
  CoordinateSystem(const Vector3& position, const Rotation& orientation) :
    mPosition(position),
    mOrientation(orientation)
  { }
  CoordinateSystem(const CoordinateSystem& coordinateSystem) :
    mPosition(coordinateSystem.mPosition),
    mOrientation(coordinateSystem.mOrientation)
  { }

  /// Return the position of this coordinate system wrt the reference coordinate
  /// system. The position is measured in the reference coordinate
  /// systems coordinates.
  const Vector3& getPosition() const
  { return mPosition; }
  void setPosition(const Vector3& position)
  { mPosition = position; }

  /// Return the orientation of this coordinate system wrt the reference
  /// coordinate system. The orientation is measured in the reference coordinate
  /// systems coordinates.
  const Rotation& getOrientation() const
  { return mOrientation; }
  void setOrientation(const Rotation& orientation)
  { mOrientation = orientation; }
  void setOrientation(const Quaternion& orientation)
  { mOrientation = orientation; }

  /// Set up a new coordinate system with a relative position
  /// and a relative orientation to this parent coordinate system.
  /// ??? is that below ...
  CoordinateSystem getRelative(const Vector3& position) const
  { return CoordinateSystem(toReference(position), mOrientation); }
  CoordinateSystem getRelative(const Quaternion& orientation) const
  { return CoordinateSystem(mPosition, toReference(orientation)); }
  CoordinateSystem getRelative(const Vector3& p, const Quaternion& o) const
  { return CoordinateSystem(toReference(p), toReference(o)); }

  // Returns the reference coordinates system in this coordinate systems
  // coordinates.
  CoordinateSystem referenceToLocal() const
  { return toLocal(CoordinateSystem()); }

  /// Conversion functions between local and reference coordinates
  CoordinateSystem toReference(const CoordinateSystem& coorinateSystem) const
  { return CoordinateSystem(toReference(coorinateSystem.getPosition()),
                            toReference(coorinateSystem.getOrientation())); }
  CoordinateSystem toLocal(const CoordinateSystem& coorinateSystem) const
  { return CoordinateSystem(toLocal(coorinateSystem.getPosition()),
                            toLocal(coorinateSystem.getOrientation())); }

  Vector3 toReference(const Vector3& position) const
  { return mOrientation.backTransform(position) + mPosition; }
  Vector3 toLocal(const Vector3& position) const
  { return mOrientation.transform(position - mPosition); }
  Quaternion toReference(const Quaternion& orientation) const
  { return mOrientation*orientation; }
  Quaternion toLocal(const Quaternion& orientation) const
  { return orientation*inverse(mOrientation); }

  Vector3 rotToReference(const Vector3& v) const
  { return mOrientation.backTransform(v); }
  Vector3 rotToLocal(const Vector3& v) const
  { return mOrientation.transform(v); }

  Vector6 rotToReference(const Vector6& v) const
  { return Vector6(mOrientation.backTransform(v.getAngular()),
                   mOrientation.backTransform(v.getLinear())); }
  Vector6 rotToLocal(const Vector6& v) const
  { return Vector6(mOrientation.transform(v.getAngular()),
                   mOrientation.transform(v.getLinear())); }

  template<unsigned n>
  LinAlg::Matrix<real_type,3,n>
  rotToReference(const LinAlg::Matrix<real_type,3,n>& m) const
  {
    LinAlg::Matrix<real_type,3,n> result;
    for (unsigned i = 0; i < n; ++i)
      result(Range(0, 2), i) = mOrientation.backTransform(m(Range(0, 2), i));
    return result;
  }
  template<unsigned n>
  LinAlg::Matrix<real_type,3,n>
  rotToLocal(const LinAlg::Matrix<real_type,3,n>& m) const
  {
    LinAlg::Matrix<real_type,3,n> result;
    for (unsigned i = 0; i < n; ++i)
      result(Range(0, 2), i) = mOrientation.transform(m(Range(0, 2), i));
    return result;
  }


  template<unsigned n>
  LinAlg::Matrix<real_type,6,n>
  rotToReference(const LinAlg::Matrix<real_type,6,n>& m) const
  {
    LinAlg::Matrix<real_type,6,n> result;
    for (unsigned i = 0; i < n; ++i) {
      result(Range(0, 2), i) = mOrientation.backTransform(m(Range(0, 2), i));
      result(Range(3, 5), i) = mOrientation.backTransform(m(Range(3, 5), i));
    }
    return result;
  }
  template<unsigned n>
  LinAlg::Matrix<real_type,6,n>
  rotToLocal(const LinAlg::Matrix<real_type,6,n>& m) const
  {
    LinAlg::Matrix<real_type,6,n> result;
    for (unsigned i = 0; i < n; ++i) {
      result(Range(0, 2), i) = mOrientation.transform(m(Range(0, 2), i));
      result(Range(3, 5), i) = mOrientation.transform(m(Range(3, 5), i));
    }
    return result;
  }




  InertiaMatrix rotToReference(const InertiaMatrix& inertia) const
  { return inertiaFrom(mOrientation, inertia); }
  InertiaMatrix rotToLocal(const InertiaMatrix& inertia) const
  { return inertiaTo(mOrientation, inertia); }

  SpatialInertia rotToReference(const SpatialInertia& inertia) const
  { return inertiaFrom(mOrientation, inertia); }
  SpatialInertia rotToLocal(const SpatialInertia& inertia) const
  { return inertiaFrom(Rotation(inverse(mOrientation)), inertia); }


  // Avoid that here. We *only* handle coordinate systems not reference frames.
  Vector6 motionToReference(const Vector6& motion) const
  { return motionFrom(mPosition, mOrientation, motion); }
  Vector6 motionToLocal(const Vector6& motion) const
  { return motionTo(mPosition, mOrientation, motion); }

  // Hmm force and inertia have this cross product stuff also,
  // is that something reference frame related or not ???
  Vector6 forceToReference(const Vector6& force) const
  { return forceFrom(mPosition, mOrientation, force); }
  Vector6 forceToLocal(const Vector6& force) const
  { return forceTo(mPosition, mOrientation, force); }

  SpatialInertia inertiaToReference(const SpatialInertia& inertia) const
  {
    if (mOrientation.isIdentity()) {
      if (mPosition == Vector3::zeros()) {
        return inertia;
      } else {
        return inertiaFrom(mPosition, inertia);
      }
    } else {
      if (mPosition == Vector3::zeros()) {
        return inertiaFrom(mOrientation, inertia);
      } else {
        return inertiaFrom(mPosition, mOrientation, inertia);
      }
    }
  }
  // Seldom used function, optimize at some time
  SpatialInertia inertiaToLocal(const SpatialInertia& inertia) const
  { return referenceToLocal().inertiaToReference(inertia); }

  Plane planeToReference(const Plane& plane) const
  { return planeFrom(mPosition, mOrientation, plane); }
  Plane planeToLocal(const Plane& plane) const
  { return planeTo(mPosition, mOrientation, plane); }

private:
  Vector3 mPosition;
  Rotation mOrientation;
};

template<typename char_type, typename traits_type>
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os,
           const CoordinateSystem& coordinateSystem)
{
  os << os.widen('(') << trans(coordinateSystem.getPosition()) << os.widen(' ')
     << coordinateSystem.getOrientation() << os.widen(')');
  return os;
}

} // namespace OpenFDM

#endif
