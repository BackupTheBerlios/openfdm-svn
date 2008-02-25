/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Rotation_H
#define OpenFDM_Rotation_H

#include <iosfwd>

#include "Assert.h"
#include "Types.h"
#include "Unit.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"

namespace OpenFDM {

namespace LinAlg {

template<typename T>
class Rotation
  : public Quaternion<T> {
public:
  typedef T value_type;
  typedef typename Quaternion<T>::Vector3 Vector3;
  typedef typename Quaternion<T>::Matrix33 Matrix33;

  OpenFDM_FORCE_INLINE
  Rotation(void)
  {}
  OpenFDM_FORCE_INLINE
  Rotation(const Rotation& r)
    : Quaternion<T>(r), mRotationMatrix(r.mRotationMatrix),
      mIdentity(r.mIdentity)
  {}
  OpenFDM_FORCE_INLINE
  Rotation(const Vector4<T>& v)
    : Quaternion<T>(v)
  { updateRotation(); }
  OpenFDM_FORCE_INLINE
  Rotation(value_type q1, value_type q2, value_type q3, value_type q4)
    : Quaternion<T>(q1, q2, q3, q4)
  { updateRotation(); }
  OpenFDM_FORCE_INLINE
  ~Rotation(void)
  {}

  OpenFDM_FORCE_INLINE
  bool isIdentity(void) const
  { return mIdentity; }

  OpenFDM_FORCE_INLINE
  Vector3 transform(const Vector3& v) const
  { if (mIdentity) return v; else return mRotationMatrix*v; }
  OpenFDM_FORCE_INLINE
  Vector3 backTransform(const Vector3& v) const
  { if (mIdentity) return v; else return trans(mRotationMatrix)*v; }

  OpenFDM_FORCE_INLINE
  const Matrix33& getTransform(void) const
  { return mRotationMatrix; }
  OpenFDM_FORCE_INLINE
  Matrix33 getBackTransform(void) const
  { return trans(mRotationMatrix); }
  
  OpenFDM_FORCE_INLINE
  Rotation& operator=(const Quaternion<T>& q)
  { Quaternion<T>::operator=(q); updateRotation(); return *this; }
  OpenFDM_FORCE_INLINE
  Rotation& operator=(const Rotation& q)
  {
    Quaternion<T>::operator=(q);
    mRotationMatrix = q.mRotationMatrix;
    mIdentity = q.mIdentity;
    return *this;
  }

  /** Arithmetic operator "*=".
      Inplace quaternion multiplication.
      @param q a quaternion to be multiplied.
      @return reference to *this. */
  OpenFDM_FORCE_INLINE
  Rotation& operator*=(const Quaternion<T>& q)
  {
    Quaternion<T>::operator=((*this)*q);
    updateRotation();
    return *this;
  }

  /** Arithmetic operator "/=".
      Inplace quaternion multiplication.
      @param q a quaternion to be multiplied.
      @return reference to *this. */
  OpenFDM_FORCE_INLINE
  Rotation& operator/=(const Quaternion<T>& q)
  {
    Quaternion<T>::operator=((*this)*inverse(q));
    updateRotation();
    return *this;
  }

private:
  void updateRotation(void)
  {
    mIdentity = Quaternion<T>::isIdentity();
    if (mIdentity)
      mRotationMatrix = Matrix33(1, 0, 0, 0, 1, 0, 0, 0, 1);
    else
      mRotationMatrix = Quaternion<T>::getTransform();
  }

  // The cached rotation matrix.
  Matrix33 mRotationMatrix;
  bool mIdentity;
};

} // namespace LinAlg

typedef LinAlg::Rotation<real_type> Rotation;

} // namespace OpenFDM

#endif
