/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Frame.h"

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"

namespace OpenFDM {

Frame::Frame() :
  mPosition(Vector3::zeros()),
  mOrientation(Quaternion::unit()),
  mRelVel(Vector6::zeros()),
  mRelVelDot(Vector6::zeros()),
  mParentSpVel(Vector6::zeros()),
  mParentSpAccel(Vector6::zeros()),
  mRefOrient(Quaternion::unit()),
  mRefPos(Vector3::zeros()),
  mRefVel(Vector6::zeros())
{
}

Frame::~Frame(void)
{
}

SpatialInertia
Frame::inertiaToParent(const SpatialInertia& I) const
{
  if (getOrientation().isIdentity()) {
    if (getPosition() == Vector3::zeros()) {
      return I;
    } else {
      return inertiaFrom(getPosition(), I);
    }
  } else {
    if (getPosition() == Vector3::zeros()) {
      return inertiaFrom(getOrientation(), I);
    } else {
      return inertiaFrom(getPosition(), getOrientation(), I);
    }
  }
}

} // namespace OpenFDM
