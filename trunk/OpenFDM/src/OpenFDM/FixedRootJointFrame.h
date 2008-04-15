/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_FixedRootJointFrame_H
#define OpenFDM_FixedRootJointFrame_H

#include "Assert.h"
#include "Vector.h"
#include "Matrix.h"
#include "Inertia.h"
#include "Frame.h"

namespace OpenFDM {

class FixedRootJointFrame :
  public Frame {
public:
  FixedRootJointFrame(const std::string& name) :
    Frame(name),
    mRelVelDot(Vector6::zeros())
  { }
  virtual ~FixedRootJointFrame(void)
  { }

  /// The interface routine for the Frame,
  /// returns the relative velocity derivative of this frame
  virtual const Vector6& getRelVelDot(void) const
  { return mRelVelDot; }

  /// Compute the articulated force and inertia past inboard to that joint
  void jointArticulation(const Vector6&, const SpatialInertia&,
                         const Gravity* gravity)
  {
    Log(ArtBody, Debug) << "FixedRootJointFrame::jointArticulation()" << endl;

    // Assumption: body is small compared to the distance to the planets
    // center of mass. That means gravity could be considered equal for the
    // whole vehicle.
    // See Featherstone, Orin: Equations and Algorithms
    Vector3 ga = gravity->gravityAccel(getRefPosition());
    Vector6 grav = Vector6(Vector3::zeros(), rotFromRef(ga));

    mRelVelDot = grav - getParentSpAccel() - getHdot();
    setAccelDirty();
  }

  using Frame::setPosition;
  using Frame::setRefPosition;
  using Frame::setOrientation;
  using Frame::setRefOrientation;

private:
  /// The derivative of the frame velocity
  Vector6 mRelVelDot;
};

} // namespace OpenFDM

#endif
