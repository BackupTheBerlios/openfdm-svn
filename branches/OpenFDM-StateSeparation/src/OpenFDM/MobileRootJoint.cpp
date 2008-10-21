/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "MobileRootJoint.h"

#include "Assert.h"
#include "LeafContext.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Gravity.h"
#include "MechanicContext.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(MobileRootJoint, RootJoint)
  END_OPENFDM_OBJECT_DEF

MobileRootJoint::MobileRootJoint(const std::string& name) :
  RootJoint(name),
  mMechanicLink(newMechanicLink("link")),
  mPositionStateInfo(new Vector3StateInfo),
  mOrientationStateInfo(new Vector4StateInfo),
  mVelocityStateInfo(new Vector6StateInfo)
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mOrientationStateInfo);
  addContinousStateInfo(mVelocityStateInfo);
}

MobileRootJoint::~MobileRootJoint()
{
}

void
MobileRootJoint::init(const Task&, DiscreteStateValueVector&,
                      ContinousStateValueVector& continousState,
                      const PortValueList& portValues) const
{
  continousState[*mPositionStateInfo] = Vector3::zeros();
  continousState[*mOrientationStateInfo] = Quaternion::unit();
  continousState[*mVelocityStateInfo] = Vector6::zeros();
}

void
MobileRootJoint::velocity(const Task&,
                          const ContinousStateValueVector& continousState,
                          PortValueList& portValues, FrameData&) const
{
  Vector3 position = continousState[*mPositionStateInfo];
  Quaternion orientation = continousState[*mOrientationStateInfo];
  Vector6 velocity = continousState[*mVelocityStateInfo];

  portValues[mMechanicLink].setPosAndVel(getAngularBaseVelocity(),
                                         position, orientation, velocity);
}

void
MobileRootJoint::articulation(const Task&, const ContinousStateValueVector&,
                              PortValueList&, FrameData&) const
{
  /// In this case a noop.
}

void
MobileRootJoint::acceleration(const Task&, const ContinousStateValueVector&,
                              PortValueList& portValues,
                              FrameData&) const
{
  // Assumption: body is small compared to the distance to the planets
  // center of mass. That means gravity could be considered equal for the
  // whole vehicle.
  // See Featherstone, Orin: Equations and Algorithms
//   Vector3 ga = gravity->gravityAccel(getRefPosition());
//     Vector6 grav = Vector6(Vector3::zeros(), rotFromRef(ga));

  // FIXME
  Vector6 grav = Vector6(Vector3::zeros(), Vector3(0, 0, 9.81));

  SpatialInertia inertia = portValues[mMechanicLink].getInertia();
  Vector6 force = portValues[mMechanicLink].getForce();

  Vector6 spatialAcceleration = grav - solve(inertia, force);
  portValues[mMechanicLink].getFrame().setSpAccel(spatialAcceleration);
}

void
MobileRootJoint::derivative(const DiscreteStateValueVector&,
                            const ContinousStateValueVector& continousState,
                            const PortValueList& portValues,
                            FrameData& context,
                            ContinousStateValueVector& derivatives) const
{
  Quaternion orientation = continousState[*mOrientationStateInfo];
  Vector6 velocity = continousState[*mVelocityStateInfo];

  Vector3 pDot = orientation.backTransform(velocity.getLinear());

  // Compute the derivative term originating from the angular velocity.
  // Correction term to keep the quaternion normalized.
  // That is if |q| < 1 add a little radial component outward,
  // if |q| > 1 add a little radial component inward
  Quaternion q = orientation;
  Vector3 angVel = velocity.getAngular();
  Vector4 qderiv = LinAlg::derivative(q, angVel) + 1e1*(normalize(q) - q);

  Vector6 velDeriv = portValues[mMechanicLink].getFrame().getRelVelDot();

  derivatives[*mPositionStateInfo] = pDot;
  derivatives[*mOrientationStateInfo] = qderiv;
  derivatives[*mVelocityStateInfo] = velDeriv;
}

} // namespace OpenFDM
