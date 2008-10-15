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
                          PortValueList& portValues, MechanicContext& context) const
{
  Vector3 position = continousState[*mPositionStateInfo];
  Quaternion orientation = continousState[*mOrientationStateInfo];
  Vector6 velocity = continousState[*mVelocityStateInfo];

  Vector6 parentSpatialVelocity = angularMotionTo(position, orientation,
                                                  getAngularBaseVelocity());

  context.mParentSpVel = parentSpatialVelocity;
  context.mParentSpAccel = Vector6::zeros();
  Vector6 pivel = parentSpatialVelocity;
  context.mHDot = Vector6(cross(pivel.getAngular(), velocity.getAngular()),
                          cross(pivel.getAngular(), velocity.getLinear()) + 
                          cross(pivel.getLinear(), velocity.getAngular()));

  portValues[mMechanicLink].mPosition = position;
  portValues[mMechanicLink].mOrientation = orientation;
  portValues[mMechanicLink].mSpatialVelocity = velocity + parentSpatialVelocity;
}

void
MobileRootJoint::articulation(const Task&, const ContinousStateValueVector&,
                              PortValueList&, MechanicContext&) const
{
  /// In this case a noop.
}

void
MobileRootJoint::acceleration(const Task&, const ContinousStateValueVector&,
                              PortValueList& portValues, MechanicContext& context) const
{
  // Assumption: body is small compared to the distance to the planets
  // center of mass. That means gravity could be considered equal for the
  // whole vehicle.
  // See Featherstone, Orin: Equations and Algorithms
//   Vector3 ga = gravity->gravityAccel(getRefPosition());
//     Vector6 grav = Vector6(Vector3::zeros(), rotFromRef(ga));

  // FIXME
  Vector6 grav = Vector6(Vector3::zeros(), Vector3(0, 0, 9.81));

  SpatialInertia inertia = portValues[mMechanicLink].mArticulatedInertia;
  Vector6 force = portValues[mMechanicLink].mArticulatedForce;

  // FIXME
//   mRelVelDot = grav - solve(inertia, force) - getParentSpAccel() - getHdot();
  Vector6 acceleration = grav - solve(inertia, force) - context.mParentSpAccel - context.mHDot;
  context.mRelVelDot = acceleration;
  

  portValues[mMechanicLink].mSpatialAcceleration = acceleration;
}

void
MobileRootJoint::derivative(const DiscreteStateValueVector&,
                            const ContinousStateValueVector& continousState,
                            const PortValueList& portValues,
                            MechanicContext& context,
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

  derivatives[*mPositionStateInfo] = pDot; 
  derivatives[*mOrientationStateInfo] = qderiv;
  derivatives[*mVelocityStateInfo] = context.mRelVelDot;
}

} // namespace OpenFDM
