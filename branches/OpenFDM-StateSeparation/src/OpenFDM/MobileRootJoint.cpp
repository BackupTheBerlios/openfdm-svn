/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "MobileRootJoint.h"

#include "Assert.h"
#include "LogStream.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Task.h"

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
MobileRootJoint::initDesignPosition(PortValueList& portValues) const
{
  portValues[mMechanicLink].setDesignPosition(getPosition());
}

void
MobileRootJoint::velocity(const Task& task, const Environment& environment,
                          const ContinousStateValueVector& continousState,
                          PortValueList& portValues) const
{
  Vector3 angularBaseVelocity = environment.getAngularVelocity(task.getTime());

  Vector3 position = continousState[*mPositionStateInfo];
  Quaternion orientation = continousState[*mOrientationStateInfo];
  Vector6 velocity = continousState[*mVelocityStateInfo];

  portValues[mMechanicLink].setCoordinateSystem(CoordinateSystem(position,
                                                                 orientation));
  portValues[mMechanicLink].setPosAndVel(angularBaseVelocity,
                                         position, orientation, velocity);
}

void
MobileRootJoint::articulation(const Task&, const Environment& environment,
                              const ContinousStateValueVector&,
                              PortValueList&) const
{
  /// In this case a noop.
}

void
MobileRootJoint::acceleration(const Task& task, const Environment& environment,
                              const ContinousStateValueVector&,
                              PortValueList& portValues) const
{
  Vector6 spatialAcceleration = environment.getAcceleration(task.getTime());

  SpatialInertia inertia = portValues[mMechanicLink].getInertia();
  Vector6 force = portValues[mMechanicLink].getForce();

  spatialAcceleration -= solve(inertia, force);
  portValues[mMechanicLink].getFrame().setSpAccel(spatialAcceleration);
}

void
MobileRootJoint::derivative(const Environment& environment,
                            const DiscreteStateValueVector&,
                            const ContinousStateValueVector& continousState,
                            const PortValueList& portValues,
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
