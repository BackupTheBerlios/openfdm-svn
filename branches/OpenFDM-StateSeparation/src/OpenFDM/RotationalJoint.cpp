/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "RotationalJoint.h"
#include "Assert.h"
#include "LogStream.h"
#include "Limits.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "PortValueList.h"
#include "ContinousStateValueVector.h"
#include "MechanicContext.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RotationalJoint, Joint)
  DEF_OPENFDM_PROPERTY(Quaternion, InitialOrientation, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, InitialVelocity, Serialized)
  END_OPENFDM_OBJECT_DEF

RotationalJoint::RotationalJoint(const std::string& name) :
  CartesianJoint<3>(name),
  mOrientationPort(this, "orientation", Size(4, 1)),
  mVelocityPort(this, "velocity", Size(3, 1)),
  mPositionStateInfo(new Vector4StateInfo),
  mVelocityStateInfo(new Vector3StateInfo),
  mInitialOrientation(Quaternion::unit()),
  mInitialVelocity(Vector3::zeros())
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mVelocityStateInfo);
}

RotationalJoint::~RotationalJoint(void)
{
}

const Quaternion&
RotationalJoint::getInitialOrientation() const
{
  return mInitialOrientation;
}

void
RotationalJoint::setInitialOrientation(const Quaternion& initialOrientation)
{
  mInitialOrientation = initialOrientation;
}

const Vector3&
RotationalJoint::getInitialVelocity() const
{
  return mInitialVelocity;
}

void
RotationalJoint::setInitialVelocity(const Vector3& initialVelocity)
{
  mInitialVelocity = initialVelocity;
}

void
RotationalJoint::setEnableExternalForce(bool enable)
{
  if (enable == getEnableExternalForce())
    return;
  if (enable)
    mForcePort = MatrixInputPort(this, "force", Size(3, 1), true);
  else
    mForcePort.clear();
}

bool
RotationalJoint::getEnableExternalForce() const
{
  return !mForcePort.empty();
}

void
RotationalJoint::init(const Task&, DiscreteStateValueVector&,
                      ContinousStateValueVector& continousState,
                      const PortValueList&) const
{
  continousState[*mPositionStateInfo] = mInitialOrientation;
  continousState[*mVelocityStateInfo] = mInitialVelocity;
}

RotationalJoint::Matrix6N
RotationalJoint::getJointMatrix() const
{
  Matrix6N jointMatrix;
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 6; ++j)
      jointMatrix(j, i) = real_type(i == j);
  return jointMatrix;
}

void
RotationalJoint::velocity(const Task&, Context& context,
                          const ContinousStateValueVector& states,
                          PortValueList& portValues) const
{
  Quaternion orientation = states[*mPositionStateInfo];
  if (!mOrientationPort.empty())
    portValues[mOrientationPort] = orientation;
  
  VectorN jointVel = states[*mVelocityStateInfo];
  if (!mVelocityPort.empty())
    portValues[mVelocityPort] = jointVel;
  
  context.setPosAndVel(Vector3::zeros(), orientation, jointVel);
}

void
RotationalJoint::articulation(const Task&, Context& context,
                              const ContinousStateValueVector& states,
                              PortValueList& portValues) const
{
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];
  
  context.applyJointForce(jointForce);
}

void
RotationalJoint::acceleration(const Task&, Context& context,
                              const ContinousStateValueVector&,
                              PortValueList&) const
{
  context.accelerateDueToForce();
}

void
RotationalJoint::derivative(const Task&, Context& context,
                            const ContinousStateValueVector& states,
                            const PortValueList&,
                            ContinousStateValueVector& derivative) const
{
  // Compute the derivative term originating from the angular velocity.
  // Correction term to keep the quaternion normalized.
  // That is if |q| < 1 add a little radial component outward,
  // if |q| > 1 add a little radial component inward
  Quaternion q = states[*mPositionStateInfo];
  Vector3 angVel = states[*mVelocityStateInfo];
  Vector4 qderiv = LinAlg::derivative(q, angVel) + 1e1*(normalize(q) - q);

  derivative[*mPositionStateInfo] = qderiv;
  derivative[*mVelocityStateInfo] = context.getVelDot();
}

} // namespace OpenFDM
