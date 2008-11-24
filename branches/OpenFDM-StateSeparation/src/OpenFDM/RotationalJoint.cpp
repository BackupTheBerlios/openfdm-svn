/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RotationalJoint, Joint)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  END_OPENFDM_OBJECT_DEF

RotationalJoint::RotationalJoint(const std::string& name) :
  CartesianJoint<3>(name),
  mOrientationPort(this, "orientation", Size(4, 1)),
  mVelocityPort(this, "velocity", Size(3, 1)),
  mPositionStateInfo(new Vector4StateInfo),
  mVelocityStateInfo(new Vector3StateInfo)
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mVelocityStateInfo);

  Matrix6N jointMatrix;
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 6; ++j)
      jointMatrix(j, i) = real_type(i == j);
  setJointMatrix(jointMatrix);
}

RotationalJoint::~RotationalJoint(void)
{
}

const Vector3&
RotationalJoint::getPosition() const
{
  return CartesianJoint<3>::getPosition();
}

void
RotationalJoint::setPosition(const Vector3& position)
{
  CartesianJoint<3>::setPosition(position);
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
  continousState[*mPositionStateInfo] = Quaternion::unit();
  continousState[*mVelocityStateInfo] = Vector3::zeros();
}

void
RotationalJoint::velocity(const MechanicLinkValue& parentLink,
                        MechanicLinkValue& childLink,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues) const
{
  Quaternion orientation = states[*mPositionStateInfo];
  if (!mOrientationPort.empty())
    portValues[mOrientationPort] = orientation;
  
  VectorN jointVel = states[*mVelocityStateInfo];
  if (!mVelocityPort.empty())
    portValues[mVelocityPort] = jointVel;
  
  Vector3 position = getPosition() - parentLink.getDesignPosition();
  velocity(parentLink, childLink, position,
           orientation, getJointMatrix()*jointVel);
}

void
RotationalJoint::articulation(MechanicLinkValue& parentLink,
                              const MechanicLinkValue& childLink,
                              const ContinousStateValueVector& states,
                              PortValueList& portValues,
                              MatrixFactorsNN& hIh, Vector6& pAlpha) const
{
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];
  
  articulation(parentLink, childLink, jointForce, hIh, pAlpha);
}

void
RotationalJoint::acceleration(const MechanicLinkValue& parentLink,
                              MechanicLinkValue& childLink,
                              const ContinousStateValueVector& states,
                              PortValueList& portValues,
                              const MatrixFactorsNN& hIh, const Vector6& pAlpha,
                              VectorN& velDot) const
{
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];
  
  acceleration(parentLink, childLink, jointForce, hIh, pAlpha, velDot);
}

void
RotationalJoint::derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector& states,
                          const PortValueList&, const VectorN& velDot,
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
  derivative[*mVelocityStateInfo] = velDot;
}

} // namespace OpenFDM
