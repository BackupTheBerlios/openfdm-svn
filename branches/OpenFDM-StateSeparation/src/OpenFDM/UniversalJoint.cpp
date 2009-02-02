/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "UniversalJoint.h"

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

BEGIN_OPENFDM_OBJECT_DEF(UniversalJoint, Joint)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  END_OPENFDM_OBJECT_DEF

UniversalJoint::UniversalJoint(const std::string& name) :
  CartesianJoint<2>(name),
  mOrientationPort(this, "orientation", Size(4, 1)),
  mVelocityPort(this, "velocity", Size(2, 1)),
  mPositionStateInfo(new Vector3StateInfo),
  mVelocityStateInfo(new Vector2StateInfo),
  mAxis(Vector3(1, 0, 0))
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mVelocityStateInfo);
}

UniversalJoint::~UniversalJoint(void)
{
}

const Vector3&
UniversalJoint::getAxis() const
{
  return mAxis;
}

void
UniversalJoint::setAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm <= Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }
  mAxis = (1/nrm)*axis;
  mOrientation = Quaternion::fromRotateTo(Vector3(0, 0, 1), mAxis);
}

void
UniversalJoint::setEnableExternalForce(bool enable)
{
  if (enable == getEnableExternalForce())
    return;
  if (enable)
    mForcePort = MatrixInputPort(this, "force", Size(2, 1), true);
  else
    mForcePort.clear();
}

bool
UniversalJoint::getEnableExternalForce() const
{
  return !mForcePort.empty();
}

void
UniversalJoint::init(const Task&, DiscreteStateValueVector&,
                     ContinousStateValueVector& continousState,
                     const PortValueList&) const
{
  continousState[*mPositionStateInfo] = Vector3(1, 0, 0);
  continousState[*mVelocityStateInfo] = Vector2(0, 0);
}

UniversalJoint::Matrix6N
UniversalJoint::getJointMatrix() const
{
  Vector3 axis1 = perpendicular(mAxis);
  Vector3 axis2 = cross(mAxis, axis1);
  Matrix6N jointMatrix;
  jointMatrix(Range(0, 5), Range(0)) = Vector6(axis1, Vector3::zeros());
  jointMatrix(Range(0, 5), Range(1)) = Vector6(axis2, Vector3::zeros());
  return jointMatrix;
}

void
UniversalJoint::velocity(const Task&, Context& context,
                         const ContinousStateValueVector& states,
                         PortValueList& portValues) const
{
  Vector3 jointPos = states[*mPositionStateInfo];
  Quaternion orientation(jointPos(0), jointPos(1), 0, jointPos(2));
  orientation *= mOrientation;

  if (!mOrientationPort.empty())
    portValues[mOrientationPort] = orientation;
  
  VectorN jointVel = states[*mVelocityStateInfo];
  if (!mVelocityPort.empty())
    portValues[mVelocityPort] = jointVel;
  
  context.setPosAndVel(Vector3::zeros(), orientation, jointVel);
}

void
UniversalJoint::articulation(const Task&, Context& context,
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
UniversalJoint::acceleration(const Task&, Context& context,
                             const ContinousStateValueVector&,
                             PortValueList&) const
{
  context.accelerateDueToForce();
}

void
UniversalJoint::derivative(const Task&, Context& context,
                           const ContinousStateValueVector& states,
                           const PortValueList&,
                           ContinousStateValueVector& derivative) const
{
  Vector3 jointPos = states[*mPositionStateInfo];
  Quaternion q = Quaternion(jointPos(0), jointPos(1), 0, jointPos(2));
  q *= mOrientation;

  // Compute the derivative term originating from the angular velocity.
  // Correction term to keep the quaternion normalized.
  // That is if |q| < 1 add a little radial component outward,
  // if |q| > 1 add a little radial component inward
  Vector3 angVel = getJointMatrix()(Range(0, 2), Range(0, 1))
    *states[*mVelocityStateInfo];
  Vector4 qderiv = LinAlg::derivative(q, angVel) + 1e1*(normalize(q) - q);

  derivative[*mPositionStateInfo] = Vector3(qderiv(0), qderiv(1), qderiv(3));
  derivative[*mVelocityStateInfo] = context.getVelDot();
}

} // namespace OpenFDM
