/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "RevoluteJoint.h"
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

BEGIN_OPENFDM_OBJECT_DEF(RevoluteJoint, Joint)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  END_OPENFDM_OBJECT_DEF

RevoluteJoint::RevoluteJoint(const std::string& name) :
  Joint(name),
  mForcePort(this, "force", Size(1, 1), true),
  mPositionPort(this, "position", Size(1, 1)),
  mVelocityPort(this, "velocity", Size(1, 1)),
  mPositionStateInfo(new Vector1StateInfo),
  mVelocityStateInfo(new Vector1StateInfo),
  mAxis(Vector3(1, 0, 0))
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mVelocityStateInfo);

  // FIXME
  setAxis(mAxis);
}

RevoluteJoint::~RevoluteJoint(void)
{
}

const Vector3&
RevoluteJoint::getAxis() const
{
  return mAxis;
}

void
RevoluteJoint::setAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm <= Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }
  mAxis = (1/nrm)*axis;
  mJointMatrix = Vector6(mAxis, Vector3::zeros());
}

void
RevoluteJoint::init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector& continousState,
                    const PortValueList&) const
{
  continousState[*mPositionStateInfo] = 0;
  continousState[*mVelocityStateInfo] = 0;
}

void
RevoluteJoint::velocity(const MechanicLinkValue& parentLink,
                        MechanicLinkValue& childLink,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues,
                        FrameData& frameData) const
{
  VectorN jointPos = states[*mPositionStateInfo];
  if (!mPositionPort.empty())
    portValues[mPositionPort] = jointPos;

  VectorN jointVel = states[*mVelocityStateInfo];
  if (!mVelocityPort.empty())
    portValues[mVelocityPort] = jointVel;

  Vector3 position(0, 0, 0);
  Quaternion orientation(Quaternion::fromAngleAxis(jointPos(0), mAxis));
  Vector6 velocity(mAxis*jointVel, Vector3::zeros());

  childLink.setPosAndVel(parentLink, position, orientation, velocity);
}

void
RevoluteJoint::articulation(MechanicLinkValue& parentLink,
                            const MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            FrameData& frameData) const
{
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];

  // The formulas conform to Roy Featherstones book eqn (6.37), (6.38)

  // Store the outboard values since we will need them later in velocity
  // derivative computations
  SpatialInertia I = childLink.getInertia();

  // Compute the projection to the joint coordinate space
  Matrix6N Ih = I*mJointMatrix;
  frameData.hIh = trans(mJointMatrix)*Ih;
  MatrixFactorsNN hIh = MatrixNN(frameData.hIh);

  // Note that the momentum of the local mass is already included in the
  // child links force due the the mass model ...
  Vector6 mPAlpha = childLink.getForce() + I*childLink.getFrame().getHdot();
  Vector6 force = mPAlpha;

  if (hIh.singular()) {
    Log(ArtBody,Error) << "Detected singular mass matrix for "
                       << "CartesianJointFrame \"" << getName()
                       << "\": Fix your model!" << endl;
    return;
  }
  
  // Project away the directions handled with this current joint
  force -= Ih*hIh.solve(trans(mJointMatrix)*mPAlpha - jointForce);
  I -= SpatialInertia(Ih*hIh.solve(trans(Ih)));

  // Transform to parent link's coordinates and apply to the parent link
  parentLink.applyForce(childLink.getFrame().forceToParent(force));
  parentLink.applyInertia(childLink.getFrame().inertiaToParent(I));
}

void
RevoluteJoint::acceleration(const MechanicLinkValue& parentLink,
                            MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            FrameData& frameData) const
{
  Vector6 parentSpAccel
    = childLink.getFrame().motionFromParent(parentLink.getFrame().getSpAccel());

  Vector6 f = childLink.getForce();
  f += childLink.getInertia()*(parentSpAccel + childLink.getFrame().getHdot());
  MatrixFactorsNN hIh = MatrixNN(frameData.hIh);
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];
  VectorN velDot = hIh.solve(jointForce - trans(mJointMatrix)*f);
  frameData.velDot = velDot;
  childLink.setAccel(parentLink, mJointMatrix*velDot);
}

void
RevoluteJoint::derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector& states,
                          const PortValueList&, FrameData& frameData,
                          ContinousStateValueVector& derivative) const
{
  derivative[*mPositionStateInfo] = states[*mVelocityStateInfo];
  derivative[*mVelocityStateInfo] = frameData.velDot;
}

} // namespace OpenFDM
