/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "MobileRootJoint.h"

#include "Assert.h"
#include "JointContext.h"
#include "LogStream.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Task.h"

namespace OpenFDM {

class MobileRootJoint::Context : public JointContext {
public:
  Context(const MobileRootJoint* rootJoint, const Environment* environment,
          MechanicLinkValue* parentLinkValue, MechanicLinkValue* childLinkValue,
          PortValueList& portValueList) :
    JointContext(environment, parentLinkValue, childLinkValue, portValueList),
    mMobileRootJoint(rootJoint)
  {}
  virtual ~Context() {}
  
  virtual const MobileRootJoint& getNode() const
  { return *mMobileRootJoint; }
  
  virtual void initDesignPosition()
  {
    mChildLink.setDesignPosition(mMobileRootJoint->getPosition());
  }

  virtual void init(const /*Init*/Task& task)
  {
    mMobileRootJoint->init(task, mContinousState);
  }
  
  virtual void velocities(const Task& task)
  {
    mMobileRootJoint->velocity(task, getEnvironment(), mContinousState, mChildLink);
  }
  virtual void articulation(const Task& task)
  {
  }
  virtual void accelerations(const Task& task)
  {
    SpatialInertia inertia = mChildLink.getInertia();
    Vector6 force = -mChildLink.getForce();
    Vector6 spatialAcceleration = solve(inertia, force);
    mChildLink.setSpAccel(spatialAcceleration);
  }
  
  virtual void derivative(const Task& task)
  {
    mMobileRootJoint->derivative(task, getEnvironment(), mDiscreteState, mContinousState, mChildLink,
                           mContinousStateDerivative);
  }
  
private:
  SharedPtr<const MobileRootJoint> mMobileRootJoint;
};

BEGIN_OPENFDM_OBJECT_DEF(MobileRootJoint, RootJoint)
  DEF_OPENFDM_PROPERTY(Vector3, InitialPosition, Serialized)
  DEF_OPENFDM_PROPERTY(Quaternion, InitialOrientation, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, InitialLinearVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, InitialAngularVelocity, Serialized)
  END_OPENFDM_OBJECT_DEF

MobileRootJoint::MobileRootJoint(const std::string& name) :
  RootJoint(name),
  mMechanicLink(new MechanicLink(this, "link")),
  mPositionStateInfo(new Vector3StateInfo),
  mOrientationStateInfo(new Vector4StateInfo),
  mVelocityStateInfo(new Vector6StateInfo),
  mInitialPosition(Vector3::zeros()),
  mInitialOrientation(Quaternion::unit()),
  mInitialLinearVelocity(Vector3::zeros()),
  mInitialAngularVelocity(Vector3::zeros())
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mOrientationStateInfo);
  addContinousStateInfo(mVelocityStateInfo);
}

MobileRootJoint::~MobileRootJoint()
{
}

JointContext*
MobileRootJoint::newJointContext(const Environment* environment,
                                 MechanicLinkValue* parentLinkValue,
                                 MechanicLinkValue* childLinkValue,
                                 PortValueList& portValueList) const
{
  SharedPtr<Context> context = new Context(this, environment, parentLinkValue,
                                           childLinkValue, portValueList);
  if (!context->allocStates()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return false;
  }
  return context.release();
}

const Vector3&
MobileRootJoint::getInitialPosition() const
{
  return mInitialPosition;
}

void
MobileRootJoint::setInitialPosition(const Vector3& initialPosition)
{
  mInitialPosition = initialPosition;
}

const Quaternion&
MobileRootJoint::getInitialOrientation() const
{
  return mInitialOrientation;
}

void
MobileRootJoint::setInitialOrientation(const Quaternion& initialOrientation)
{
  mInitialOrientation = initialOrientation;
}

const Vector3&
MobileRootJoint::getInitialLinearVelocity() const
{
  return mInitialLinearVelocity;
}

void
MobileRootJoint::setInitialLinearVelocity(const Vector3& initialVelocity)
{
  mInitialLinearVelocity = initialVelocity;
}

const Vector3&
MobileRootJoint::getInitialAngularVelocity() const
{
  return mInitialAngularVelocity;
}

void
MobileRootJoint::setInitialAngularVelocity(const Vector3& initialVelocity)
{
  mInitialAngularVelocity = initialVelocity;
}


void
MobileRootJoint::init(const Task&,
                      ContinousStateValueVector& continousState) const
{
  continousState[*mPositionStateInfo] = mInitialPosition;
  continousState[*mOrientationStateInfo] = mInitialOrientation;
  Vector6 initialVelocity(mInitialAngularVelocity, mInitialLinearVelocity);
  continousState[*mVelocityStateInfo] = initialVelocity;
}

void
MobileRootJoint::velocity(const Task& task, const Environment& environment,
                          const ContinousStateValueVector& continousState,
                          ChildLink& childLink) const
{
  Vector3 angularBaseVelocity = environment.getAngularVelocity(task.getTime());

  Vector3 position = continousState[*mPositionStateInfo];
  Quaternion orientation = continousState[*mOrientationStateInfo];
  Vector6 velocity = continousState[*mVelocityStateInfo];

  childLink.setCoordinateSystem(CoordinateSystem(position, orientation));
  childLink.setPosAndVel(angularBaseVelocity, position, orientation, velocity);
}

void
MobileRootJoint::derivative(const Task& task, const Environment& environment,
                            const DiscreteStateValueVector&,
                            const ContinousStateValueVector& continousState,
                            const ChildLink& childLink,
                            ContinousStateValueVector& derivatives) const
{
  Vector3 position = continousState[*mPositionStateInfo];
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

  // Now the derivative of the relative velocity, that is take the
  // spatial acceleration and subtract the inertial base velocity and the
  // derivative of the 'joint matrix'.
  Vector6 spatialAcceleration = environment.getAcceleration(task.getTime());
//       pivel = mRelativeCoordinateSystem.motionToLocal(pivel);
  spatialAcceleration = motionTo(position, orientation, spatialAcceleration);

  Vector3 angularBaseVelocity = environment.getAngularVelocity(task.getTime());
//       Vector6 pivel = mParentLink.getSpVelAtLink();
  Vector6 pivel(angularBaseVelocity, Vector3::zeros());
//       pivel = mRelativeCoordinateSystem.motionToLocal(pivel);
  Vector6 relVel = velocity;
  Vector6 Hdot = Vector6(cross(pivel.getAngular(), relVel.getAngular()),
                         cross(pivel.getAngular(), relVel.getLinear()) + 
                         cross(pivel.getLinear(), relVel.getAngular()));

  Vector6 velDeriv = childLink.getSpAccel() - spatialAcceleration - Hdot;

  derivatives[*mPositionStateInfo] = pDot;
  derivatives[*mOrientationStateInfo] = qderiv;
  derivatives[*mVelocityStateInfo] = velDeriv;
}

} // namespace OpenFDM
