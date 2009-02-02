/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "InternalInteract.h"

#include "NumericPortValue.h"
#include "PortValueList.h"
#include "Task.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(InternalInteract, DoubleLinkInteract)
  DEF_OPENFDM_PROPERTY(Vector3, Position0, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, Position1, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableDistance, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableForce, Serialized)
  END_OPENFDM_OBJECT_DEF

class InternalInteract::Context : public DoubleLinkInteract::Context {
public:
  Context(const InternalInteract* internalInteract,
          const Environment* environment, PortValueList& portValueList) :
    DoubleLinkInteract::Context(internalInteract, environment, portValueList),
    mInternalSensor(internalInteract),
    mDistanceValue(portValueList.getPortValue(internalInteract->mDistancePort)),
    mVelocityValue(portValueList.getPortValue(internalInteract->mVelocityPort)),
    mForceValue(portValueList.getPortValue(internalInteract->mForcePort))
  { }
  virtual ~Context() {}
    
  virtual const InternalInteract& getNode() const
  { return *mInternalSensor; }

  virtual void initDesignPosition()
  {
    getLink0().setDesignPosition(mInternalSensor->getPosition0());
    getLink1().setDesignPosition(mInternalSensor->getPosition1());
  }
  virtual void velocities(const Task&)
  {
    mRelCoordSys = getLink0().getRelativeCoordinateSystem(getLink1());
  
    Vector3 relPos = mRelCoordSys.getPosition();

    real_type nrmRelPos = norm(relPos);
    if (nrmRelPos <= Limits<real_type>::min())
      mDirection = Vector3::zeros();
    else
      mDirection = (1/nrmRelPos)*relPos;

    // The relative distance of these two points
    if (mDistanceValue.isConnected())
      mDistanceValue = nrmRelPos;

    if (mVelocityValue.isConnected()) {
      // The motion of link1 measured in link0
      Vector6 relVel = mRelCoordSys.motionToReference(getLink1().getRefVel());
      // The relative motion of link1 wrt link0 measured in link0
      relVel -= getLink0().getRefVel();
      // The scalar product is what we need.
      // Here the additional cross product term cancels out
      mVelocityValue = dot(mDirection, relVel.getLinear());
    }
  }
  virtual void articulation(const Task&)
  {
    if (!mForceValue.isConnected())
      return;

    // Since we assume positive input forces to push the two attached
    // RigidBodies, we need that minus sign to negate the current position
    // offset
    real_type force = mForceValue;
    Vector3 force0 = (-force)*mDirection;
    getLink0().applyBodyForce(force0);
    
    Vector3 force1 = force*mRelCoordSys.rotToLocal(mDirection);
    getLink1().applyBodyForce(force1);
  }

private:
  SharedPtr<const InternalInteract> mInternalSensor;
  RealOutputPortHandle mDistanceValue;
  RealOutputPortHandle mVelocityValue;
  RealInputPortHandle mForceValue;
  CoordinateSystem mRelCoordSys;
  Vector3 mDirection;
};

InternalInteract::InternalInteract(const std::string& name) :
  DoubleLinkInteract(name),
  mPosition0(0, 0, 0),
  mPosition1(0, 0, 0)
{
}

InternalInteract::~InternalInteract(void)
{
}

MechanicContext*
InternalInteract::newMechanicContext(const Environment* environment,
                                   PortValueList& portValueList) const
{
  return new Context(this, environment, portValueList);
}

void
InternalInteract::setPosition0(const Vector3& position)
{
  mPosition0 = position;
}

const Vector3&
InternalInteract::getPosition0() const
{
  return mPosition0;
}

void
InternalInteract::setPosition1(const Vector3& position)
{
  mPosition1 = position;
}

const Vector3&
InternalInteract::getPosition1() const
{
  return mPosition1;
}

void
InternalInteract::setEnableDistance(bool enable)
{
  if (enable == getEnableDistance())
    return;
  if (enable)
    mDistancePort = RealOutputPort(this, "distance");
  else
    mDistancePort.clear();
}

bool
InternalInteract::getEnableDistance() const
{
  return !mDistancePort.empty();
}

void
InternalInteract::setEnableVelocity(bool enable)
{
  if (enable == getEnableVelocity())
    return;
  if (enable)
    mVelocityPort = RealOutputPort(this, "velocity");
  else
    mVelocityPort.clear();
}

bool
InternalInteract::getEnableVelocity() const
{
  return !mVelocityPort.empty();
}

void
InternalInteract::setEnableForce(bool enable)
{
  if (enable == getEnableForce())
    return;
  if (enable)
    mForcePort = RealInputPort(this, "force", true);
  else
    mForcePort.clear();
}

bool
InternalInteract::getEnableForce() const
{
  return !mForcePort.empty();
}

void
InternalInteract::setEnableAllOutputs(bool enable)
{
  setEnableDistance(enable);
  setEnableVelocity(enable);
}

} // namespace OpenFDM
