/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "InternalSensor.h"

#include "MechanicLinkValue.h"
#include "NumericPortValue.h"
#include "PortValueList.h"
#include "Task.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(InternalSensor, DoubleLinkInteract)
  DEF_OPENFDM_PROPERTY(Vector3, Position0, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, Position1, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableDistance, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableForce, Serialized)
  END_OPENFDM_OBJECT_DEF

class InternalSensor::Context : public DoubleLinkInteract::Context {
public:
  Context(const InternalSensor* internalSensor,
          const Environment* environment, PortValueList& portValueList) :
    DoubleLinkInteract::Context(internalSensor, environment, portValueList),
    mInternalSensor(internalSensor),
    mLinkRelPos0(Vector3::zeros()),
    mLinkRelPos1(Vector3::zeros())
  {
    mDistanceValue = portValueList.getPortValue(internalSensor->mDistancePort);
    mVelocityValue = portValueList.getPortValue(internalSensor->mVelocityPort);
    mForceValue = portValueList.getPortValue(internalSensor->mForcePort);
  }
  virtual ~Context() {}
    
  virtual const InternalSensor& getNode() const
  { return *mInternalSensor; }

  virtual void initDesignPosition()
  {
    mLinkRelPos0 = mInternalSensor->getPosition0();
    mLinkRelPos0 -= getLink0().getDesignPosition();
    mLinkRelPos1 = mInternalSensor->getPosition1();
    mLinkRelPos1 -= getLink1().getDesignPosition();
  }
  virtual void velocities(const Task&)
  {
    mRelCoordSys = getLink0().getRelativeCoordinateSystem(getLink1());
  
    Vector3 relPos = mRelCoordSys.toReference(mLinkRelPos1) - mLinkRelPos0;
    real_type nrmRelPos = norm(relPos);
    if (nrmRelPos <= Limits<real_type>::min())
      mDirection = Vector3::zeros();
    else
      mDirection = (1/nrmRelPos)*relPos;

    // The relative distance of these two points
    if (mDistanceValue)
      mDistanceValue->getValue()(0, 0) = nrmRelPos;

    if (mVelocityValue) {
      // The motion of link1 measured in link0
      Vector6 relVel = mRelCoordSys.motionToReference(getLink1().getSpVel());
      // The relative motion of link1 wrt link0 measured in link0
      relVel -= getLink0().getSpVel();
      // Transform to the internal reference point
      relVel = motionTo(mLinkRelPos0, relVel);
      // The scalar product is what we need.
      // Here the additional cross product term cancels out
      mVelocityValue->getValue()(0, 0) = dot(mDirection, relVel.getLinear());
    }
  }
  virtual void articulation(const Task&)
  {
    if (!mForceValue)
      return;

    // Since we assume positive input forces to push the two attached
    // RigidBodies, we need that minus sign to negate the current position
    // offset
    real_type force = mForceValue->getValue()(0, 0);
    Vector3 force0 = (-force)*mDirection;
    getLink0().applyForce(mLinkRelPos0, force0);
    
    Vector3 force1 = force*mRelCoordSys.rotToLocal(mDirection);
    getLink1().applyForce(mLinkRelPos1, force1);
  }

private:
  SharedPtr<const InternalSensor> mInternalSensor;
  SharedPtr<NumericPortValue> mDistanceValue;
  SharedPtr<NumericPortValue> mVelocityValue;
  SharedPtr<const NumericPortValue> mForceValue;
  Vector3 mLinkRelPos0;
  Vector3 mLinkRelPos1;
  CoordinateSystem mRelCoordSys;
  Vector3 mDirection;
};

InternalSensor::InternalSensor(const std::string& name) :
  DoubleLinkInteract(name),
  mPosition0(0, 0, 0),
  mPosition1(0, 0, 0)
{
}

InternalSensor::~InternalSensor(void)
{
}

MechanicContext*
InternalSensor::newMechanicContext(const Environment* environment,
                                   PortValueList& portValueList) const
{
  return new Context(this, environment, portValueList);
}

void
InternalSensor::setPosition0(const Vector3& position)
{
  mPosition0 = position;
}

const Vector3&
InternalSensor::getPosition0() const
{
  return mPosition0;
}

void
InternalSensor::setPosition1(const Vector3& position)
{
  mPosition1 = position;
}

const Vector3&
InternalSensor::getPosition1() const
{
  return mPosition1;
}

void
InternalSensor::setEnableDistance(bool enable)
{
  if (enable == getEnableDistance())
    return;
  if (enable)
    mDistancePort = RealOutputPort(this, "distance");
  else
    mDistancePort.clear();
}

bool
InternalSensor::getEnableDistance() const
{
  return !mDistancePort.empty();
}

void
InternalSensor::setEnableVelocity(bool enable)
{
  if (enable == getEnableVelocity())
    return;
  if (enable)
    mVelocityPort = RealOutputPort(this, "velocity");
  else
    mVelocityPort.clear();
}

bool
InternalSensor::getEnableVelocity() const
{
  return !mVelocityPort.empty();
}

void
InternalSensor::setEnableForce(bool enable)
{
  if (enable == getEnableForce())
    return;
  if (enable)
    mForcePort = RealInputPort(this, "force", true);
  else
    mForcePort.clear();
}

bool
InternalSensor::getEnableForce() const
{
  return !mForcePort.empty();
}

void
InternalSensor::setEnableAllOutputs(bool enable)
{
  setEnableDistance(enable);
  setEnableVelocity(enable);
}

} // namespace OpenFDM
