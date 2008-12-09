/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "InternalSensor.h"

#include "MechanicLinkValue.h"
#include "NumericPortValue.h"
#include "PortValueList.h"
#include "Task.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(InternalSensor, Interact)
  DEF_OPENFDM_PROPERTY(Vector3, Position0, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, Position1, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableDistance, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableVelocity, Serialized)
  END_OPENFDM_OBJECT_DEF

InternalSensor::InternalSensor(const std::string& name) :
  Interact(name),
  mMechanicLink0(newMechanicLink("link0")),
  mMechanicLink1(newMechanicLink("link1")),
  mPosition0(0, 0, 0),
  mPosition1(0, 0, 0)
{
}

InternalSensor::~InternalSensor(void)
{
}

void
InternalSensor::velocity(const Task& task, const ContinousStateValueVector&,
                         PortValueList& portValues) const
{
  const Frame& frame0 = portValues[mMechanicLink0].getFrame();
  const Frame& frame1 = portValues[mMechanicLink1].getFrame();

  // FIXME, for now relative position
  Vector3 position0 = mPosition0-portValues[mMechanicLink0].getDesignPosition();
  Vector3 position1 = mPosition1-portValues[mMechanicLink1].getDesignPosition();

  bool enableDistance = getEnableDistance();
  bool enableVelocity = getEnableVelocity();
  if (enableDistance || enableVelocity) {
    Vector3 relPos = frame0.posFromRef(frame1.posToRef(position1)) - position0;
    real_type nrmRelPos = norm(relPos);

    // The relative distance of these two points
    if (enableDistance)
      portValues[mDistancePort] = nrmRelPos;

    if (enableVelocity) {
      /// FIXME: avoid that transform to the reference frame. The relative
      /// position must be sufficient ...
      Vector6 refVel1 = frame1.motionToRef(motionTo(position1, frame1.getRefVel()));
      Vector6 refVel0 = motionTo(position0, frame0.motionFromRef(refVel1) - frame0.getRefVel());

      Vector3 relVel = refVel0.getLinear();
      if (nrmRelPos <= Limits<real_type>::min())
        portValues[mVelocityPort] = 0;
      else
        portValues[mVelocityPort] = dot(relPos, relVel)/nrmRelPos;
    }
  }
}

void
InternalSensor::articulation(const Task& task, const ContinousStateValueVector&,
                             PortValueList& portValues) const
{
  if (getEnableForce()) {
    const Frame& frame0 = portValues[mMechanicLink0].getFrame();
    const Frame& frame1 = portValues[mMechanicLink1].getFrame();

    // FIXME, for now relative position
    Vector3 position0=mPosition0-portValues[mMechanicLink0].getDesignPosition();
    Vector3 position1=mPosition1-portValues[mMechanicLink1].getDesignPosition();
    
    // FIXME, already have that computed in the velocity step
    Vector3 relPos = frame0.posFromRef(frame1.posToRef(position1)) - position0;
    real_type nrmRelPos = norm(relPos);

    // If we have reached the zero position, the force vector is undefined.
    if (Limits<real_type>::min() < nrmRelPos) {
      Vector3 dir = (1/nrmRelPos)*relPos;
      // Since we assume positive input forces to push the two attached
      // RigidBodies, we need that minus sign to negate the current position
      // offset
      Vector3 force0 = portValues[mForcePort]*dir;
      portValues[mMechanicLink0].applyForce(forceFrom(position0, force0));
      
      Vector3 force1 = -frame0.getRelOrientation(frame1).transform(force0);
      portValues[mMechanicLink1].applyForce(forceFrom(position1, force1));
    }
  }
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
InternalSensor::setEnableAll(bool enable)
{
  setEnableDistance(enable);
  setEnableVelocity(enable);
}

} // namespace OpenFDM
