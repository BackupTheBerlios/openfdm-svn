/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Sensor.h"

#include "MechanicLinkValue.h"
#include "NumericPortValue.h"
#include "PortValueList.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Sensor, Interact)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnablePosition, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableOrientation, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableEulerAngles, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableLinearVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableAngularVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableCentrifugalAcceleration, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableTemperature, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnablePressure, Serialized)
  END_OPENFDM_OBJECT_DEF

Sensor::Sensor(const std::string& name) :
  Interact(name),
  mMechanicLink(newMechanicLink("link")),
  mPosition(0, 0, 0)
{
}

Sensor::~Sensor(void)
{
}

void
Sensor::initDesignPosition(PortValueList& portValues) const
{
}

void
Sensor::velocity(const Task&, const ContinousStateValueVector&,
                 PortValueList& portValues) const
{
  const Environment* environment;
  environment = portValues[mMechanicLink].getEnvironment();
  const Frame& frame = portValues[mMechanicLink].getFrame();

  // FIXME, for now relative position
  Vector3 position = mPosition - portValues[mMechanicLink].getDesignPosition();
  Vector3 refPosition = frame.posToRef(position);

  if (getEnablePosition())
    portValues[mPositionPort] = refPosition;

  if (getEnableOrientation())
    portValues[mOrientationPort] = frame.getRefOrientation();

  if (getEnableEulerAngles())
    portValues[mEulerAnglesPort] = frame.getRefOrientation().getEuler();

  Vector6 refVelocity = frame.getRefVelAt(position);
  if (getEnableAngularVelocity())
    portValues[mAngularVelocityPort] = refVelocity.getAngular();

  if (getEnableLinearVelocity())
    portValues[mLinearVelocityPort] = refVelocity.getLinear();

  // Atmosphere related sensing
  bool enableTemperature = getEnableTemperature();
  bool enablePressure = getEnablePressure();
  if (enableTemperature || enablePressure) {
    AtmosphereData data = environment->getAtmosphereData(refPosition);
    if (enableTemperature)
      portValues[mTemperaturePort] = data.temperature;
    if (enablePressure)
      portValues[mPressurePort] = data.pressure;
  }
}

void
Sensor::acceleration(const Task&, const ContinousStateValueVector&,
                     PortValueList& portValues, const Matrix&, Vector&) const
{
  const Frame& frame = portValues[mMechanicLink].getFrame();

  // FIXME, for now relative position
  Vector3 position = mPosition - portValues[mMechanicLink].getDesignPosition();

  if (getEnableCentrifugalAcceleration()) {
    Vector6 spatialVel = motionTo(position, frame.getSpVel());
    Vector6 spatialAccel = motionTo(position, frame.getSpAccel());
    Vector3 centrifugalAccel = spatialAccel.getLinear();
    centrifugalAccel += cross(spatialVel.getAngular(), spatialVel.getLinear());

    portValues[mCentrifugalAccelerationPort] = centrifugalAccel;
  }

//   if (getEnableLoadAcceleration())
//     portValues[mLoadAccelerationPort] = centrifugalAccel + gravity;
}

void
Sensor::setPosition(const Vector3& position)
{
  mPosition = position;
}

const Vector3&
Sensor::getPosition() const
{
  return mPosition;
}

void
Sensor::setEnablePosition(bool enable)
{
  if (enable == getEnablePosition())
    return;
  if (enable)
    mPositionPort = MatrixOutputPort(this, "position", Size(3, 1));
  else
    mPositionPort.clear();
}

bool
Sensor::getEnablePosition() const
{
  return !mPositionPort.empty();
}

void
Sensor::setEnableOrientation(bool enable)
{
  if (enable == getEnableOrientation())
    return;
  if (enable)
    mOrientationPort = MatrixOutputPort(this, "orientation", Size(4, 1));
  else
    mOrientationPort.clear();
}

bool
Sensor::getEnableOrientation() const
{
  return !mOrientationPort.empty();
}

void
Sensor::setEnableEulerAngles(bool enable)
{
  if (enable == getEnableEulerAngles())
    return;
  if (enable)
    mEulerAnglesPort = MatrixOutputPort(this, "eulerAngles", Size(3, 1));
  else
    mEulerAnglesPort.clear();
}

bool
Sensor::getEnableEulerAngles() const
{
  return !mEulerAnglesPort.empty();
}

void
Sensor::setEnableLinearVelocity(bool enable)
{
  if (enable == getEnableLinearVelocity())
    return;
  if (enable)
    mLinearVelocityPort = MatrixOutputPort(this, "linearVelocity", Size(3, 1));
  else
    mLinearVelocityPort.clear();
}

bool
Sensor::getEnableLinearVelocity() const
{
  return !mLinearVelocityPort.empty();
}

void
Sensor::setEnableAngularVelocity(bool enable)
{
  if (enable == getEnableAngularVelocity())
    return;
  if (enable)
    mAngularVelocityPort
      = MatrixOutputPort(this, "angularVelocity", Size(3, 1));
  else
    mAngularVelocityPort.clear();
}

bool
Sensor::getEnableAngularVelocity() const
{
  return !mAngularVelocityPort.empty();
}

void
Sensor::setEnableCentrifugalAcceleration(bool enable)
{
  if (enable == getEnableCentrifugalAcceleration())
    return;
  if (enable)
    mCentrifugalAccelerationPort
      = MatrixOutputPort(this, "centrifugalAcceleration", Size(3, 1), true);
  else
    mCentrifugalAccelerationPort.clear();
}

bool
Sensor::getEnableCentrifugalAcceleration() const
{
  return !mCentrifugalAccelerationPort.empty();
}

void
Sensor::setEnableTemperature(bool enable)
{
  if (enable == getEnableTemperature())
    return;
  if (enable)
    mTemperaturePort = RealOutputPort(this, "temperature");
  else
    mTemperaturePort.clear();
}

bool
Sensor::getEnableTemperature() const
{
  return !mTemperaturePort.empty();
}

void
Sensor::setEnablePressure(bool enable)
{
  if (enable == getEnablePressure())
    return;
  if (enable)
    mPressurePort = RealOutputPort(this, "pressure");
  else
    mPressurePort.clear();
}

bool
Sensor::getEnablePressure() const
{
  return !mPressurePort.empty();
}

} // namespace OpenFDM
