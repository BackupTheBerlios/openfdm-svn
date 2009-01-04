/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "ExternalInteract.h"

#include "MechanicLinkValue.h"
#include "NumericPortValue.h"
#include "PortValueList.h"
#include "Task.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(ExternalInteract, SingleLinkInteract)
  DEF_OPENFDM_PROPERTY(Bool, EnablePosition, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableOrientation, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableEulerAngles, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableLinearVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableAngularVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableCentrifugalAcceleration, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableWindVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableTemperature, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnablePressure, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableDensity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableSoundSpeed, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableAltitude, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableAboveGroundLevel, Serialized)
  END_OPENFDM_OBJECT_DEF

class ExternalInteract::Context : public SingleLinkInteract::Context {
public:
  Context(const ExternalInteract* externalInteract,
          const Environment* environment, PortValueList& portValueList) :
    SingleLinkInteract::Context(externalInteract, environment, portValueList),
    mExternalInteract(externalInteract)
  { }
  virtual ~Context() {}
    
  virtual const ExternalInteract& getNode() const
  { return *mExternalInteract; }
  
  virtual void velocities(const Task& task)
  {
    mExternalInteract->velocity(task, getEnvironment(), mContinousState, mPortValueList);
  }
  virtual void articulation(const Task& task)
  {
    mExternalInteract->articulation(task, getEnvironment(), mContinousState, mPortValueList);
  }
  virtual void accelerations(const Task& task)
  {
    mExternalInteract->acceleration(task, getEnvironment(), mContinousState, mPortValueList);
  }
  
private:
  SharedPtr<const ExternalInteract> mExternalInteract;
};

ExternalInteract::ExternalInteract(const std::string& name) :
  SingleLinkInteract(name)
{
}

ExternalInteract::~ExternalInteract(void)
{
}

MechanicContext*
ExternalInteract::newMechanicContext(const Environment* environment,
                           PortValueList& portValueList) const
{
  SharedPtr<Context> context = new Context(this, environment, portValueList);
  if (!context->alloc()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return 0;
  }
  return context.release();
}

void
ExternalInteract::velocity(const Task& task, const Environment& environment,
                 const ContinousStateValueVector&,
                 PortValueList& portValues) const
{
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

  // Velocity related sensing
  bool enableAngularVelocity = getEnableAngularVelocity();
  bool enableLinearVelocity = getEnableLinearVelocity();
  bool enableWindVelocity = getEnableWindVelocity();
  if (enableAngularVelocity || enableLinearVelocity || enableWindVelocity) {
    Vector6 refVelocity = frame.getRefVelAt(position);
    if (enableAngularVelocity)
      portValues[mAngularVelocityPort] = refVelocity.getAngular();
    
    if (enableLinearVelocity)
      portValues[mLinearVelocityPort] = refVelocity.getLinear();

    // Wind sensing
    if (enableWindVelocity) {
      Vector6 wind = environment.getWindVelocity(task.getTime(), position);
      wind -= refVelocity;
      portValues[mWindVelocityPort] = frame.rotFromRef(wind.getLinear());
    }
  }

  // Atmosphere related sensing
  bool enableAltitude = getEnableAltitude();
  
  bool enableTemperature = getEnableTemperature();
  bool enablePressure = getEnablePressure();
  bool enableDensity = getEnableDensity();
  bool enableSoundSpeed = getEnableSoundSpeed();
  bool enableAtmosphere = (enableTemperature || enablePressure ||
                           enableDensity || enableSoundSpeed);
  if (enableAltitude || enableAtmosphere) {
    real_type altitude = environment.getAltitude(refPosition);
    if (enableAltitude)
      portValues[mAltitudePort] = altitude;

    if (enableAtmosphere) {
      const AbstractAtmosphere* atmosphere = environment.getAtmosphere();
      AtmosphereData data = atmosphere->getData(task.getTime(), altitude);
      if (enableTemperature)
        portValues[mTemperaturePort] = data.temperature;
      if (enablePressure)
        portValues[mPressurePort] = data.pressure;
      if (enableDensity)
        portValues[mDensityPort] = data.density;
      if (enableSoundSpeed)
        portValues[mSoundSpeedPort]
          = atmosphere->getSoundSpeed(data.temperature);
    }
  }

  if (getEnableAboveGroundLevel()) {
    real_type agl;
    agl = environment.getAboveGroundLevel(task.getTime(), refPosition);
    portValues[mAboveGroundLevelPort] = agl;
  }
}

void
ExternalInteract::articulation(const Task&, const Environment&,
                     const ContinousStateValueVector&,
                     PortValueList& portValues) const
{
  const Frame& frame = portValues[mMechanicLink].getFrame();
  // FIXME, for now relative position
  Vector3 position = mPosition - portValues[mMechanicLink].getDesignPosition();
  
  if (getEnableBodyForce()) {
    Vector3 force = portValues[mBodyForcePort];
    portValues[mMechanicLink].applyForce(position, force);
  }
  if (getEnableBodyTorque()) {
    Vector3 torque = portValues[mBodyTorquePort];
    portValues[mMechanicLink].applyTorque(torque);
  }
  if (getEnableGlobalForce()) {
    Vector3 force = portValues[mGlobalForcePort];
    portValues[mMechanicLink].applyForce(position, frame.rotFromRef(force));
  }
  if (getEnableGlobalTorque()) {
    Vector3 torque = portValues[mGlobalTorquePort];
    portValues[mMechanicLink].applyTorque(frame.rotFromRef(torque));
  }
}

void
ExternalInteract::acceleration(const Task&, const Environment& environment,
                     const ContinousStateValueVector&,
                     PortValueList& portValues) const
{
  const Frame& frame = portValues[mMechanicLink].getFrame();

  // FIXME, for now relative position
  Vector3 position = mPosition - portValues[mMechanicLink].getDesignPosition();

  bool enableCentrifugalAcceleration = getEnableCentrifugalAcceleration();
  bool enableLoad = getEnableLoad();
  if (enableCentrifugalAcceleration || enableLoad) {
    Vector6 spatialVel = motionTo(position, frame.getSpVel());
    Vector6 spatialAccel = motionTo(position, frame.getSpAccel());
    Vector3 centrifugalAccel = spatialAccel.getLinear();
    centrifugalAccel += cross(spatialVel.getAngular(), spatialVel.getLinear());

    if (enableCentrifugalAcceleration)
      portValues[mCentrifugalAccelerationPort] = centrifugalAccel;
    if (enableLoad) {
      // May be cache that from the velocity step??
      Vector3 refPosition = frame.posToRef(position);
      Vector3 gravity = environment.getGravityAcceleration(refPosition);
      gravity = frame.rotFromRef(gravity);
      portValues[mLoadPort] = centrifugalAccel - gravity;
    }
  }
}

void
ExternalInteract::setEnablePosition(bool enable)
{
  if (enable == getEnablePosition())
    return;
  if (enable)
    mPositionPort = MatrixOutputPort(this, "position", Size(3, 1));
  else
    mPositionPort.clear();
}

bool
ExternalInteract::getEnablePosition() const
{
  return !mPositionPort.empty();
}

void
ExternalInteract::setEnableOrientation(bool enable)
{
  if (enable == getEnableOrientation())
    return;
  if (enable)
    mOrientationPort = MatrixOutputPort(this, "orientation", Size(4, 1));
  else
    mOrientationPort.clear();
}

bool
ExternalInteract::getEnableOrientation() const
{
  return !mOrientationPort.empty();
}

void
ExternalInteract::setEnableEulerAngles(bool enable)
{
  if (enable == getEnableEulerAngles())
    return;
  if (enable)
    mEulerAnglesPort = MatrixOutputPort(this, "eulerAngles", Size(3, 1));
  else
    mEulerAnglesPort.clear();
}

bool
ExternalInteract::getEnableEulerAngles() const
{
  return !mEulerAnglesPort.empty();
}

void
ExternalInteract::setEnableLinearVelocity(bool enable)
{
  if (enable == getEnableLinearVelocity())
    return;
  if (enable)
    mLinearVelocityPort = MatrixOutputPort(this, "linearVelocity", Size(3, 1));
  else
    mLinearVelocityPort.clear();
}

bool
ExternalInteract::getEnableLinearVelocity() const
{
  return !mLinearVelocityPort.empty();
}

void
ExternalInteract::setEnableAngularVelocity(bool enable)
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
ExternalInteract::getEnableAngularVelocity() const
{
  return !mAngularVelocityPort.empty();
}

void
ExternalInteract::setEnableCentrifugalAcceleration(bool enable)
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
ExternalInteract::getEnableCentrifugalAcceleration() const
{
  return !mCentrifugalAccelerationPort.empty();
}

void
ExternalInteract::setEnableLoad(bool enable)
{
  if (enable == getEnableLoad())
    return;
  if (enable)
    mLoadPort = MatrixOutputPort(this, "load", Size(3, 1), true);
  else
    mLoadPort.clear();
}

bool
ExternalInteract::getEnableLoad() const
{
  return !mLoadPort.empty();
}

void
ExternalInteract::setEnableWindVelocity(bool enable)
{
  if (enable == getEnableWindVelocity())
    return;
  if (enable)
    mWindVelocityPort = MatrixOutputPort(this, "windVelocity", Size(3, 1));
  else
    mWindVelocityPort.clear();
}

bool
ExternalInteract::getEnableWindVelocity() const
{
  return !mWindVelocityPort.empty();
}

void
ExternalInteract::setEnableTemperature(bool enable)
{
  if (enable == getEnableTemperature())
    return;
  if (enable)
    mTemperaturePort = RealOutputPort(this, "temperature");
  else
    mTemperaturePort.clear();
}

bool
ExternalInteract::getEnableTemperature() const
{
  return !mTemperaturePort.empty();
}

void
ExternalInteract::setEnablePressure(bool enable)
{
  if (enable == getEnablePressure())
    return;
  if (enable)
    mPressurePort = RealOutputPort(this, "pressure");
  else
    mPressurePort.clear();
}

bool
ExternalInteract::getEnablePressure() const
{
  return !mPressurePort.empty();
}

void
ExternalInteract::setEnableDensity(bool enable)
{
  if (enable == getEnableDensity())
    return;
  if (enable)
    mDensityPort = RealOutputPort(this, "density");
  else
    mDensityPort.clear();
}

bool
ExternalInteract::getEnableDensity() const
{
  return !mDensityPort.empty();
}

void
ExternalInteract::setEnableSoundSpeed(bool enable)
{
  if (enable == getEnableSoundSpeed())
    return;
  if (enable)
    mSoundSpeedPort = RealOutputPort(this, "soundSpeed");
  else
    mSoundSpeedPort.clear();
}

bool
ExternalInteract::getEnableSoundSpeed() const
{
  return !mSoundSpeedPort.empty();
}

void
ExternalInteract::setEnableAltitude(bool enable)
{
  if (enable == getEnableAltitude())
    return;
  if (enable)
    mAltitudePort = RealOutputPort(this, "altitude");
  else
    mAltitudePort.clear();
}

bool
ExternalInteract::getEnableAltitude() const
{
  return !mAltitudePort.empty();
}

void
ExternalInteract::setEnableAboveGroundLevel(bool enable)
{
  if (enable == getEnableAboveGroundLevel())
    return;
  if (enable)
    mAboveGroundLevelPort = RealOutputPort(this, "aboveGroundLevel");
  else
    mAboveGroundLevelPort.clear();
}

bool
ExternalInteract::getEnableAboveGroundLevel() const
{
  return !mAboveGroundLevelPort.empty();
}

void
ExternalInteract::setEnableBodyForce(bool enable)
{
  if (enable == getEnableBodyForce())
    return;
  if (enable)
    mBodyForcePort = MatrixInputPort(this, "bodyForce", Size(3, 1), true);
  else
    mBodyForcePort.clear();
}

bool
ExternalInteract::getEnableBodyForce() const
{
  return !mBodyForcePort.empty();
}

void
ExternalInteract::setEnableBodyTorque(bool enable)
{
  if (enable == getEnableBodyTorque())
    return;
  if (enable)
    mBodyTorquePort = MatrixInputPort(this, "bodyTorque", Size(3, 1), true);
  else
    mBodyTorquePort.clear();
}

bool
ExternalInteract::getEnableBodyTorque() const
{
  return !mBodyTorquePort.empty();
}

void
ExternalInteract::setEnableGlobalForce(bool enable)
{
  if (enable == getEnableGlobalForce())
    return;
  if (enable)
    mGlobalForcePort = MatrixInputPort(this, "globalForce", Size(3, 1), true);
  else
    mGlobalForcePort.clear();
}

bool
ExternalInteract::getEnableGlobalForce() const
{
  return !mGlobalForcePort.empty();
}

void
ExternalInteract::setEnableGlobalTorque(bool enable)
{
  if (enable == getEnableGlobalTorque())
    return;
  if (enable)
    mGlobalTorquePort = MatrixInputPort(this, "globalTorque", Size(3, 1), true);
  else
    mGlobalTorquePort.clear();
}

bool
ExternalInteract::getEnableGlobalTorque() const
{
  return !mGlobalTorquePort.empty();
}

void
ExternalInteract::setEnableAllOutputs(bool enable)
{
  setEnablePosition(enable);
  setEnableOrientation(enable);
  setEnableEulerAngles(enable);
  setEnableLinearVelocity(enable);
  setEnableAngularVelocity(enable);
  setEnableCentrifugalAcceleration(enable);
  setEnableLoad(enable);
  setEnableWindVelocity(enable);
  setEnableTemperature(enable);
  setEnablePressure(enable);
  setEnableDensity(enable);
  setEnableSoundSpeed(enable);
  setEnableAltitude(enable);
  setEnableAboveGroundLevel(enable);
}

} // namespace OpenFDM
