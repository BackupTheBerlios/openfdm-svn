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
  DEF_OPENFDM_PROPERTY(Bool, EnableBodyLinearVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableBodyAngularVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableGlobalLinearVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableGlobalAngularVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableBodyCentrifugalAcceleration, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableBodyLoad, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableBodyWindVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableGlobalWindVelocity, Serialized)
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
  bool enableBodyAngularVelocity = getEnableBodyAngularVelocity();
  bool enableGlobalAngularVelocity = getEnableGlobalAngularVelocity();
  bool enableBodyLinearVelocity = getEnableBodyLinearVelocity();
  bool enableGlobalLinearVelocity = getEnableGlobalLinearVelocity();
  bool enableBodyWindVelocity = getEnableBodyWindVelocity();
  bool enableGlobalWindVelocity = getEnableGlobalWindVelocity();
  if (enableBodyAngularVelocity || enableBodyLinearVelocity
      || enableGlobalAngularVelocity || enableGlobalLinearVelocity
      || enableBodyWindVelocity || enableGlobalWindVelocity) {
    Vector6 refVelocity = motionTo(position, frame.getRefVel());
    if (enableBodyAngularVelocity)
      portValues[mBodyAngularVelocityPort] = refVelocity.getAngular();
    if (enableGlobalAngularVelocity)
      portValues[mGlobalAngularVelocityPort]
        = frame.rotToRef(refVelocity.getAngular());
    
    if (enableBodyLinearVelocity)
      portValues[mBodyLinearVelocityPort] = refVelocity.getLinear();
    if (enableGlobalLinearVelocity)
      portValues[mGlobalLinearVelocityPort]
        = frame.rotToRef(refVelocity.getLinear());

    // Wind sensing
    if (enableBodyWindVelocity || enableGlobalWindVelocity) {
      Vector6 wind = environment.getWindVelocity(task.getTime(), position);
      wind = Vector6(frame.rotFromRef(wind.getAngular()),
                     frame.rotFromRef(wind.getLinear()));
      wind -= refVelocity;
      if (enableBodyWindVelocity)
        portValues[mBodyWindVelocityPort] = wind.getLinear();
      if (enableGlobalWindVelocity)
        portValues[mGlobalWindVelocityPort] = frame.rotToRef(wind.getLinear());
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

  bool enableBodyCentrifugalAcceleration=getEnableBodyCentrifugalAcceleration();
  bool enableBodyLoad = getEnableBodyLoad();
  if (enableBodyCentrifugalAcceleration || enableBodyLoad) {
    Vector6 spatialVel = motionTo(position, frame.getSpVel());
    Vector6 spatialAccel = motionTo(position, frame.getSpAccel());
    Vector3 centrifugalAccel = spatialAccel.getLinear();
    centrifugalAccel += cross(spatialVel.getAngular(), spatialVel.getLinear());

    if (enableBodyCentrifugalAcceleration)
      portValues[mBodyCentrifugalAccelerationPort] = centrifugalAccel;
    if (enableBodyLoad) {
      // May be cache that from the velocity step??
      Vector3 refPosition = frame.posToRef(position);
      Vector3 gravity = environment.getGravityAcceleration(refPosition);
      gravity = frame.rotFromRef(gravity);
      portValues[mBodyLoadPort] = centrifugalAccel - gravity;
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
ExternalInteract::setEnableBodyLinearVelocity(bool enable)
{
  if (enable == getEnableBodyLinearVelocity())
    return;
  if (enable)
    mBodyLinearVelocityPort
      = MatrixOutputPort(this, "bodyLinearVelocity", Size(3, 1));
  else
    mBodyLinearVelocityPort.clear();
}

bool
ExternalInteract::getEnableBodyLinearVelocity() const
{
  return !mBodyLinearVelocityPort.empty();
}

void
ExternalInteract::setEnableBodyAngularVelocity(bool enable)
{
  if (enable == getEnableBodyAngularVelocity())
    return;
  if (enable)
    mBodyAngularVelocityPort
      = MatrixOutputPort(this, "bodyAngularVelocity", Size(3, 1));
  else
    mBodyAngularVelocityPort.clear();
}

bool
ExternalInteract::getEnableBodyAngularVelocity() const
{
  return !mBodyAngularVelocityPort.empty();
}

void
ExternalInteract::setEnableGlobalLinearVelocity(bool enable)
{
  if (enable == getEnableGlobalLinearVelocity())
    return;
  if (enable)
    mGlobalLinearVelocityPort
      = MatrixOutputPort(this, "globalLinearVelocity", Size(3, 1));
  else
    mGlobalLinearVelocityPort.clear();
}

bool
ExternalInteract::getEnableGlobalLinearVelocity() const
{
  return !mGlobalLinearVelocityPort.empty();
}

void
ExternalInteract::setEnableGlobalAngularVelocity(bool enable)
{
  if (enable == getEnableGlobalAngularVelocity())
    return;
  if (enable)
    mGlobalAngularVelocityPort
      = MatrixOutputPort(this, "globalAngularVelocity", Size(3, 1));
  else
    mGlobalAngularVelocityPort.clear();
}

bool
ExternalInteract::getEnableGlobalAngularVelocity() const
{
  return !mGlobalAngularVelocityPort.empty();
}

void
ExternalInteract::setEnableBodyCentrifugalAcceleration(bool enable)
{
  if (enable == getEnableBodyCentrifugalAcceleration())
    return;
  if (enable)
    mBodyCentrifugalAccelerationPort
      = MatrixOutputPort(this, "bodyCentrifugalAcceleration", Size(3, 1), true);
  else
    mBodyCentrifugalAccelerationPort.clear();
}

bool
ExternalInteract::getEnableBodyCentrifugalAcceleration() const
{
  return !mBodyCentrifugalAccelerationPort.empty();
}

void
ExternalInteract::setEnableBodyLoad(bool enable)
{
  if (enable == getEnableBodyLoad())
    return;
  if (enable)
    mBodyLoadPort = MatrixOutputPort(this, "bodyLoad", Size(3, 1), true);
  else
    mBodyLoadPort.clear();
}

bool
ExternalInteract::getEnableBodyLoad() const
{
  return !mBodyLoadPort.empty();
}

void
ExternalInteract::setEnableBodyWindVelocity(bool enable)
{
  if (enable == getEnableBodyWindVelocity())
    return;
  if (enable)
    mBodyWindVelocityPort
      = MatrixOutputPort(this, "bodyWindVelocity", Size(3, 1));
  else
    mBodyWindVelocityPort.clear();
}

bool
ExternalInteract::getEnableBodyWindVelocity() const
{
  return !mBodyWindVelocityPort.empty();
}

void
ExternalInteract::setEnableGlobalWindVelocity(bool enable)
{
  if (enable == getEnableGlobalWindVelocity())
    return;
  if (enable)
    mGlobalWindVelocityPort
      = MatrixOutputPort(this, "globalWindVelocity", Size(3, 1));
  else
    mGlobalWindVelocityPort.clear();
}

bool
ExternalInteract::getEnableGlobalWindVelocity() const
{
  return !mGlobalWindVelocityPort.empty();
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
  setEnableBodyLinearVelocity(enable);
  setEnableBodyAngularVelocity(enable);
  setEnableGlobalLinearVelocity(enable);
  setEnableGlobalAngularVelocity(enable);
  setEnableBodyCentrifugalAcceleration(enable);
  setEnableBodyLoad(enable);
  setEnableBodyWindVelocity(enable);
  setEnableGlobalWindVelocity(enable);
  setEnableTemperature(enable);
  setEnablePressure(enable);
  setEnableDensity(enable);
  setEnableSoundSpeed(enable);
  setEnableAltitude(enable);
  setEnableAboveGroundLevel(enable);
}

} // namespace OpenFDM
