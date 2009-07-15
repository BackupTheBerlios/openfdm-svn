/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "ExternalInteract.h"

#include "NumericPortValue.h"
#include "PortValueList.h"
#include "Task.h"
#include "TypeInfo.h"
#include "Variant.h"

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
    mExternalInteract(externalInteract),
    mPosition(portValueList.getPortValue(externalInteract->mPositionPort)),
    mOrientation(portValueList.getPortValue(externalInteract->mOrientationPort)),
    mEulerAngles(portValueList.getPortValue(externalInteract->mEulerAnglesPort)),
    mBodyLinearVelocity(portValueList.getPortValue(externalInteract->mBodyLinearVelocityPort)),
    mBodyAngularVelocity(portValueList.getPortValue(externalInteract->mBodyAngularVelocityPort)),
    mGlobalLinearVelocity(portValueList.getPortValue(externalInteract->mGlobalLinearVelocityPort)),
    mGlobalAngularVelocity(portValueList.getPortValue(externalInteract->mGlobalAngularVelocityPort)),
    mBodyCentrifugalAcceleration(portValueList.getPortValue(externalInteract->mBodyCentrifugalAccelerationPort)),
    mBodyLoad(portValueList.getPortValue(externalInteract->mBodyLoadPort)),
    mBodyWindVelocity(portValueList.getPortValue(externalInteract->mBodyWindVelocityPort)),
    mGlobalWindVelocity(portValueList.getPortValue(externalInteract->mGlobalWindVelocityPort)),
    mTemperature(portValueList.getPortValue(externalInteract->mTemperaturePort)),
    mPressure(portValueList.getPortValue(externalInteract->mPressurePort)),
    mDensity(portValueList.getPortValue(externalInteract->mDensityPort)),
    mSoundSpeed(portValueList.getPortValue(externalInteract->mSoundSpeedPort)),
    mAltitude(portValueList.getPortValue(externalInteract->mAltitudePort)),
    mAboveGroundLevel(portValueList.getPortValue(externalInteract->mAboveGroundLevelPort)),
    mBodyForce(portValueList.getPortValue(externalInteract->mBodyForcePort)),
    mBodyTorque(portValueList.getPortValue(externalInteract->mBodyTorquePort)),
    mGlobalForce(portValueList.getPortValue(externalInteract->mGlobalForcePort)),
    mGlobalTorque(portValueList.getPortValue(externalInteract->mGlobalTorquePort))
  { }
  virtual ~Context() {}
    
  virtual const ExternalInteract& getNode() const
  { return *mExternalInteract; }
  
  virtual void velocities(const Task& task)
  {
    const CoordinateSystem& cs = getLink().getCoordinateSystem();

    // The global coordinates position
    Vector3 refPosition = getLink().getRefPos();
    if (mPosition.isConnected())
      mPosition = refPosition;
    
    if (mOrientation.isConnected())
      mOrientation = getLink().getRefOr();
    
    if (mEulerAngles.isConnected())
      mEulerAngles = getLink().getRefOr().getEuler();
    
    // Velocity related sensing
    bool enableBodyAngularVelocity = mBodyAngularVelocity.isConnected();
    bool enableGlobalAngularVelocity = mGlobalAngularVelocity.isConnected();
    bool enableBodyLinearVelocity = mBodyLinearVelocity.isConnected();
    bool enableGlobalLinearVelocity = mGlobalLinearVelocity.isConnected();
    bool enableBodyWindVelocity = mBodyWindVelocity.isConnected();
    bool enableGlobalWindVelocity = mGlobalWindVelocity.isConnected();
    if (enableBodyAngularVelocity || enableBodyLinearVelocity
        || enableGlobalAngularVelocity || enableGlobalLinearVelocity
        || enableBodyWindVelocity || enableGlobalWindVelocity) {
      Vector6 refVelocity = getLink().getLocalVelocity();
      if (enableBodyAngularVelocity)
        mBodyAngularVelocity = refVelocity.getAngular();
      if (enableGlobalAngularVelocity)
        mGlobalAngularVelocity = cs.rotToReference(refVelocity.getAngular());
      
      if (enableBodyLinearVelocity)
        mBodyLinearVelocity = refVelocity.getLinear();
      if (enableGlobalLinearVelocity)
        mGlobalLinearVelocity = cs.rotToReference(refVelocity.getLinear());
      
      // Wind sensing
      if (enableBodyWindVelocity || enableGlobalWindVelocity) {
        Vector6 wind = getEnvironment().getWindVelocity(task.getTime(),
                                                        refPosition);
        wind = Vector6(cs.rotToLocal(wind.getAngular()),
                       cs.rotToLocal(wind.getLinear()));
        wind -= refVelocity;
        if (enableBodyWindVelocity)
          mBodyWindVelocity = wind.getLinear();
        if (enableGlobalWindVelocity)
          mGlobalWindVelocity = cs.rotToReference(wind.getLinear());
      }
    }
    
    // Atmosphere related sensing
    bool enableAltitude = mAltitude.isConnected();
    
    bool enableTemperature = mTemperature.isConnected();
    bool enablePressure = mPressure.isConnected();
    bool enableDensity = mDensity.isConnected();
    bool enableSoundSpeed = mSoundSpeed.isConnected();
    bool enableAtmosphere = (enableTemperature || enablePressure ||
                             enableDensity || enableSoundSpeed);
    if (enableAltitude || enableAtmosphere) {
      real_type altitude = getEnvironment().getAltitude(refPosition);
      if (enableAltitude)
        mAltitude = altitude;
      
      if (enableAtmosphere) {
        const AbstractAtmosphere* atmosphere = getEnvironment().getAtmosphere();
        AtmosphereData data = atmosphere->getData(task.getTime(), altitude);
        if (enableTemperature)
          mTemperature = data.temperature;
        if (enablePressure)
          mPressure = data.pressure;
        if (enableDensity)
          mDensity = data.density;
        if (enableSoundSpeed)
          mSoundSpeed = atmosphere->getSoundSpeed(data.temperature);
      }
    }
    
    if (mAboveGroundLevel.isConnected()) {
      real_type agl;
      agl = getEnvironment().getAboveGroundLevel(task.getTime(), refPosition);
      mAboveGroundLevel = agl;
    }
  }
  virtual void articulation(const Task& task)
  {
    // Apply all the forces ...
    if (mBodyForce.isConnected())
      getLink().applyBodyForce(Vector3(mBodyForce.getValue()));

    if (mBodyTorque.isConnected())
      getLink().applyBodyTorque(Vector3(mBodyTorque.getValue()));

    if (mGlobalForce.isConnected())
      getLink().applyGlobalForce(Vector3(mGlobalForce.getValue()));

    if (mGlobalTorque.isConnected())
      getLink().applyGlobalTorque(Vector3(mGlobalTorque.getValue()));
  }
  virtual void accelerations(const Task& task)
  {
    bool enableBodyCentrifugalAcceleration
      = mBodyCentrifugalAcceleration.isConnected();
    bool enableBodyLoad = mBodyLoad.isConnected();
    if (enableBodyCentrifugalAcceleration || enableBodyLoad) {
      Vector6 spatialVel = getLink().getLocalVelocity();
      Vector6 spatialAccel = getLink().getLocalAcceleration();
      Vector3 centrifugalAccel = spatialAccel.getLinear();
      centrifugalAccel += cross(spatialVel.getAngular(),spatialVel.getLinear());

      if (enableBodyCentrifugalAcceleration)
        mBodyCentrifugalAcceleration = centrifugalAccel;
      if (enableBodyLoad) {
        // May be cache that from the velocity step??
        Vector3 refPosition = getLink().getRefPos();
        Vector3 gravity = getEnvironment().getGravityAcceleration(refPosition);
        gravity = getLink().getCoordinateSystem().rotToLocal(gravity);
        mBodyLoad = centrifugalAccel - gravity;
      }
    }
  }
  
private:
  SharedPtr<const ExternalInteract> mExternalInteract;

  MatrixOutputPortHandle mPosition;
  MatrixOutputPortHandle mOrientation;
  MatrixOutputPortHandle mEulerAngles;

  MatrixOutputPortHandle mBodyLinearVelocity;
  MatrixOutputPortHandle mBodyAngularVelocity;
  MatrixOutputPortHandle mGlobalLinearVelocity;
  MatrixOutputPortHandle mGlobalAngularVelocity;

  MatrixOutputPortHandle mBodyCentrifugalAcceleration;
  MatrixOutputPortHandle mBodyLoad;

  MatrixOutputPortHandle mBodyWindVelocity;
  MatrixOutputPortHandle mGlobalWindVelocity;

  RealOutputPortHandle mTemperature;
  RealOutputPortHandle mPressure;
  RealOutputPortHandle mDensity;
  RealOutputPortHandle mSoundSpeed;

  RealOutputPortHandle mAltitude;
  RealOutputPortHandle mAboveGroundLevel;

  MatrixInputPortHandle mBodyForce;
  MatrixInputPortHandle mBodyTorque;
  MatrixInputPortHandle mGlobalForce;
  MatrixInputPortHandle mGlobalTorque;
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
