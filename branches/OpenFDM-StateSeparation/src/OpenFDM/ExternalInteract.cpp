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
  DEF_OPENFDM_PROPERTY(Quaternion, Orientation, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnablePosition, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableOrientation, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableEulerAngles, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableLinearVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableAngularVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableCentrifugalAcceleration, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableLoad, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableAngularAcceleration, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableLinearWindVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableAngularWindVelocity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableGroundSpeed, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableClimbSpeed, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableTemperature, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableStaticPressure, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableDensity, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableSoundSpeed, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableSpecificHeatRatio, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableAltitude, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableAboveGroundLevel, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableForce, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableTorque, Serialized)
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
    mLinearVelocity(portValueList.getPortValue(externalInteract->mLinearVelocityPort)),
    mAngularVelocity(portValueList.getPortValue(externalInteract->mAngularVelocityPort)),
    mCentrifugalAcceleration(portValueList.getPortValue(externalInteract->mCentrifugalAccelerationPort)),
    mLoad(portValueList.getPortValue(externalInteract->mLoadPort)),
    mAngularAcceleration(portValueList.getPortValue(externalInteract->mAngularAccelerationPort)),
    mLinearWindVelocity(portValueList.getPortValue(externalInteract->mLinearWindVelocityPort)),
    mAngularWindVelocity(portValueList.getPortValue(externalInteract->mAngularWindVelocityPort)),
    mGroundSpeed(portValueList.getPortValue(externalInteract->mGroundSpeedPort)),
    mClimbSpeed(portValueList.getPortValue(externalInteract->mClimbSpeedPort)),
    mTemperature(portValueList.getPortValue(externalInteract->mTemperaturePort)),
    mStaticPressure(portValueList.getPortValue(externalInteract->mStaticPressurePort)),
    mDensity(portValueList.getPortValue(externalInteract->mDensityPort)),
    mSoundSpeed(portValueList.getPortValue(externalInteract->mSoundSpeedPort)),
    mSpecificHeatRatio(portValueList.getPortValue(externalInteract->mSpecificHeatRatioPort)),
    mAltitude(portValueList.getPortValue(externalInteract->mAltitudePort)),
    mAboveGroundLevel(portValueList.getPortValue(externalInteract->mAboveGroundLevelPort)),
    mForce(portValueList.getPortValue(externalInteract->mForcePort)),
    mTorque(portValueList.getPortValue(externalInteract->mTorquePort))
  { }
  virtual ~Context() {}
    
  virtual const ExternalInteract& getNode() const
  { return *mExternalInteract; }
  
  virtual void init(const /*Init*/Task& task)
  {
    switch (mExternalInteract->getCoordinates()) {
    case BodyFixedCoordinates:
      break;
    case GlobalCoordinates:
    default:
      mCoordinateSystem.setPosition(mExternalInteract->getPosition());
      mCoordinateSystem.setOrientation(mExternalInteract->getOrientation());
      break;
    }
  }

  virtual void velocities(const Task& task)
  {
    switch (mExternalInteract->getCoordinates()) {
    case BodyFixedCoordinates:
      // FIXME Is this the expected semantic??

      mCoordinateSystem = getLink().getCoordinateSystem().
        getRelative(mExternalInteract->getOrientation());
      
      // The global coordinates position
      if (mPosition.isConnected())
        mPosition = mCoordinateSystem.getPosition();
      
      if (mOrientation.isConnected())
        mOrientation = mCoordinateSystem.getOrientation();
      
      if (mEulerAngles.isConnected())
        mEulerAngles = mCoordinateSystem.getOrientation().getEuler();

      break;
    case GlobalCoordinates:
    default:
      // FIXME Is this the expected semantic??
      // FIXME bad optimized.
      
      // The global coordinates position
      if (mPosition.isConnected())
        mPosition = getLink().getPosition(mCoordinateSystem);
      
      if (mOrientation.isConnected())
        mOrientation = getLink().getOrientation(mCoordinateSystem);
      
      if (mEulerAngles.isConnected())
        mEulerAngles = getLink().getOrientation(mCoordinateSystem).getEuler();

      break;
    }

    // Velocity related sensing
    bool enableLinearVelocity = mLinearVelocity.isConnected();
    bool enableAngularVelocity = mAngularVelocity.isConnected();
    bool enableLinearWindVelocity = mLinearWindVelocity.isConnected();
    bool enableAngularWindVelocity = mAngularWindVelocity.isConnected();
    bool enableGroundSpeed = mGroundSpeed.isConnected();
    bool enableClimbSpeed = mClimbSpeed.isConnected();
    if (enableLinearVelocity || enableAngularVelocity
        || enableLinearWindVelocity || enableAngularWindVelocity
        || enableGroundSpeed || enableClimbSpeed) {
      Vector6 refVelocity = getLink().getVelocity(mCoordinateSystem);
      if (enableLinearVelocity)
        mLinearVelocity = refVelocity.getLinear();
      if (enableAngularVelocity)
        mAngularVelocity = refVelocity.getAngular();
     
      
      // Wind sensing
      if (enableLinearWindVelocity || enableAngularWindVelocity) {
        Vector6 wind = getEnvironment().getWindVelocity(task.getTime(),
                                                        mCoordinateSystem.getPosition());
        wind = refVelocity - wind;
        if (enableLinearWindVelocity)
          mLinearWindVelocity = wind.getLinear();
        if (enableAngularWindVelocity)
          mAngularWindVelocity = wind.getAngular();
      }

      if (enableGroundSpeed || enableClimbSpeed) {
        Vector3 linVel = mCoordinateSystem.rotToReference(refVelocity.getLinear());
        Plane plane = getEnvironment().getHorizontalLocalPlane(mCoordinateSystem.getPosition());
        if (enableGroundSpeed)
          mGroundSpeed = norm(plane.projectToPlane(linVel));
        if (enableClimbSpeed)
          mClimbSpeed = -dot(plane.getNormal(), linVel);
      }
    }

    // Atmosphere related sensing
    bool enableAltitude = mAltitude.isConnected();
    
    bool enableTemperature = mTemperature.isConnected();
    bool enableStaticPressure = mStaticPressure.isConnected();
    bool enableDensity = mDensity.isConnected();
    bool enableSoundSpeed = mSoundSpeed.isConnected();
    bool enableSpecificHeatRatio = mSpecificHeatRatio.isConnected();
    bool enableAtmosphere = (enableTemperature || enableStaticPressure ||
                             enableDensity || enableSoundSpeed ||
                             enableSpecificHeatRatio);
    if (enableAltitude || enableAtmosphere) {
      real_type altitude = getEnvironment().getAltitude(mCoordinateSystem.getPosition());
      if (enableAltitude)
        mAltitude = altitude;
      
      if (enableAtmosphere) {
        const AbstractAtmosphere* atmosphere = getEnvironment().getAtmosphere();
        AtmosphereData data = atmosphere->getData(task.getTime(), altitude);
        if (enableTemperature)
          mTemperature = data.temperature;
        if (enableStaticPressure)
          mStaticPressure = data.pressure;
        if (enableDensity)
          mDensity = data.density;
        if (enableSoundSpeed)
          mSoundSpeed = atmosphere->getSoundSpeed(data.temperature);
        if (enableSpecificHeatRatio)
          mSpecificHeatRatio = atmosphere->getSpecificHeatRatio(data.temperature);
      }
    }
    
    if (mAboveGroundLevel.isConnected()) {
      real_type agl;
      agl = getEnvironment().getAboveGroundLevel(task.getTime(),
                                                 mCoordinateSystem.getPosition());
      mAboveGroundLevel = agl;
    }
  }
  virtual void articulation(const Task& task)
  {
    // Apply all the forces ...
    if (mForce.isConnected())
      getLink().applyForce(mCoordinateSystem, Vector3(mForce.getValue()));
    if (mTorque.isConnected())
      getLink().applyTorque(mCoordinateSystem, mTorque.getValue());
  }
  virtual void accelerations(const Task& task)
  {
    bool enableCentrifugalAcceleration
      = mCentrifugalAcceleration.isConnected();
    bool enableLoad = mLoad.isConnected();
    bool enableAngularAcceleration
      = mAngularAcceleration.isConnected();
    if (enableCentrifugalAcceleration || enableLoad ||
        enableAngularAcceleration) {
      Vector6 spatialVel = getLink().getInertialVelocity(mCoordinateSystem);
      Vector6 spatialAccel = getLink().getInertialAcceleration(mCoordinateSystem);
      Vector3 centrifugalAccel = spatialAccel.getLinear();
      centrifugalAccel += cross(spatialVel.getAngular(),spatialVel.getLinear());

      if (enableCentrifugalAcceleration)
        mCentrifugalAcceleration = centrifugalAccel;
      if (enableLoad) {
        // May be cache that from the velocity step??
        Vector3 gravity = getEnvironment().getGravityAcceleration(mCoordinateSystem.getPosition());
        mLoad = Vector3(centrifugalAccel - gravity);
      }
      if (enableAngularAcceleration)
        mAngularAcceleration = spatialAccel.getAngular();
    }
  }
  
private:
  SharedPtr<const ExternalInteract> mExternalInteract;

  CoordinateSystem mCoordinateSystem;

  MatrixOutputPortHandle mPosition;
  MatrixOutputPortHandle mOrientation;
  MatrixOutputPortHandle mEulerAngles;

  MatrixOutputPortHandle mLinearVelocity;
  MatrixOutputPortHandle mAngularVelocity;

  MatrixOutputPortHandle mCentrifugalAcceleration;
  MatrixOutputPortHandle mLoad;
  MatrixOutputPortHandle mAngularAcceleration;

  MatrixOutputPortHandle mLinearWindVelocity;
  MatrixOutputPortHandle mAngularWindVelocity;

  RealOutputPortHandle mGroundSpeed;
  RealOutputPortHandle mClimbSpeed;

  RealOutputPortHandle mTemperature;
  RealOutputPortHandle mStaticPressure;
  RealOutputPortHandle mDensity;
  RealOutputPortHandle mSoundSpeed;
  RealOutputPortHandle mSpecificHeatRatio;

  RealOutputPortHandle mAltitude;
  RealOutputPortHandle mAboveGroundLevel;

  MatrixInputPortHandle mForce;
  MatrixInputPortHandle mTorque;
};

ExternalInteract::ExternalInteract(const std::string& name) :
  SingleLinkInteract(name),
  mCoordinates(BodyFixedCoordinates),
  mOrientation(Quaternion::unit())
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
ExternalInteract::setCoordinates(Coordinates coordinates)
{
  mCoordinates = coordinates;
}

void
ExternalInteract::setOrientation(const Quaternion& orientation)
{
  mOrientation = orientation;
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
ExternalInteract::setEnableAngularAcceleration(bool enable)
{
  if (enable == getEnableAngularAcceleration())
    return;
  if (enable)
    mAngularAccelerationPort
      = MatrixOutputPort(this, "angularAcceleration", Size(3, 1), true);
  else
    mAngularAccelerationPort.clear();
}

bool
ExternalInteract::getEnableAngularAcceleration() const
{
  return !mAngularAccelerationPort.empty();
}

void
ExternalInteract::setEnableLinearWindVelocity(bool enable)
{
  if (enable == getEnableLinearWindVelocity())
    return;
  if (enable)
    mLinearWindVelocityPort
      = MatrixOutputPort(this, "linearWindVelocity", Size(3, 1));
  else
    mLinearWindVelocityPort.clear();
}

bool
ExternalInteract::getEnableLinearWindVelocity() const
{
  return !mLinearWindVelocityPort.empty();
}

void
ExternalInteract::setEnableAngularWindVelocity(bool enable)
{
  if (enable == getEnableAngularWindVelocity())
    return;
  if (enable)
    mAngularWindVelocityPort
      = MatrixOutputPort(this, "angularWindVelocity", Size(3, 1));
  else
    mAngularWindVelocityPort.clear();
}

bool
ExternalInteract::getEnableAngularWindVelocity() const
{
  return !mAngularWindVelocityPort.empty();
}

void
ExternalInteract::setEnableGroundSpeed(bool enable)
{
  if (enable == getEnableGroundSpeed())
    return;
  if (enable)
    mGroundSpeedPort = RealOutputPort(this, "groundSpeed");
  else
    mGroundSpeedPort.clear();
}

bool
ExternalInteract::getEnableGroundSpeed() const
{
  return !mGroundSpeedPort.empty();
}

void
ExternalInteract::setEnableClimbSpeed(bool enable)
{
  if (enable == getEnableClimbSpeed())
    return;
  if (enable)
    mClimbSpeedPort = RealOutputPort(this, "climbSpeed");
  else
    mClimbSpeedPort.clear();
}

bool
ExternalInteract::getEnableClimbSpeed() const
{
  return !mClimbSpeedPort.empty();
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
ExternalInteract::setEnableStaticPressure(bool enable)
{
  if (enable == getEnableStaticPressure())
    return;
  if (enable)
    mStaticPressurePort = RealOutputPort(this, "staticPressure");
  else
    mStaticPressurePort.clear();
}

bool
ExternalInteract::getEnableStaticPressure() const
{
  return !mStaticPressurePort.empty();
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
ExternalInteract::setEnableSpecificHeatRatio(bool enable)
{
  if (enable == getEnableSpecificHeatRatio())
    return;
  if (enable)
    mSpecificHeatRatioPort = RealOutputPort(this, "specificHeatRatio");
  else
    mSpecificHeatRatioPort.clear();
}

bool
ExternalInteract::getEnableSpecificHeatRatio() const
{
  return !mSpecificHeatRatioPort.empty();
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
ExternalInteract::setEnableForce(bool enable)
{
  if (enable == getEnableForce())
    return;
  if (enable)
    mForcePort = MatrixInputPort(this, "force", Size(3, 1), true);
  else
    mForcePort.clear();
}

bool
ExternalInteract::getEnableForce() const
{
  return !mForcePort.empty();
}

void
ExternalInteract::setEnableTorque(bool enable)
{
  if (enable == getEnableTorque())
    return;
  if (enable)
    mTorquePort = MatrixInputPort(this, "torque", Size(3, 1), true);
  else
    mTorquePort.clear();
}

bool
ExternalInteract::getEnableTorque() const
{
  return !mTorquePort.empty();
}

void
ExternalInteract::setEnableAllOutputs(bool enable)
{
  setEnablePosition(enable);
  setEnableOrientation(enable);
  setEnableEulerAngles(enable);
  setEnableLinearVelocity(enable);
  setEnableAngularVelocity(enable);
  setEnableLinearWindVelocity(enable);
  setEnableAngularWindVelocity(enable);
  setEnableCentrifugalAcceleration(enable);
  setEnableLoad(enable);
  setEnableAngularAcceleration(enable);
  setEnableGroundSpeed(enable);
  setEnableClimbSpeed(enable);
  setEnableTemperature(enable);
  setEnableStaticPressure(enable);
  setEnableDensity(enable);
  setEnableSoundSpeed(enable);
  setEnableSpecificHeatRatio(enable);
  setEnableAltitude(enable);
  setEnableAboveGroundLevel(enable);
}

} // namespace OpenFDM
