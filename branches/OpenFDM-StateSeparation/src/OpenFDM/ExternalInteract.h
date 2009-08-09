/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ExternalInteract_H
#define OpenFDM_ExternalInteract_H

#include "SingleLinkInteract.h"
#include "MatrixInputPort.h"
#include "MatrixOutputPort.h"
#include "RealOutputPort.h"
#include "Vector.h"

namespace OpenFDM {

class ExternalInteract : public SingleLinkInteract {
  OPENFDM_OBJECT(ExternalInteract, SingleLinkInteract);
public:
  ExternalInteract(const std::string& name);
  virtual ~ExternalInteract(void);

  virtual MechanicContext*
  newMechanicContext(const Environment*, PortValueList&) const;

  enum Coordinates {
    BodyFixedCoordinates,
    GlobalCoordinates
  };

  /// The Coordinates
  Coordinates getCoordinates() const
  { return mCoordinates; }
  void setCoordinates(Coordinates coordinates);

  /// The orientation of the coordinate system this Interact measures its values
  const Quaternion& getOrientation() const
  { return mOrientation; }
  void setOrientation(const Quaternion& orientation);

  /// Set availabilty of the position output port
  void setEnablePosition(bool enable);
  /// Get availabilty of the position output port
  bool getEnablePosition() const;

  /// Set availabilty of the orientation output port
  void setEnableOrientation(bool enable);
  /// Get availabilty of the orientation output port
  bool getEnableOrientation() const;

  /// Set availabilty of the euler angles output port
  void setEnableEulerAngles(bool enable);
  /// Get availabilty of the euler angles output port
  bool getEnableEulerAngles() const;


  /// Set availabilty of the linear velocity output port
  void setEnableLinearVelocity(bool enable);
  /// Get availabilty of the linear velocity output port
  bool getEnableLinearVelocity() const;

  /// Set availabilty of the angular velocity output port
  void setEnableAngularVelocity(bool enable);
  /// Get availabilty of the angular velocity output port
  bool getEnableAngularVelocity() const;


  /// Set availabilty of the angular acceleration output port
  void setEnableAngularAcceleration(bool enable);
  /// Get availabilty of the angular acceleration output port
  bool getEnableAngularAcceleration() const;

  /// Set availabilty of the linear acceleration output port
  void setEnableCentrifugalAcceleration(bool enable);
  /// Get availabilty of the linear acceleration output port
  bool getEnableCentrifugalAcceleration() const;

  /// Set availabilty of the load output port
  void setEnableLoad(bool enable);
  /// Get availabilty of the load output port
  bool getEnableLoad() const;


  /// Set availabilty of the wind velocity output port
  void setEnableLinearWindVelocity(bool enable);
  /// Get availabilty of the wind velocity output port
  bool getEnableLinearWindVelocity() const;

  /// Set availabilty of the wind velocity output port
  /// The output vector is measured in global coordinates
  void setEnableAngularWindVelocity(bool enable);
  /// Get availabilty of the wind velocity output port
  bool getEnableAngularWindVelocity() const;


  /// Set availabilty of the ground speed output port
  void setEnableGroundSpeed(bool enable);
  /// Get availabilty of the ground speed output port
  bool getEnableGroundSpeed() const;


  /// Set availabilty of the temperature output port
  void setEnableTemperature(bool enable);
  /// Get availabilty of the temperature output port
  bool getEnableTemperature() const;

  /// Set availabilty of the static pressure output port
  void setEnableStaticPressure(bool enable);
  /// Get availabilty of the static pressure output port
  bool getEnableStaticPressure() const;

  /// Set availabilty of the density output port
  void setEnableDensity(bool enable);
  /// Get availabilty of the density output port
  bool getEnableDensity() const;

  /// Set availabilty of the sound speed output port
  void setEnableSoundSpeed(bool enable);
  /// Get availabilty of the sound speed output port
  bool getEnableSoundSpeed() const;

  /// Set availabilty of the specific heat ratio output port
  void setEnableSpecificHeatRatio(bool enable);
  /// Get availabilty of the specific heat ratio output port
  bool getEnableSpecificHeatRatio() const;


  /// Set availabilty of the altitude output port
  void setEnableAltitude(bool enable);
  /// Get availabilty of the altitude output port
  bool getEnableAltitude() const;

  /// Set availabilty of the above ground level output port
  void setEnableAboveGroundLevel(bool enable);
  /// Get availabilty of the above ground level output port
  bool getEnableAboveGroundLevel() const;


  /// Set availabilty of the body force input port
  void setEnableForce(bool enable);
  /// Get availabilty of the body force input port
  bool getEnableForce() const;

  /// Set availabilty of the body torque input port
  void setEnableTorque(bool enable);
  /// Get availabilty of the body torque input port
  bool getEnableTorque() const;


  /// Set availabilty of all output ports
  /// This is mostly for convinience in testing
  void setEnableAllOutputs(bool enable);

protected:
  class Context;

  Coordinates mCoordinates;

  Quaternion mOrientation;

  /// Positional state sensing
  MatrixOutputPort mPositionPort;
  MatrixOutputPort mOrientationPort;
  MatrixOutputPort mEulerAnglesPort;

  /// Velocity state sensing
  MatrixOutputPort mLinearVelocityPort;
  MatrixOutputPort mAngularVelocityPort;

  /// Acceleration sensing
  MatrixOutputPort mAngularAccelerationPort;
  MatrixOutputPort mCentrifugalAccelerationPort;
  MatrixOutputPort mLoadPort;

  /// Wind sensing
  MatrixOutputPort mLinearWindVelocityPort;
  MatrixOutputPort mAngularWindVelocityPort;

  RealOutputPort mGroundSpeedPort;

  RealOutputPort mTemperaturePort;
  RealOutputPort mStaticPressurePort;
  RealOutputPort mDensityPort;
  RealOutputPort mSoundSpeedPort;
  RealOutputPort mSpecificHeatRatioPort;

  RealOutputPort mAltitudePort;
  RealOutputPort mAboveGroundLevelPort;

  /// Force inputs
  MatrixInputPort mForcePort;
  MatrixInputPort mTorquePort;
};

} // namespace OpenFDM

#endif
