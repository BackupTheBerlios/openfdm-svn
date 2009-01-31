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
  class Context;
public:
  ExternalInteract(const std::string& name);
  virtual ~ExternalInteract(void);

  virtual MechanicContext*
  newMechanicContext(const Environment*, PortValueList&) const;

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
  /// The output vector is measured in body coordinates
  void setEnableBodyLinearVelocity(bool enable);
  /// Get availabilty of the linear velocity output port
  bool getEnableBodyLinearVelocity() const;

  /// Set availabilty of the angular velocity output port
  /// The output vector is measured in body coordinates
  void setEnableBodyAngularVelocity(bool enable);
  /// Get availabilty of the angular velocity output port
  bool getEnableBodyAngularVelocity() const;

  /// Set availabilty of the linear velocity output port
  /// The output vector is measured in global coordinates
  void setEnableGlobalLinearVelocity(bool enable);
  /// Get availabilty of the linear velocity output port
  bool getEnableGlobalLinearVelocity() const;

  /// Set availabilty of the angular velocity output port
  /// The output vector is measured in global coordinates
  void setEnableGlobalAngularVelocity(bool enable);
  /// Get availabilty of the angular velocity output port
  bool getEnableGlobalAngularVelocity() const;


  /// Set availabilty of the linear acceleration output port
  /// The output vector is measured in body coordinates
  void setEnableBodyCentrifugalAcceleration(bool enable);
  /// Get availabilty of the linear acceleration output port
  bool getEnableBodyCentrifugalAcceleration() const;

  /// Set availabilty of the load output port
  /// The output vector is measured in body coordinates
  void setEnableBodyLoad(bool enable);
  /// Get availabilty of the load output port
  bool getEnableBodyLoad() const;


  /// Set availabilty of the wind velocity output port
  /// The output vector is measured in body coordinates
  void setEnableBodyWindVelocity(bool enable);
  /// Get availabilty of the wind velocity output port
  bool getEnableBodyWindVelocity() const;

  /// Set availabilty of the wind velocity output port
  /// The output vector is measured in global coordinates
  void setEnableGlobalWindVelocity(bool enable);
  /// Get availabilty of the wind velocity output port
  bool getEnableGlobalWindVelocity() const;


  /// Set availabilty of the temperature output port
  void setEnableTemperature(bool enable);
  /// Get availabilty of the temperature output port
  bool getEnableTemperature() const;

  /// Set availabilty of the pressure output port
  void setEnablePressure(bool enable);
  /// Get availabilty of the pressure output port
  bool getEnablePressure() const;

  /// Set availabilty of the density output port
  void setEnableDensity(bool enable);
  /// Get availabilty of the density output port
  bool getEnableDensity() const;

  /// Set availabilty of the sound speed output port
  void setEnableSoundSpeed(bool enable);
  /// Get availabilty of the sound speed output port
  bool getEnableSoundSpeed() const;


  /// Set availabilty of the altitude output port
  void setEnableAltitude(bool enable);
  /// Get availabilty of the altitude output port
  bool getEnableAltitude() const;

  /// Set availabilty of the above ground level output port
  void setEnableAboveGroundLevel(bool enable);
  /// Get availabilty of the above ground level output port
  bool getEnableAboveGroundLevel() const;


  /// Set availabilty of the body force input port
  void setEnableBodyForce(bool enable);
  /// Get availabilty of the body force input port
  bool getEnableBodyForce() const;

  /// Set availabilty of the body torque input port
  void setEnableBodyTorque(bool enable);
  /// Get availabilty of the body torque input port
  bool getEnableBodyTorque() const;


  /// Set availabilty of the global force input port
  void setEnableGlobalForce(bool enable);
  /// Get availabilty of the global force input port
  bool getEnableGlobalForce() const;

  /// Set availabilty of the global torque input port
  void setEnableGlobalTorque(bool enable);
  /// Get availabilty of the global torque input port
  bool getEnableGlobalTorque() const;


  /// Set availabilty of all output ports
  /// This is mostly for convinience in testing
  void setEnableAllOutputs(bool enable);

protected:
  /// Positional state sensing
  MatrixOutputPort mPositionPort;
  MatrixOutputPort mOrientationPort;
  MatrixOutputPort mEulerAnglesPort;

  /// Velocity state sensing
  MatrixOutputPort mBodyLinearVelocityPort;
  MatrixOutputPort mBodyAngularVelocityPort;
  MatrixOutputPort mGlobalLinearVelocityPort;
  MatrixOutputPort mGlobalAngularVelocityPort;

  MatrixOutputPort mBodyCentrifugalAccelerationPort;
  MatrixOutputPort mBodyLoadPort;

  MatrixOutputPort mBodyWindVelocityPort;
  MatrixOutputPort mGlobalWindVelocityPort;

  RealOutputPort mTemperaturePort;
  RealOutputPort mPressurePort;
  RealOutputPort mDensityPort;
  RealOutputPort mSoundSpeedPort;

  RealOutputPort mAltitudePort;
  RealOutputPort mAboveGroundLevelPort;

  MatrixInputPort mBodyForcePort;
  MatrixInputPort mBodyTorquePort;
  MatrixInputPort mGlobalForcePort;
  MatrixInputPort mGlobalTorquePort;
};

} // namespace OpenFDM

#endif
