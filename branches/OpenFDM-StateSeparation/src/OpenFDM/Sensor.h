/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Sensor_H
#define OpenFDM_Sensor_H

#include "Interact.h"
#include "MatrixOutputPort.h"
#include "MechanicLink.h"
#include "RealOutputPort.h"
#include "Vector.h"

namespace OpenFDM {

class Sensor : public Interact {
  OPENFDM_OBJECT(Sensor, Interact);
public:
  Sensor(const std::string& name);
  virtual ~Sensor(void);

  virtual void initDesignPosition(PortValueList&) const;
  virtual void velocity(const Task&, const ContinousStateValueVector&,
                        PortValueList&) const;
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList&) const;

  /// Set the position of the sensor in design coordinates
  void setPosition(const Vector3& position);
  /// Get the position of the sensor in design coordinates
  const Vector3& getPosition() const;

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

  /// Set availabilty of the linear acceleration output port
  void setEnableCentrifugalAcceleration(bool enable);
  /// Get availabilty of the linear acceleration output port
  bool getEnableCentrifugalAcceleration() const;

  /// Set availabilty of the load output port
  void setEnableLoad(bool enable);
  /// Get availabilty of the load output port
  bool getEnableLoad() const;


  /// Set availabilty of the wind velocity output port
  void setEnableWindVelocity(bool enable);
  /// Get availabilty of the wind velocity output port
  bool getEnableWindVelocity() const;


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


  /// Set availabilty of all output ports
  /// This is mostly for convinience in testing
  void setEnableAll(bool enable);

protected:
  MechanicLink mMechanicLink;

  Vector3 mPosition;

  MatrixOutputPort mPositionPort;
  MatrixOutputPort mOrientationPort;
  MatrixOutputPort mEulerAnglesPort;

  MatrixOutputPort mLinearVelocityPort;
  MatrixOutputPort mAngularVelocityPort;

  MatrixOutputPort mCentrifugalAccelerationPort;
  MatrixOutputPort mLoadPort;

  MatrixOutputPort mWindVelocityPort;

  RealOutputPort mTemperaturePort;
  RealOutputPort mPressurePort;
  RealOutputPort mDensityPort;
  RealOutputPort mSoundSpeedPort;

  RealOutputPort mAltitudePort;
  RealOutputPort mAboveGroundLevelPort;
};

} // namespace OpenFDM

#endif
