/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_InternalSensor_H
#define OpenFDM_InternalSensor_H

#include "Interact.h"
#include "MechanicLink.h"
#include "RealOutputPort.h"
#include "Vector.h"

namespace OpenFDM {

class InternalSensor : public Interact {
  OPENFDM_OBJECT(InternalSensor, Interact);
public:
  InternalSensor(const std::string& name);
  virtual ~InternalSensor(void);

  virtual void initDesignPosition(PortValueList&) const {}
  virtual void velocity(const Task&, const ContinousStateValueVector&,
                        PortValueList&) const;

  /// Set the position of the sensor in design coordinates
  void setPosition0(const Vector3& position);
  /// Get the position of the sensor in design coordinates
  const Vector3& getPosition0() const;

  /// Set the position of the sensor in design coordinates
  void setPosition1(const Vector3& position);
  /// Get the position of the sensor in design coordinates
  const Vector3& getPosition1() const;

  /// Set availabilty of the distance output port
  void setEnableDistance(bool enable);
  /// Get availabilty of the distance output port
  bool getEnableDistance() const;

  /// Set availabilty of the velocity output port
  void setEnableVelocity(bool enable);
  /// Get availabilty of the velocity output port
  bool getEnableVelocity() const;

  /// Set availabilty of all output ports
  /// This is mostly for convinience in testing
  void setEnableAll(bool enable);

protected:
  MechanicLink mMechanicLink0;
  MechanicLink mMechanicLink1;

  Vector3 mPosition0;
  Vector3 mPosition1;

  RealOutputPort mDistancePort;
  RealOutputPort mVelocityPort;
};

} // namespace OpenFDM

#endif
