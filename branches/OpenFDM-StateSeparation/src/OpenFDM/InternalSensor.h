/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_InternalSensor_H
#define OpenFDM_InternalSensor_H

#include "DoubleLinkInteract.h"
#include "MechanicLink.h"
#include "RealInputPort.h"
#include "RealOutputPort.h"
#include "Vector.h"

namespace OpenFDM {

class InternalSensor : public DoubleLinkInteract {
  OPENFDM_OBJECT(InternalSensor, DoubleLinkInteract);
public:
  InternalSensor(const std::string& name);
  virtual ~InternalSensor(void);

  virtual void initDesignPosition(PortValueList&) const {}
  virtual void velocity(const Task&, const Environment& environment,
                        const ContinousStateValueVector&,
                        PortValueList&) const;
  virtual void articulation(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
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

  /// Set availabilty of the force output port
  void setEnableForce(bool enable);
  /// Get availabilty of the force output port
  bool getEnableForce() const;

  /// Set availabilty of all output ports
  /// This is mostly for convinience in testing
  void setEnableAllOutputs(bool enable);

protected:
  Vector3 mPosition0;
  Vector3 mPosition1;

  RealOutputPort mDistancePort;
  RealOutputPort mVelocityPort;
  RealInputPort mForcePort;
};

} // namespace OpenFDM

#endif
