/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SimpleGear_H
#define OpenFDM_SimpleGear_H

#include "Assert.h"
#include "Unit.h"
#include "Object.h"
#include "Vector.h"
#include "Contact.h"

namespace OpenFDM {

class SimpleGear : public Contact {
  OPENFDM_OBJECT(SimpleGear, Contact);
public:
  SimpleGear(const std::string& name);
  virtual ~SimpleGear(void);

  /// Set availabilty of the steering angle input port
  void setEnableSteeringAngle(bool enable);
  /// Get availabilty of the steering angle input port
  bool getEnableSteeringAngle() const;

  /// Set availabilty of the brake command input port
  void setEnableBrakeCommand(bool enable);
  /// Get availabilty of the brake command input port
  bool getEnableBrakeCommand() const;

  real_type getSpringConstant(void) const;
  void setSpringConstant(real_type springConst);

  real_type getDamperConstant(void) const;
  void setDamperConstant(real_type damperConstant);

  real_type getFrictionCoeficient(void) const;
  void setFrictionCoeficient(real_type frictionCoef);

  // Compute the plane normal force.
  virtual real_type
  computeNormalForce(real_type compressLen, real_type compressVel,
                     PortValueList&) const;

  // Compute the friction force.
  virtual Vector3
  computeFrictionForce(real_type normForce, const Vector3& vel,
                       const Vector3& groundNormal, real_type friction,
                       PortValueList&) const;

private:
  RealInputPort mSteeringAnglePort;
  RealInputPort mBrakeCommandPort;

  real_type mSpringConst;
  real_type mDamperConst;
  real_type mFrictionCoef;
};

} // namespace OpenFDM

#endif
