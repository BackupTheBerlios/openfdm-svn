/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SimpleGear_H
#define OpenFDM_SimpleGear_H

#include "Assert.h"
#include "Unit.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Contact.h"

namespace OpenFDM {

class SimpleGear : public Contact {
  OPENFDM_OBJECT(SimpleGear, Contact);
public:
  SimpleGear(const std::string& name);
  virtual ~SimpleGear(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  real_type getSteeringAngle(void) const;
  void setSteeringAngle(real_type steeringAngle);

  real_type getBrake(void) const;
  void setBrake(real_type brake);

  real_type getSpringConstant(void) const;
  void setSpringConstant(real_type springConst);

  real_type getSpringDamping(void) const;
  void setSpringDamping(real_type springDamp);

  real_type getFrictionCoeficient(void) const;
  void setFrictionCoeficient(real_type frictionCoef);

  // Compute the plane normal force.
  virtual real_type
  computeNormalForce(real_type compressLen, real_type compressVel) const;

  // Compute the friction force.
  virtual Vector3
  computeFrictionForce(real_type normForce, const Vector3& vel,
                       const Vector3& groundNormal, real_type friction) const;

private:
  real_type mSteeringAngle;
  real_type mBrake;

  real_type mSpringConst;
  real_type mSpringDamp;
  real_type mFrictionCoef;

  RealPortHandle mBrakeCommandHandle;
  RealPortHandle mSteeringAngleHandle;
};

} // namespace OpenFDM

#endif
