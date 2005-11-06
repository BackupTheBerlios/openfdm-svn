/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WheelContact_H
#define OpenFDM_WheelContact_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Ground.h"
#include "Environment.h"

namespace OpenFDM {

class WheelContact
  : public ExternalForce {
public:
  WheelContact(const std::string& name, Environment* env);
  virtual ~WheelContact(void);

  virtual void output(const TaskInfo&);

  virtual void computeForce(void);

  // Compute the plane normal force.
  virtual real_type
  computeNormalForce(real_type compressLen, real_type compressVel) const;

  // Compute the friction force.
  virtual Vector2
  computeFrictionForce(real_type normForce, const Vector2& vel,
                       real_type omegaR, real_type friction) const;

  void setWheelRadius(real_type wheelRadius)
  { mWheelRadius = wheelRadius; }
  real_type getWheelRadius(void) const
  { return mWheelRadius; }

  void setSpringConstant(real_type springConstant)
  { mSpringConstant = springConstant; }
  real_type getSpringConstant(void) const
  { return mSpringConstant; }

  void setSpringDamping(real_type springDamping)
  { mSpringDamping = springDamping; }
  real_type getSpringDamping(void) const
  { return mSpringDamping; }

  void setFrictionCoeficient(real_type frictionCoeficient)
  { mFrictionCoeficient = frictionCoeficient; }
  real_type getFrictionCoeficient(void) const
  { return mFrictionCoeficient; }
private:
  void getGround(real_type t);

  GroundValues mGroundVal;
  shared_ptr<Environment> mEnvironment;

  real_type mWheelRadius;
  real_type mSpringConstant;
  real_type mSpringDamping;
  real_type mFrictionCoeficient;
};

} // namespace OpenFDM

#endif
