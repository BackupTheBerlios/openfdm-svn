/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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

class WheelContact : public ExternalForce {
  OPENFDM_OBJECT(WheelContact, ExternalForce);
public:
  WheelContact(const std::string& name);
  virtual ~WheelContact(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  // Compute the plane normal force.
  virtual real_type
  computeNormalForce(real_type compressLen, real_type compressVel) const;

  // Compute the friction force.
  virtual Vector2
  computeFrictionForce(real_type normForce, const Vector2& vel,
                       real_type omegaR, real_type friction) const;

  void setWheelRadius(const real_type& wheelRadius)
  { mWheelRadius = wheelRadius; }
  const real_type& getWheelRadius(void) const
  { return mWheelRadius; }

  void setSpringConstant(const real_type& springConstant)
  { mSpringConstant = springConstant; }
  const real_type& getSpringConstant(void) const
  { return mSpringConstant; }

  void setSpringDamping(const real_type& springDamping)
  { mSpringDamping = springDamping; }
  const real_type& getSpringDamping(void) const
  { return mSpringDamping; }

  void setFrictionCoeficient(const real_type& frictionCoeficient)
  { mFrictionCoeficient = frictionCoeficient; }
  const real_type& getFrictionCoeficient(void) const
  { return mFrictionCoeficient; }
private:
  void getGround(real_type t);

  GroundValues mGroundVal;
  SharedPtr<Environment> mEnvironment;

  real_type mWheelRadius;
  real_type mSpringConstant;
  real_type mSpringDamping;
  real_type mFrictionCoeficient;
};

} // namespace OpenFDM

#endif
