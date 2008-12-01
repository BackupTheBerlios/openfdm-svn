/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WheelContact_H
#define OpenFDM_WheelContact_H

#include "Interact.h"

namespace OpenFDM {

class WheelContact : public Interact {
  OPENFDM_OBJECT(WheelContact, Interact);
public:
  WheelContact(const std::string& name);
  virtual ~WheelContact(void);

  virtual void initDesignPosition(PortValueList&) const {}
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues) const;

  // Compute the plane normal force.
  virtual real_type
  computeNormalForce(real_type compressLen, real_type compressVel) const;

  // Compute the friction force.
  virtual Vector2
  computeFrictionForce(real_type normForce, const Vector2& vel,
                       real_type omegaR, real_type friction) const;

  /** Set a position offset for the inertia given.
   */
  const Vector3& getPosition(void) const;
  void setPosition(const Vector3& position);

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
  MechanicLink mMechanicLink;

  Vector3 mPosition;
  real_type mWheelRadius;
  real_type mSpringConstant;
  real_type mSpringDamping;
  real_type mFrictionCoeficient;
};

} // namespace OpenFDM

#endif
