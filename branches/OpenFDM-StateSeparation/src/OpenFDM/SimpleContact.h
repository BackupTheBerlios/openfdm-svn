/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SimpleContact_H
#define OpenFDM_SimpleContact_H

#include "Contact.h"

namespace OpenFDM {

class SimpleContact : public Contact {
  OPENFDM_OBJECT(SimpleContact, Contact);
public:
  SimpleContact(const std::string& name);
  virtual ~SimpleContact(void);

  const real_type& getSpringConstant(void) const;
  void setSpringConstant(const real_type& springConst);

  const real_type& getDamperConstant(void) const;
  void setDamperConstant(const real_type& damperConstant);

  const real_type& getFrictionCoeficient(void) const;
  void setFrictionCoeficient(const real_type& frictionCoef);

  // Compute the plane normal force.
  virtual real_type
  computeNormalForce(real_type compressLen, real_type compressVel,
                     PortValueList&) const;

  // Compute the friction force.
  virtual Vector3
  computeFrictionForce(real_type normForce, const Vector3& vel,
                       const Vector3&, real_type friction,
                       PortValueList&) const;

private:
  real_type mSpringConst;
  real_type mDamperConstant;
  real_type mFrictionCoef;
};

} // namespace OpenFDM

#endif
