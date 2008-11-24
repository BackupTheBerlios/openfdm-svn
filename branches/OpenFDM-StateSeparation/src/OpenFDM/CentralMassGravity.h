/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_CentralMassGravity_H
#define OpenFDM_CentralMassGravity_H

#include "Types.h"
#include "Vector.h"
#include "AbstractGravity.h"

namespace OpenFDM {

/**
 * The CentralMassGravity class.
 *
 * It holds some information about the gravity the simulation is running on.
 */
class CentralMassGravity : public AbstractGravity {
public:
  /** CentralMass constructor.
   */
  CentralMassGravity(void);

  /** CentralMass destructor.
   */
  virtual ~CentralMassGravity(void);

  /** Get planet mass.
   */
  real_type getPlanetMass(void) const;

  /** Set planet mass.
   */
  void setPlanetMass(real_type mass);

  /** Gravity acceleration at the cartesion position cart.
   */
  virtual Vector3 getGravityAcceleration(const Environment&,
                                         const Vector3&) const;

private:
  real_type mMass;
};

} // namespace OpenFDM

#endif
