/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DefaultGravity_H
#define OpenFDM_DefaultGravity_H

#include "Types.h"
#include "Vector.h"
#include "Gravity.h"

namespace OpenFDM {

/**
 * The DefaultGravity class.
 *
 * It holds some information about the gravity the simulation is running on.
 */
class DefaultGravity
  : public Gravity {
public:
  /** Default constructor.
   */
  DefaultGravity(void);

  /** Default destructor.
   */
  virtual ~DefaultGravity(void);

  /** Get planet mass.
   */
  real_type getPlanetMass(void) const;

  /** Set planet mass.
   */
  void setPlanetMass(real_type mass);

  /** Gravity acceleration at the cartesion position cart.
   */
  virtual Vector3 gravityAccel(const Vector3& cart) const;

private:
  real_type mMass;
};

} // namespace OpenFDM

#endif
