/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Gravity_H
#define OpenFDM_Gravity_H

#include "Types.h"
#include "EnvironmentObject.h"
#include "Vector.h"

namespace OpenFDM {

/**
 * The Gravity class.
 */
class Gravity
  : public EnvironmentObject {
public:
  /** Default constructor.
   */
  Gravity(void);

  /** Default destructor.
   */
  virtual ~Gravity(void);

  /** Gravity acceleration at the cartesion position cart.
   */
  virtual Vector3 gravityAccel(const Vector3& cart) const = 0;
};

} // namespace OpenFDM

#endif
