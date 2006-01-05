/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DefaultGround_H
#define OpenFDM_DefaultGround_H

#include "Types.h"
#include "Object.h"
#include "Plane.h"
#include "Ground.h"

namespace OpenFDM {

class Environment;

/**
 * The DefaultGround class.
 */
class DefaultGround
  : public Ground {
public:
  /** Default constructor.
   */
  DefaultGround();

  /** Default destructor.
   */
  virtual ~DefaultGround(void);

  virtual GroundValues
  getGroundPlane(real_type t, const Vector3& refPos) const;
};

} // namespace OpenFDM

#endif
