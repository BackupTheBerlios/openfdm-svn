/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Ground_H
#define OpenFDM_Ground_H

#include "Types.h"
#include "EnvironmentObject.h"
#include "Plane.h"

namespace OpenFDM {

struct GroundValues {
  GroundValues(const Plane& p = Plane(), const Vector6& v = Vector6::zeros(),
               real_type fr = 1)
    : plane(p), vel(v), friction(fr)
  {}
  Plane plane;
  Vector6 vel;
  real_type friction;
};

/**
 * The Ground class.
 */
class Ground
  : public EnvironmentObject {
public:
  /** Default constructor.
   */
  Ground(void);

  /** Default destructor.
   */
  virtual ~Ground(void);

  virtual GroundValues
  getGroundPlane(real_type t, const Vector3& refPos) const = 0;
};

} // namespace OpenFDM

#endif
