/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Ground_H
#define OpenFDM_Ground_H

#include "Types.h"
#include "EnvironmentObject.h"
#include "Plane.h"
#include "Quaternion.h"

namespace OpenFDM {

struct GroundValues {
  GroundValues(const Plane& p = Plane(), const Vector6& v = Vector6::zeros(),
               real_type fr = 1) :
    plane(p), vel(v), friction(fr) {}
  Plane plane;
  Vector6 vel;
  real_type friction;
};

struct CatapultValues {
  /// The position of the catapult start in reference coordinates
  Vector3 position;
  /// The orientation of the catapult frame
  Quaternion orientation;
  /// The spatial velocity of the catapult frame
  Vector6 velocity;
  /// The catapult length
  real_type length;
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

  /** Returns true if a catapult is within some reasonable range to
      the reference position. The reference position is usually the
      position of the launchbar.
   */
  virtual bool
  getCatapultValues(real_type t, const Vector3& refPos,
                    CatapultValues& catVal) const
  { return false; }
};

} // namespace OpenFDM

#endif
