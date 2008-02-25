/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

struct HookPosition {
  /// The time it is meant for
  real_type t;
  /// The hooks base position in reference frames coordinates
  Vector3 basePosition;
  /// The hooks direction and length in reference frames coordinates
  /// That is basePosition + hookVector = hookTip
  Vector3 hookVector;
};

struct WireValues {
  /// The position of the wire midpoint in reference coordinates
  Vector3 position;
  /// The orientation of the wire frame
  Quaternion orientation;
  /// The spatial velocity of the wire frame
  Vector6 velocity;
  /// The wire mounts width, they are assumed to be mounted along the y axis
  real_type width;
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
                    CatapultValues& catVal) const;

  /** Returns true if we caught a wire while traversing the given rectangle
      given in reference cordinates.
   */
  virtual bool
  caughtWire(const HookPosition& old, const HookPosition& current) const;

  /** Writes the motions values for the wires we have caught.
      Returns false if the wires are lost somehow.
   */
  virtual bool
  getWireEnds(real_type t, WireValues& wireVal) const;

  /** Called if the wire values are no longer used.
   */
  virtual void
  releaseWire(void) const;
};

} // namespace OpenFDM

#endif
