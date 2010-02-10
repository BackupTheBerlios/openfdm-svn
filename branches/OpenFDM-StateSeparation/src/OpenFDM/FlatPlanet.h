/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_FlatPlanet_H
#define OpenFDM_FlatPlanet_H

#include "Types.h"
#include "Vector.h"
#include "AbstractPlanet.h"

namespace OpenFDM {

class Environment;

/**
 * The FlatPlanet class.
 *
 * Used for simulation where we do not want to move on the whole worlds surface.
 */
class FlatPlanet : public AbstractPlanet {
public:
  /** Flat constructor.
   */
  FlatPlanet(void);

  /** Flat destructor.
   */
  virtual ~FlatPlanet(void);

  /** Returns the horizontal plane at zero altitude.
   *  Plane normal points downward.
   */
  virtual Plane getHorizont(const Vector3& position) const;

  /** Returns the gravitational acceleration for the given position.
   *  Note that this should not contain the effects of a non inertial
   *  reference frame as this effect is captured by the inertial
   *  frame methods.
   */
  virtual Vector3 getGravityAcceleration(const Vector3&) const;

  /** Return the global reference frames velocity and acceleration.
   *  Note that these both must fit together to make the simulation
   *  simulate something usable.
   */
  virtual Vector3 getAngularVelocity(const real_type& t) const;
  virtual Vector6 getAcceleration(const real_type& t) const;
};

} // namespace OpenFDM

#endif
