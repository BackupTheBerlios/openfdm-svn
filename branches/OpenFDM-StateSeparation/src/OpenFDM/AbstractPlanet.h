/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractPlanet_H
#define OpenFDM_AbstractPlanet_H

#include "Types.h"
#include "Vector.h"
#include "Plane.h"
#include "Referenced.h"

namespace OpenFDM {

/**
 * The Planet class.
 *
 * It holds some information about the planet the simulation is running on.
 */
class AbstractPlanet : public Referenced {
public:
  /** Default constructor.
   */
  AbstractPlanet(void);

  /** Default destructor.
   */
  virtual ~AbstractPlanet(void);

  /** Returns the horizontal plane at zero altitude.
   *  Plane normal points downward.
   */
  virtual Plane getHorizont(const Vector3& position) const = 0;

  /** Returns the gravitational acceleration for the given position.
   *  Note that this should not contain the effects of a non inertial
   *  reference frame as this effect is captured by the inertial
   *  frame methods.
   */
  virtual Vector3 getGravityAcceleration(const Vector3&) const = 0;

  /** Return the global reference frames velocity and acceleration.
   *  Note that these both must fit together to make the simulation
   *  simulate something usable.
   *  FIXME: ReferenceFrame class???
   */
  virtual Vector3 getAngularVelocity(const real_type& t) const = 0;
  virtual Vector6 getAcceleration(const real_type& t) const = 0;
};

} // namespace OpenFDM

#endif
