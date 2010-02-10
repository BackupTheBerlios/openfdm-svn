/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WGS84Planet_H
#define OpenFDM_WGS84Planet_H

#include <iosfwd>

#include "Types.h"
#include "Vector.h"
#include "Quaternion.h"
#include "AbstractPlanet.h"

namespace OpenFDM {

/**
 * Datatype for the Geodetic position on some ellipsoid.
 */
struct Geodetic {
  Geodetic(real_type lat = 0.0, real_type lon = 0.0, real_type alt = 0.0)
    : latitude(lat), longitude(lon), altitude(alt)
  {}
  real_type latitude;
  real_type longitude;
  real_type altitude;
};

/**
 * The WGS84Planet class.
 *
 * It holds some information about the planet the simulation is running on.
 */
class WGS84Planet : public AbstractPlanet {
public:
  /** Elliptic constructor.
   */
  WGS84Planet(void);

  /** Elliptic destructor.
   */
  virtual ~WGS84Planet(void);

  /** Get aequatorial radius, also called semi major axis.
   */
  const real_type& getAequatorialRadius(void) const;
  /** Set aequatorial radius, also called semi major axis.
   */
  void setAequatorialRadius(const real_type& r);

  /** Get planet flattening.
   */
  const real_type& getFlattening() const;
  /** Set planet flattening.
   */
  void setFlattening(const real_type& flat);

  /** Get the gravittational constant multiplied by the planets mass.
   */
  const real_type& getGM() const;
  /** Set the gravittational constant multiplied by the planets mass.
   */
  void setGM(const real_type& gm);

  /** Get the rotational speed of the planet.
   */
  const real_type& getOmega() const;
  /** Set the rotational speed of the planet.
   */
  void setOmega(const real_type& omega);

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


  /** Transform cartesian coordinates to geodetic coordinates.
   */
  Geodetic toGeod(const Vector3& cart) const;

  /** Transform geodetic coordinates to cartesian coordinates.
   */
  Vector3 toCart(const Geodetic& geod) const;

  /** Orientation of the Geodetic horizontal local frame.
   */
  Quaternion getGeodHLOrientation(const Geodetic& pos) const;

  /** Orientation of the Geodetic horizontal local frame.
   */
  Quaternion getGeodHLOrientation(const Vector3& pos) const;

private:
  /**
   */
  real_type a;
  real_type f;
  real_type a2;
  real_type ra2;
  real_type e;
  real_type e2;
  real_type e4;
  real_type GM;
  real_type mOmega;
};

/** Pretty printing of geodetic coordinates.
 */
std::ostream& operator<<(std::ostream& os, const Geodetic& geod);

} // namespace OpenFDM

#endif
