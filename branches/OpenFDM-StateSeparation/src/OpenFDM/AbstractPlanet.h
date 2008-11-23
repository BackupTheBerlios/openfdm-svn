/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractPlanet_H
#define OpenFDM_AbstractPlanet_H

#include <iosfwd>

#include "Types.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Referenced.h"

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
 * Datatype for a position in speric coordinates.
 */
struct Geocentric {
  Geocentric(real_type lat = 0.0, real_type lon = 0.0, real_type rad = 0.0)
    : latitude(lat), longitude(lon), radius(rad)
  {}
  real_type latitude;
  real_type longitude;
  real_type radius;
};

// FIXME Find out what we need, Move the rest of the elliptic stuff in here
// into the EllipticPlanet class.
// We probably only need the altitude computation from the cartesian coordinates

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

  /** Transform cartesian coordinates to geodetic coordinates.
   */
  virtual Geodetic toGeod(const Vector3& cart) const = 0;

  /** Transform geodetic coordinates to cartesian coordinates.
   */
  virtual Vector3 toCart(const Geodetic& geod) const = 0;

  /** Transform cartesian coordinates to geocentric coordinates.
   */
  Geocentric toGeoc(const Vector3& cart) const;

  /** Transform geocentric coordinates to cartesian coordinates.
   */
  Vector3 toCart(const Geocentric& geoc) const;

  /** Transform geocentric coordinates to geodetic coordinates.
   */
  Geocentric toGeoc(const Geodetic& geod) const;

  /** Transform geodetic coordinates to geocentric coordinates.
   */
  Geodetic toGeod(const Geocentric& geoc) const;

  /** Orientation of the Geodetic horizontal local frame.
   */
  Quaternion getGeodHLOrientation(const Geodetic& pos) const;

  /** Orientation of the Geodetic horizontal local frame.
   */
  Quaternion getGeodHLOrientation(const Vector3& pos) const;

  /** Orientation of the Geodetic horizontal local frame.
   */
  Quaternion getGeodHLOrientation(const Geocentric& pos) const;

  /** Orientation of the Geocentric horizontal local frame.
   */
  Quaternion getGeocHLOrientation(const Geodetic& pos) const;

  /** Orientation of the Geocentric horizontal local frame.
   */
  Quaternion getGeocHLOrientation(const Vector3& pos) const;

  /** Orientation of the Geocentric horizontal local frame.
   */
  Quaternion getGeocHLOrientation(const Geocentric& pos) const;

  /** Rotation rate of the Geocentric horizontal local frame.
   */
  Vector3 getGoecHLRate(const Geocentric& pos, const Vector3& ecVel) const;

  /** Rotation rate of the Geocentric horizontal local frame.
   */
  Vector3 getGoecHLRate(const Vector3& pos, const Vector3& ecVel) const;

  /** Rotation rate of the Geocentric horizontal local frame.
   */
  Vector3 getGoecHLRate(const Geodetic& pos, const Vector3& ecVel) const;
};

/** Pretty printing of geodetic coordinates.
 */
std::ostream& operator<<(std::ostream& os, const Geodetic& geod);

/** Pretty printing of geocentric coordinates.
 */
std::ostream& operator<<(std::ostream& os, const Geocentric& geoc);

} // namespace OpenFDM

#endif
