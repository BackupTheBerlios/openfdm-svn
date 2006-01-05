/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DefaultPlanet_H
#define OpenFDM_DefaultPlanet_H

#include "Types.h"
#include "Object.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Planet.h"

namespace OpenFDM {

/**
 * The DefaultPlanet class.
 *
 * It holds some information about the planet the simulation is running on.
 */
class DefaultPlanet
  : public Planet {
public:
  /** Default constructor.
   */
  DefaultPlanet(void);

  /** Default destructor.
   */
  virtual ~DefaultPlanet(void);

  /** Get aequatorial radius.
   */
  real_type getAequatorialRadius(void) const;

  /** Set aequatorial radius.
   */
  void setAequatorialRadius(real_type r);

  /** Set planet flattening.
   */
  void setFlattening(real_type flat);

  /** Transform cartesian coordinates to geodetic coordinates.
   */
  virtual Geodetic toGeod(const Vector3& cart) const;

  /** Transform geodetic coordinates to cartesian coordinates.
   */
  virtual Vector3 toCart(const Geodetic& geod) const;

private:
  /**
   */
  real_type a;
  real_type ra2;
  real_type e;
  real_type e2;
  real_type e4;

  real_type mRotationRate;
};

} // namespace OpenFDM

#endif
