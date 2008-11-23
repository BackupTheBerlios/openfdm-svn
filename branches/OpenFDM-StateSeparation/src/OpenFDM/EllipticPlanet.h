/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_EllipticPlanet_H
#define OpenFDM_EllipticPlanet_H

#include "Types.h"
#include "Vector.h"
#include "Quaternion.h"
#include "AbstractPlanet.h"

namespace OpenFDM {

/**
 * The EllipticPlanet class.
 *
 * It holds some information about the planet the simulation is running on.
 */
class EllipticPlanet : public AbstractPlanet {
public:
  /** Elliptic constructor.
   */
  EllipticPlanet(void);

  /** Elliptic destructor.
   */
  virtual ~EllipticPlanet(void);

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
};

} // namespace OpenFDM

#endif
