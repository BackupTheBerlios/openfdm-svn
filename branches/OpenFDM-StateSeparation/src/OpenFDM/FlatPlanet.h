/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_FlatPlanet_H
#define OpenFDM_FlatPlanet_H

#include "Types.h"
#include "Vector.h"
#include "AbstractPlanet.h"

namespace OpenFDM {

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

  /** Transform cartesian coordinates to geodetic coordinates.
   */
  virtual Geodetic toGeod(const Vector3& cart) const;

  /** Transform geodetic coordinates to cartesian coordinates.
   */
  virtual Vector3 toCart(const Geodetic& geod) const;
};

} // namespace OpenFDM

#endif
