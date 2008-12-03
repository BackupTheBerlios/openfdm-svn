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
};

} // namespace OpenFDM

#endif
