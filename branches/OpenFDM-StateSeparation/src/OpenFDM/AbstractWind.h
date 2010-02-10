/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractWind_H
#define OpenFDM_AbstractWind_H

#include "Referenced.h"
#include "Vector.h"

namespace OpenFDM {

class Environment;

class AbstractWind : public Referenced {
public:
  virtual ~AbstractWind();
  virtual Vector6 getWindVelocity(const Environment&, const real_type& t,
                                  const Vector3&) const;
};

} // namespace OpenFDM

#endif
