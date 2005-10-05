/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ExplicitEuler_H
#define OpenFDM_ExplicitEuler_H

#include "Object.h"
#include "ODESolver.h"

namespace OpenFDM {

class ExplicitEuler
  : public ODESolver {
public:
  ExplicitEuler(void);
  virtual ~ExplicitEuler(void);

  virtual bool integrate(real_type toTEnd);
};

} // namespace OpenFDM

#endif
