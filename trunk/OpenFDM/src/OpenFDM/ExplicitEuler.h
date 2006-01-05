/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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
  virtual bool denseOutput(real_type t, Vector& out);

private:
  /// Vector storing the derivative of that step. That is used for
  /// dense output.
  Vector mDeriv;
};

} // namespace OpenFDM

#endif
