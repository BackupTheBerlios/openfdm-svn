/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LineForce_H
#define OpenFDM_LineForce_H

#include "Model.h"
#include "Vector.h"

namespace OpenFDM {

class LineForce :
    public Model {
public:
  LineForce(const std::string& name);
  virtual ~LineForce(void);

  void computeForce(real_type position, real_type vel);

  real_type getPosition(void) const
  { return mPosition; }
  real_type getVel(void) const
  { return mVel; }

  real_type getForce(void) const
  { return mForce; }

protected:
  void setForce(real_type force)
  { mForce = force; }

private:
  real_type mPosition;
  real_type mVel;
  real_type mForce;
};

} // namespace OpenFDM

#endif
