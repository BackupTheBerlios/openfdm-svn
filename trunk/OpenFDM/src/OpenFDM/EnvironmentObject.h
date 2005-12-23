/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_EnvironmentObject_H
#define OpenFDM_EnvironmentObject_H

#include "Object.h"

namespace OpenFDM {

class Environment;

class EnvironmentObject :
    public Object {
public:
  EnvironmentObject(void);
  virtual ~EnvironmentObject(void);

  Environment* getEnvironment(void)
  { return mEnvironment; }
  const Environment* getEnvironment(void) const
  { return mEnvironment; }

private:
  void attachToEnvironment(Environment* environment);

  WeakPtr<Environment> mEnvironment;

  friend class Environment;
};

} // namespace OpenFDM

#endif
