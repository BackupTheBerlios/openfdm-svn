/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_EnvironmentObject_H
#define OpenFDM_EnvironmentObject_H

#include "Object.h"
#include "WeakPtr.h"
#include "Environment.h"

namespace OpenFDM {

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
