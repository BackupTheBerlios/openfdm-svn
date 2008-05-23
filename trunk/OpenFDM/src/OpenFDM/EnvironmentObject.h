/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

  SharedPtr<Environment> getEnvironment(void)
  { return mEnvironment.lock(); }
  SharedPtr<const Environment> getEnvironment(void) const
  { return mEnvironment.lock(); }

private:
  void attachToEnvironment(Environment* environment);

  WeakPtr<Environment> mEnvironment;

  friend class Environment;
};

} // namespace OpenFDM

#endif
