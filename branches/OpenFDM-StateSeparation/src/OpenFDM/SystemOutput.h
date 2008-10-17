/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SystemOutput_H
#define OpenFDM_SystemOutput_H

#include "System.h"

namespace OpenFDM {

class SystemOutput : public Referenced {
public:
  virtual ~SystemOutput();

  void setSystem(const System* system);

  static SystemOutput* newDefaultSystemOutput(const std::string& filename);

  virtual void output(const real_type& t) = 0;
  virtual void attachTo(const System* system) = 0;

protected:
  SharedPtr<const System> getSystem() const
  { return mSystem.lock(); }

private:
  WeakPtr<const System> mSystem;
};

} // namespace OpenFDM

#endif
