/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DiscBrake_H
#define OpenFDM_DiscBrake_H

#include "Model.h"
#include "Vector.h"
#include "LineForce.h"

namespace OpenFDM {

/// Linear spring damper model
class DiscBrake :
    public LineForce {
public:
  DiscBrake(const std::string& name);
  virtual ~DiscBrake(void);

  virtual void output(const TaskInfo& taskInfo);

  real_type getFrictionConstant(void) const;
  void setFrictionConstant(real_type frictionConstant);

private:
  real_type mFrictionConstant;
};

} // namespace OpenFDM

#endif
