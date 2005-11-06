/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LineActuator_H
#define OpenFDM_LineActuator_H

#include "Model.h"
#include "Vector.h"
#include "LineForce.h"

namespace OpenFDM {

/// Linear spring damper model
class LineActuator :
    public LineForce {
public:
  LineActuator(const std::string& name);
  virtual ~LineActuator(void);

  virtual void output(const TaskInfo& taskInfo);

  real_type getProportionalGain(void) const;
  void setProportionalGain(real_type proportionalGain);

  real_type getDerivativeGain(void) const;
  void setDerivativeGain(real_type derivativeGain);

private:
  real_type mProportionalGain;
  real_type mDerivativeGain;
};

} // namespace OpenFDM

#endif
