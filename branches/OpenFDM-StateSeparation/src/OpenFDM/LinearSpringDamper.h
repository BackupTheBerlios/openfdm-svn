/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinearSpringDamper_H
#define OpenFDM_LinearSpringDamper_H

#include <string>

#include "Model.h"
#include "RealInputPort.h"
#include "RealOutputPort.h"

namespace OpenFDM {

/// Linear spring damper model
class LinearSpringDamper : public Model {
  OPENFDM_OBJECT(LinearSpringDamper, Model);
public:
  LinearSpringDamper(const std::string& name);
  virtual ~LinearSpringDamper(void);

  virtual void output(const Task&, const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;

  const real_type& getSpringReference(void) const;
  void setSpringReference(const real_type& springReference);

  const real_type& getSpringConstant(void) const;
  void setSpringConstant(const real_type& springConstant);

  const real_type& getDamperConstant(void) const;
  void setDamperConstant(const real_type& damperConstant);

private:
  RealInputPort mPositionPort;
  RealInputPort mVelocityPort;
  RealOutputPort mForcePort;

  real_type mSpringReference;
  real_type mSpringConstant;
  real_type mDamperConstant;
};

} // namespace OpenFDM

#endif
