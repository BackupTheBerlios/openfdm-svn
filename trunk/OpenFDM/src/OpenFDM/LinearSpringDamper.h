/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinearSpringDamper_H
#define OpenFDM_LinearSpringDamper_H

#include <string>

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

/// Linear spring damper model
class LinearSpringDamper :
    public Model {
public:
  LinearSpringDamper(const std::string& name);
  virtual ~LinearSpringDamper(void);

  virtual bool init(void);
  virtual void output(const TaskInfo& taskInfo);

  const real_type& getForce(void) const;

  const real_type& getSpringReference(void) const;
  void setSpringReference(const real_type& springReference);

  const real_type& getSpringConstant(void) const;
  void setSpringConstant(const real_type& springConstant);

  const real_type& getDamperConstant(void) const;
  void setDamperConstant(const real_type& damperConstant);

private:
  real_type mSpringReference;
  real_type mSpringConstant;
  real_type mDamperConstant;

  real_type mForce;

  /// The intput port which must provide the position
  RealPortHandle mPositionPort;
  /// The intput port which must provide the velocity
  RealPortHandle mVelocityPort;
};

} // namespace OpenFDM

#endif
