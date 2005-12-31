/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinearSpring_H
#define OpenFDM_LinearSpring_H

#include <string>

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

/// Linear spring damper model
class LinearSpring :
    public Model {
public:
  LinearSpring(const std::string& name);
  virtual ~LinearSpring(void);

  virtual bool init(void);
  virtual void output(const TaskInfo& taskInfo);

  const real_type& getForce(void) const;

  real_type getSpringReference(void) const;
  void setSpringReference(real_type springReference);

  real_type getSpringConstant(void) const;
  void setSpringConstant(real_type springConstant);

  real_type getDamperConstant(void) const;
  void setDamperConstant(real_type damperConstant);

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
