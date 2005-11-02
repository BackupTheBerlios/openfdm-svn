/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinearSpring_H
#define OpenFDM_LinearSpring_H

#include "Model.h"
#include "Vector.h"
#include "LineForce.h"

namespace OpenFDM {

/// Linear spring damper model
class LinearSpring :
    public LineForce {
public:
  LinearSpring(const std::string& name);
  virtual ~LinearSpring(void);

  virtual void output(const TaskInfo& taskInfo);

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
};

} // namespace OpenFDM

#endif
