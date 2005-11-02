/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "Vector.h"
#include "LinearSpring.h"

namespace OpenFDM {

LinearSpring::LinearSpring(const std::string& name) :
  LineForce(name),
  mSpringReference(0),
  mSpringConstant(0),
  mDamperConstant(0)
{
}

LinearSpring::~LinearSpring(void)
{
}

void
LinearSpring::output(const TaskInfo& taskInfo)
{
  real_type displacement = getPosition() - mSpringReference;
  setForce(mSpringConstant*displacement + getVel()*mDamperConstant);
}

real_type
LinearSpring::getSpringReference(void) const
{
  return mSpringReference;
}

void
LinearSpring::setSpringReference(real_type springReference)
{
  mSpringReference = springReference;
}

real_type
LinearSpring::getSpringConstant(void) const
{
  return mSpringConstant;
}

void
LinearSpring::setSpringConstant(real_type springConstant)
{
  mSpringConstant = springConstant;
}

real_type
LinearSpring::getDamperConstant(void) const
{
  return mDamperConstant;
}

void
LinearSpring::setDamperConstant(real_type damperConstant)
{
  mDamperConstant = damperConstant;
}

} // namespace OpenFDM
