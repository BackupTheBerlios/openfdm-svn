/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "Vector.h"
#include "LinearSpring.h"

namespace OpenFDM {

LinearSpring::LinearSpring(const std::string& name) :
  Model(name),
  mSpringReference(0),
  mSpringConstant(0),
  mDamperConstant(0)
{
}

LinearSpring::~LinearSpring(void)
{
}

bool
LinearSpring::init(void)
{
  if (!getInputPort(0)->isConnected()) {
    Log(Model, Error) << "Initialization of AirSpring model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }
  mPositionPort = getInputPort(0)->toRealPortHandle();

  if (!getInputPort(1)->isConnected()) {
    Log(Model, Error) << "Initialization of AirSpring model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(1)
                      << "\" is not connected!" << endl;
    return false;
  }
  mVelocityPort = getInputPort(1)->toRealPortHandle();

  return true;
}

void
LinearSpring::output(const TaskInfo& taskInfo)
{
  real_type position = mPositionPort.getRealValue();
  real_type vel = mVelocityPort.getRealValue();
  real_type displacement = position - mSpringReference;
  mForce = mSpringConstant*displacement + vel*mDamperConstant;
}

const real_type&
LinearSpring::getForce(void) const
{
  return mForce;
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
