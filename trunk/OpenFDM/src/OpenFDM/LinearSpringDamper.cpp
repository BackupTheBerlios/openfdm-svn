/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "Vector.h"
#include "LinearSpringDamper.h"

namespace OpenFDM {

LinearSpringDamper::LinearSpringDamper(const std::string& name) :
  Model(name),
  mSpringReference(0),
  mSpringConstant(0),
  mDamperConstant(0)
{
  setDirectFeedThrough(true);

  setNumInputPorts(2);
  setInputPortName(0, "position");
  setInputPortName(1, "velocity");
  
  setNumOutputPorts(1);
  setOutputPort(0, "force", this, &LinearSpringDamper::getForce);
}

LinearSpringDamper::~LinearSpringDamper(void)
{
}

bool
LinearSpringDamper::init(void)
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
LinearSpringDamper::output(const TaskInfo& taskInfo)
{
  real_type position = mPositionPort.getRealValue();
  real_type vel = mVelocityPort.getRealValue();
  real_type displacement = position - mSpringReference;
  mForce = mSpringConstant*displacement + vel*mDamperConstant;
}

const real_type&
LinearSpringDamper::getForce(void) const
{
  return mForce;
}

real_type
LinearSpringDamper::getSpringReference(void) const
{
  return mSpringReference;
}

void
LinearSpringDamper::setSpringReference(real_type springReference)
{
  mSpringReference = springReference;
}

real_type
LinearSpringDamper::getSpringConstant(void) const
{
  return mSpringConstant;
}

void
LinearSpringDamper::setSpringConstant(real_type springConstant)
{
  mSpringConstant = springConstant;
}

real_type
LinearSpringDamper::getDamperConstant(void) const
{
  return mDamperConstant;
}

void
LinearSpringDamper::setDamperConstant(real_type damperConstant)
{
  mDamperConstant = damperConstant;
}

} // namespace OpenFDM
