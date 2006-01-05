/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Vector.h"
#include "DirectForce.h"

namespace OpenFDM {

DirectForce::DirectForce(const std::string& name, const Vector6& direction) :
  ExternalForce(name),
  mDirection(direction),
  mMagnitude(0)
{
  addProperty("direction", Property(this, &DirectForce::getDirection, &DirectForce::setDirection));
  addProperty("magnitude", Property(this, &DirectForce::getMagnitude));

  setDirectFeedThrough(true);

  setNumInputPorts(1);
  setInputPortName(0, "magnitude");
}

DirectForce::~DirectForce(void)
{
}

void
DirectForce::setDirection(const Vector6& direction)
{
  mDirection = direction;
}

const Vector6&
DirectForce::getDirection(void) const
{
  return mDirection;
}

real_type
DirectForce::getMagnitude(void) const
{
  return mMagnitude;
}

bool
DirectForce::init(void)
{
  return true;
}

void
DirectForce::output(const TaskInfo&)
{
  RealPortHandle rh = getInputPort(0)->toRealPortHandle();
  mMagnitude = rh.getRealValue();
  setForce(mMagnitude*mDirection);
}

} // namespace OpenFDM
