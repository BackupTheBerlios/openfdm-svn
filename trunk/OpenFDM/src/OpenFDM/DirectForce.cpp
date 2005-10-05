/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Vector.h"
#include "DirectForce.h"

namespace OpenFDM {

DirectForce::DirectForce(const std::string& name, const Vector6& direction)
  : ExternalForce(name), mDirection(direction)
{
  addProperty("position", Property(this, &DirectForce::getPosition, &DirectForce::setPosition));
  addProperty("orientation", Property(this, &DirectForce::getOrientation, &DirectForce::setOrientation));
  addProperty("direction", Property(this, &DirectForce::getDirection, &DirectForce::setDirection));
  addProperty("magnitude", Property(this, &DirectForce::getMagnitude));
  addProperty("force", Property(this, &DirectForce::getForce));

  setDirectFeedThrough(true);

  setNumInputPorts(1);
  setInputPortName(0, "magnitude");
  setNumOutputPorts(1);
  setOutputPort(0, "force", Property(this, &DirectForce::getForce));
}

DirectForce::~DirectForce(void)
{
}

void
DirectForce::setPosition(const Vector3& p)
{
  mPosition = p;
}

const Vector3&
DirectForce::getPosition(void) const
{
  return mPosition;
}

void
DirectForce::setOrientation(const Quaternion& o)
{
  mOrientation = o;
}

const Quaternion&
DirectForce::getOrientation(void) const
{
  return mOrientation;
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

const Vector6&
DirectForce::getForce(void) const
{
  return mForce;
}

bool
DirectForce::init(void)
{
  return true;
}

void
DirectForce::output(void)
{
  mMagnitude = getInputPort(0).getValue().toReal();
  mForce = mMagnitude*mDirection;
}

void
DirectForce::computeForce(void)
{
  applyForce(forceFrom(mPosition, mOrientation, mForce));
}

} // namespace OpenFDM
