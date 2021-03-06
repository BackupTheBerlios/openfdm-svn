/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Tank.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Tank, Mass)
/// This is a state value ???!!!
  DEF_OPENFDM_PROPERTY(Real, Content, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Capacity, Serialized)
  END_OPENFDM_OBJECT_DEF

Tank::Tank(const std::string& name) :
  Mass(name),
  mContent(0),
  mNextContent(0),
  mCapacity(0),
  mIsEmpty(true)
{
  setNumInputPorts(1);
  setInputPortName(0, "fuelFlow");

  setNumOutputPorts(1);
  setOutputPort(0, "content", this, &Tank::getContent);
}

Tank::~Tank(void)
{
}

bool
Tank::init(void)
{
  mInputPort = getInputPort(0)->toRealPortHandle();
  return Mass::init();
}

void
Tank::output(const TaskInfo& taskInfo)
{
  if (!nonZeroIntersection(taskInfo.getSampleTimeSet(),
                           SampleTime::PerTimestep))
    return;
  mContent = mNextContent;
  setInertia(mContent);
  mIsEmpty = (mContent == 0);
}

void
Tank::update(const TaskInfo& taskInfo)
{
  if (!nonZeroIntersection(taskInfo.getSampleTimeSet(),
                           SampleTime::PerTimestep))
    return;

  if (!mInputPort.isConnected())
    return;

  // Modify the tank's content by the requested fluel flow (kg/s)
  // FIXME
  real_type dt = (*taskInfo.getSampleTimeSet().begin()).getSampleTime();
  mNextContent = mContent + dt*mInputPort.getRealValue();
  mNextContent = min(mNextContent, mCapacity);
  mNextContent = max(mNextContent, real_type(0));
}

void
Tank::setDiscreteState(const StateStream& state)
{
  state.readSubState(mContent);
}

void
Tank::getDiscreteState(StateStream& state) const
{
  state.writeSubState(mContent);
}

const real_type&
Tank::getContent(void) const
{
  return mContent;
}

void
Tank::setContent(const real_type& content)
{
  mContent = content;
}

const real_type&
Tank::getCapacity(void) const
{
  return mCapacity;
}

void
Tank::setCapacity(const real_type& capacity)
{
  mCapacity = capacity;
}

} // namespace OpenFDM
