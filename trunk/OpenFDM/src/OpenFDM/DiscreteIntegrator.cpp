/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "DiscreteIntegrator.h"

namespace OpenFDM {

DiscreteIntegrator::DiscreteIntegrator(const std::string& name) :
  Model(name)
{
  setNumInputPorts(1);
  setInputPortName(0, "derivative");

  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &DiscreteIntegrator::getIntegralOutput);

  addProperty("initialValue", Property(this, &DiscreteIntegrator::getInitialValue, &DiscreteIntegrator::setInitialValue));
  addProperty("minSaturation", Property(this, &DiscreteIntegrator::getMinSaturation, &DiscreteIntegrator::setMinSaturation));
  addProperty("maxSaturation", Property(this, &DiscreteIntegrator::getMaxSaturation, &DiscreteIntegrator::setMaxSaturation));

  addProperty("output", Property(this, &DiscreteIntegrator::getIntegralOutput));
}

DiscreteIntegrator::~DiscreteIntegrator(void)
{
}

bool
DiscreteIntegrator::init(void)
{
  OpenFDMAssert(getInputPort(0)->isConnected());

  // The initial value defaults to zero
  if (rows(mInitialValue) == 0 || cols(mInitialValue) == 0) {
    mInitialValue.resize(getInputPort(0)->getValue().toMatrix());
    mInitialValue.clear();
  }

  if (size(mMinSaturation) != Size(0, 0)) {
    OpenFDMAssert(size(mMinSaturation) == size(mInitialValue));
  }
  if (size(mMaxSaturation) != Size(0, 0)) {
    OpenFDMAssert(size(mMaxSaturation) == size(mInitialValue));
  }

  setNumDiscreteStates(rows(mInitialValue)*cols(mInitialValue));

  mIntegralState = mInitialValue;
  mIntegralOutput = mIntegralState;

  return true;
}

void
DiscreteIntegrator::output(const TaskInfo&)
{
  mIntegralOutput = mIntegralState;
}

void
DiscreteIntegrator::update(const TaskInfo& taskInfo)
{
  OpenFDMAssert(getInputPort(0)->isConnected());

  // Just compute the integral.
  // FIXME: make sure this is the only dt ...
  real_type dt = (*taskInfo.getSampleTimeSet().begin()).getSampleTime();
  MatrixPortHandle mh = getInputPort(0)->toMatrixPortHandle();
  if (size(mh.getMatrixValue()) == size(mIntegralState))
    mIntegralState += dt*mh.getMatrixValue();

  // Hmm, should that be done on state setting too???
  if (size(mMaxSaturation) == size(mInitialValue))
    mIntegralState = LinAlg::min(mIntegralState, mMaxSaturation);
  if (size(mMinSaturation) == size(mInitialValue))
    mIntegralState = LinAlg::max(mIntegralState, mMinSaturation);
}

void
DiscreteIntegrator::setDiscreteState(const StateStream& state)
{
  state.readSubState(mIntegralState);
}

void
DiscreteIntegrator::getDiscreteState(StateStream& state) const
{
  state.writeSubState(mIntegralState);
}

const Matrix&
DiscreteIntegrator::getInitialValue(void) const
{
  return mInitialValue;
}

void
DiscreteIntegrator::setInitialValue(const Matrix& value)
{
  mInitialValue = value;
}

const Matrix&
DiscreteIntegrator::getMinSaturation(void) const
{
  return mMinSaturation;
}

void
DiscreteIntegrator::setMinSaturation(const Matrix& value)
{
  mMinSaturation = value;
}

const Matrix&
DiscreteIntegrator::getMaxSaturation(void) const
{
  return mMaxSaturation;
}

void
DiscreteIntegrator::setMaxSaturation(const Matrix& value)
{
  mMaxSaturation = value;
}

const Matrix&
DiscreteIntegrator::getIntegralOutput(void) const
{
  return mIntegralOutput;
}

} // namespace OpenFDM
