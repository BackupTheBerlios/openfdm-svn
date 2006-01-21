/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "DiscreteIntegrator.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(DiscreteIntegrator)
  END_OPENFDM_OBJECT_DEF

DiscreteIntegrator::DiscreteIntegrator(const std::string& name) :
  Model(name)
{
  setNumInputPorts(2);
  setInputPortName(0, "derivative");
  setInputPortName(1, "initialValue");

  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &DiscreteIntegrator::getIntegralOutput);

  addStoredProperty("initialValue", Property(this, &DiscreteIntegrator::getInitialValue, &DiscreteIntegrator::setInitialValue));
  addStoredProperty("minSaturation", Property(this, &DiscreteIntegrator::getMinSaturation, &DiscreteIntegrator::setMinSaturation));
  addStoredProperty("maxSaturation", Property(this, &DiscreteIntegrator::getMaxSaturation, &DiscreteIntegrator::setMaxSaturation));
}

DiscreteIntegrator::~DiscreteIntegrator(void)
{
}

bool
DiscreteIntegrator::init(void)
{
  mDerivativePort = getInputPort(0)->toMatrixPortHandle();
  if (!mDerivativePort.isConnected()) {
    Log(Model,Error) << "Input port to DiscreteIntegrator Model \""
                     << getName() << "\" is not connected" << endl;
    return false;
  }

  // The initial value defaults to zero
  if (getInputPort(1)->isConnected()) {
    MatrixPortHandle mh = getInputPort(1)->toMatrixPortHandle();
    mIntegralState = mh.getMatrixValue();
  } else {
    if (rows(mInitialValue) == 0 || cols(mInitialValue) == 0) {
      mInitialValue.resize(getInputPort(0)->getValue().toMatrix());
      mInitialValue.clear();
    }
    mIntegralState = mInitialValue;
  }
  mIntegralOutput = mIntegralState;

  if (size(mMinSaturation) != Size(0, 0)) {
    OpenFDMAssert(size(mMinSaturation) == size(mIntegralState));
  }
  if (size(mMaxSaturation) != Size(0, 0)) {
    OpenFDMAssert(size(mMaxSaturation) == size(mIntegralState));
  }

  setNumDiscreteStates(rows(mIntegralState)*cols(mIntegralState));
  
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
  OpenFDMAssert(mDerivativePort.isConnected());

  // Just compute the integral.
  // FIXME: make sure this is the only dt ...
  real_type dt = (*taskInfo.getSampleTimeSet().begin()).getSampleTime();
  mIntegralState += dt*mDerivativePort.getMatrixValue();

  // Hmm, should that be done on state setting too???
  if (size(mMaxSaturation) == size(mIntegralState))
    mIntegralState = LinAlg::min(mIntegralState, mMaxSaturation);
  if (size(mMinSaturation) == size(mIntegralState))
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
