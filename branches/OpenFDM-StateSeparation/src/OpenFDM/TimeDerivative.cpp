/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "TimeDerivative.h"

#include "Assert.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(TimeDerivative, Model)
  END_OPENFDM_OBJECT_DEF

TimeDerivative::TimeDerivative(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  setNumInputPorts(1);

  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &TimeDerivative::getDerivativeOutput);
}

TimeDerivative::~TimeDerivative(void)
{
}

bool
TimeDerivative::init(void)
{
  mInputPort = getInputPort(0)->toMatrixPortHandle();
  if (!mInputPort.isConnected()) {
    Log(Model, Error) << "Initialization of TimeDerivative model \""
                      << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }

  mDerivativeOutput.resize(mInputPort.getMatrixValue());

  // Set a mark for the first step.
  mDt = 0.0;

  return Model::init();
}

void
TimeDerivative::output(const TaskInfo&)
{
  // If we are here at the first time, dt is set to zero.
  // So, computing a derivative is not possible in the first step.
  // Prepare zero output in this case.
  if (mDt != 0.0) {
    OpenFDMAssert(size(mInputPort.getMatrixValue()) == size(mPastInput));
    if (size(mInputPort.getMatrixValue()) == size(mPastInput)) {
      mDerivativeOutput = mInputPort.getMatrixValue() - mPastInput;
      mDerivativeOutput *= 1/mDt;
    }
  } else {
    mDerivativeOutput.resize(mInputPort.getMatrixValue());
    mDerivativeOutput.clear();
  }
}

void
TimeDerivative::update(const TaskInfo& taskInfo)
{
  // FIXME
  real_type dt = (*taskInfo.getSampleTimeSet().begin()).getSampleTime();
  // Updating is just storing required information for the next output step.
  mPastInput = mInputPort.getMatrixValue();
  mDt = dt;
}

const Matrix&
TimeDerivative::getDerivativeOutput(void) const
{
  return mDerivativeOutput;
}

} // namespace OpenFDM
