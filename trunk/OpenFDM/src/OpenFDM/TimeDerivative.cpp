/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "TimeDerivative.h"

namespace OpenFDM {

TimeDerivative::TimeDerivative(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  setNumInputPorts(1);

  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &TimeDerivative::getDerivativeOutput);

  addProperty("output", Property(this, &TimeDerivative::getDerivativeOutput));
}

TimeDerivative::~TimeDerivative(void)
{
}

bool
TimeDerivative::init(void)
{
  if (!getInputPort(0)->isConnected())
    return false;

  mDerivativeOutput.resize(getInputPort(0)->getValue().toMatrix());

  // Set a mark for the first step.
  mDt = 0.0;
  return true;
}

void
TimeDerivative::output(const TaskInfo&)
{
  OpenFDMAssert(getInputPort(0)->isConnected());
  MatrixPortHandle mh = getInputPort(0)->toMatrixPortHandle();
  // If we are here at the first time, dt is set to zero.
  // So, computing a derivative is not possible in the first step.
  // Prepare zero output in this case.
  if (mDt != 0.0) {
    OpenFDMAssert(size(mh.getMatrixValue()) == size(mPastInput));
    if (size(mh.getMatrixValue()) == size(mPastInput)) {
      mDerivativeOutput = mh.getMatrixValue() - mPastInput;
      mDerivativeOutput *= 1/mDt;
    }
  } else {
    mDerivativeOutput.resize(mh.getMatrixValue());
    mDerivativeOutput.clear();
  }
}

void
TimeDerivative::update(const TaskInfo& taskInfo)
{
  OpenFDMAssert(getInputPort(0)->isConnected());

  // FIXME
  real_type dt = (*taskInfo.getSampleTimeSet().begin()).getSampleTime();
  // Updating is just storing required information for the next output step.
  MatrixPortHandle mh = getInputPort(0)->toMatrixPortHandle();
  mPastInput = mh.getMatrixValue();
  mDt = dt;
}

const Matrix&
TimeDerivative::getDerivativeOutput(void) const
{
  return mDerivativeOutput;
}

} // namespace OpenFDM
