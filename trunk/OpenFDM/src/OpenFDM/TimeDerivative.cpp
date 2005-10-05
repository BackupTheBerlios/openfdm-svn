/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Types.h"
#include "Object.h"
#include "Expression.h"
#include "TimeDerivative.h"

namespace OpenFDM {

TimeDerivative::TimeDerivative(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  setNumInputPorts(1);

  setNumOutputPorts(1);
  setOutputPort(0, "output", Property(this, &TimeDerivative::getDerivativeOutput));
  addProperty("output", Property(this, &TimeDerivative::getDerivativeOutput));
}

TimeDerivative::~TimeDerivative(void)
{
}

bool
TimeDerivative::init(void)
{
  if (!getInputPort(0).isValid())
    return false;

  mDerivativeOutput.resize(getInputPort(0).getValue().toMatrix());

  // Set a mark for the first step.
  mDt = 0.0;
  return true;
}

void
TimeDerivative::output(void)
{
  OpenFDMAssert(getInputPort(0).isValid());

  // If we are here at the first time, dt is set to zero.
  // So, computing a derivative is not possible in the first step.
  // Prepare zero output in this case.
  if (mDt != 0.0) {
    Matrix input = getInputPort(0).getValue().toMatrix();
    OpenFDMAssert(size(input) == size(mPastInput));
    if (size(input) == size(mPastInput)) {
      mDerivativeOutput = input - mPastInput;
      mDerivativeOutput *= 1/mDt;
    }
  } else {
    mDerivativeOutput.resize(getInputPort(0).getValue().toMatrix());
    mDerivativeOutput.clear();
  }
}

void
TimeDerivative::update(real_type dt)
{
  OpenFDMAssert(getInputPort(0).isValid());

  // Updating is just storing required information for the next output step.
  mPastInput = getInputPort(0).getValue().toMatrix();
  mDt = dt;
}

const Matrix&
TimeDerivative::getDerivativeOutput(void) const
{
  return mDerivativeOutput;
}

} // namespace OpenFDM
