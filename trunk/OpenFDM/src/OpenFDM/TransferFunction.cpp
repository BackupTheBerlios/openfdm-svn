/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "TransferFunction.h"

namespace OpenFDM {

DiscreteTransferFunction::DiscreteTransferFunction(const std::string& name) :
  Model(name),
  mDen(1),
  mNum(1)
{
  mNum(1) = 1;
  mDen(1) = 1;

  addProperty("numerator", Property(this, &DiscreteTransferFunction::getNumerator, &DiscreteTransferFunction::setNumerator));
  addProperty("denominator", Property(this, &DiscreteTransferFunction::getDenominator, &DiscreteTransferFunction::setDenominator));
  addProperty("output", Property(this, &DiscreteTransferFunction::getOutput));

  setDirectFeedThrough(true);

  setNumInputPorts(1);
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &DiscreteTransferFunction::getOutput);
}

DiscreteTransferFunction::~DiscreteTransferFunction(void)
{
}

void
DiscreteTransferFunction::setDenominator(const Vector& den)
{
  mDen = den;
}

const Vector&
DiscreteTransferFunction::getDenominator(void) const
{
  return mDen;
}

void
DiscreteTransferFunction::setNumerator(const Vector& num)
{
  mNum = num;
}

const Vector&
DiscreteTransferFunction::getNumerator(void) const
{
  return mNum;
}

bool
DiscreteTransferFunction::init(void)
{
  OpenFDMAssert(getInputPort(0)->isConnected());
  if (!getInputPort(0)->isConnected())
    return false;

  mNumNorm.resize(0);
  mDenNorm.resize(0);
  mD = 0;

  Log(Model, Info) << "Processing transfer function \"" << getName()
                   << "\", nummerator: " << trans(mNum)
                   << ", denominator" << trans(mDen) << endl;

  // Make sure the first, not stored coefficient in the denominator is 1
  // Rescale the other coefficient vectors
  for (unsigned i = 1; i <= rows(mDen); ++i) {
    if (mDen(i) != 0) {
      real_type rescale = 1/mDen(i);
      if (i+1 <= rows(mDen)) {
        mDenNorm = rescale*mDen(Range(i+1, rows(mDen)));
      } else {
        mDenNorm.resize(0);
      }
      mNumNorm = rescale*mNum;
      break;
    }
  }

  // For now ... FIXME
  if ((rows(mDenNorm) + 1) < rows(mNumNorm)) {
    Log(Model, Error) << "Cannot handle higher degree numerator than "
                      << "denoninator polynomials for transfer function \""
                      << getName() << "\"!" << endl;
    return false;
  }

  // Split out polynomials from that rational
  if (rows(mDenNorm) < rows(mNumNorm)) {
    mD = mNumNorm(1);
    if (2 <= rows(mNumNorm)) {
      Vector tmpNum = mNumNorm(Range(2, rows(mNumNorm)));
      mNumNorm = tmpNum(Range(1, rows(mDenNorm))) - mD*mDenNorm;
    } else {
      mNumNorm.resize(0);
    }
  }
  // Blow up the numerator with zeros if required
  if (rows(mNumNorm) < rows(mDenNorm)) {
    Vector tmpNum = mNumNorm;
    mNumNorm.resize(mDenNorm);
    mNumNorm.clear();
    mNumNorm(Range(1+rows(mNumNorm)-rows(tmpNum), rows(mNumNorm))) = tmpNum;
  }

  Log(Model, Info) << "Normalized Processing transfer function \"" << getName()
                   << "\", nummerator: " << trans(mNumNorm)
                   << ", denominator: " << trans(mDenNorm)
                   << ", direct factor: " << mD << endl;

  /// Start with zero state ...
  mState.resize(rows(mDenNorm));
  mState.clear();
  setNumDiscreteStates(rows(mDenNorm));

  return true;
}

void
DiscreteTransferFunction::output(const TaskInfo&)
{
  // Compute the output ...
  RealPortHandle rh = getInputPort(0)->toRealPortHandle();
  real_type input = rh.getRealValue();
  mOutput = dot(mNumNorm, mState) + mD*input;
}

void
DiscreteTransferFunction::update(const TaskInfo& taskInfo)
{
  // FIXME: make sure this is the only dt ...
  real_type dt = (*taskInfo.getSampleTimeSet().begin()).getSampleTime();

  if (0 < rows(mState)) {
    // FIXME: use exponential integration scheme here ...
    // looks very benificial, since it is exact here!
    RealPortHandle rh = getInputPort(0)->toRealPortHandle();
    real_type input = rh.getRealValue();
    if (mState.size() == 1) {
      /// On dimensional exponetial integrator ...
      real_type z = -dt*dot(mDenNorm, mState);
      // Well, pade approximation is the right thing, but for now ...
      if (fabs(exp(z) - 1) <= sqrt(Limits<real_type>::epsilon()))
        mState(1) += dt*(input + z);
      else
        mState(1) += dt*(exp(z)-1)/z * (input + z);
    } else {
      Vector tmpState(mState);
      mState(1) += dt*(input - dot(mDenNorm, tmpState));
      for (unsigned i = 2; i <= rows(mState); ++i)
        mState(i) += dt*tmpState(i-1);
    }
  }
}

void
DiscreteTransferFunction::setDiscreteState(const StateStream& state)
{
  state.readSubState(mState);
}

void
DiscreteTransferFunction::getDiscreteState(StateStream& state) const
{
  state.writeSubState(mState);
}

const real_type&
DiscreteTransferFunction::getOutput(void) const
{
  return mOutput;
}

}
