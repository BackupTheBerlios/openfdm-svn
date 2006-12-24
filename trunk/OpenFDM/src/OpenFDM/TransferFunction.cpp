/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "TransferFunction.h"

#include "Assert.h"
#include "LogStream.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(DiscreteTransferFunction, Model)
  DEF_OPENFDM_PROPERTY(Vector, Numerator, Serialized)
  DEF_OPENFDM_PROPERTY(Vector, Denominator, Serialized)
  END_OPENFDM_OBJECT_DEF

DiscreteTransferFunction::DiscreteTransferFunction(const std::string& name) :
  Model(name),
  mDen(1),
  mNum(1)
{
  mNum(0) = 1;
  mDen(0) = 1;

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
  mInputPort = getInputPort(0)->toRealPortHandle();
  if (!mInputPort.isConnected()) {
    Log(Model, Error) << "Initialization of DiscreteTransferFunction model \""
                      << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }

  mNumNorm.resize(0);
  mDenNorm.resize(0);
  mD = 0;

  Log(Model, Info) << "Processing transfer function \"" << getName()
                   << "\", nummerator: " << trans(mNum)
                   << ", denominator" << trans(mDen) << endl;

  // Make sure the first, not stored coefficient in the denominator is 1
  // Rescale the other coefficient vectors
  for (unsigned i = 0; i < rows(mDen); ++i) {
    if (mDen(i) != 0) {
      real_type rescale = 1/mDen(i);
      if (i+1 <= rows(mDen)) {
        mDenNorm = rescale*mDen(Range(i+1, rows(mDen)-1));
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
    mD = mNumNorm(0);
    if (2 <= rows(mNumNorm)) {
      Vector tmpNum = mNumNorm(Range(1, rows(mNumNorm)-1));
      mNumNorm = tmpNum(Range(0, rows(mDenNorm)-1)) - mD*mDenNorm;
    } else {
      mNumNorm.resize(0);
    }
  }
  // Blow up the numerator with zeros if required
  if (rows(mNumNorm) < rows(mDenNorm)) {
    Vector tmpNum = mNumNorm;
    mNumNorm.resize(mDenNorm);
    mNumNorm.clear();
    mNumNorm(Range(rows(mNumNorm)-rows(tmpNum), rows(mNumNorm)-1)) = tmpNum;
  }

  Log(Model, Info) << "Normalized Processing transfer function \"" << getName()
                   << "\", nummerator: " << trans(mNumNorm)
                   << ", denominator: " << trans(mDenNorm)
                   << ", direct factor: " << mD << endl;

  /// Start with zero state ...
  mState.resize(rows(mDenNorm));
  mState.clear();
  setNumDiscreteStates(rows(mDenNorm));

  return Model::init();
}

void
DiscreteTransferFunction::output(const TaskInfo&)
{
  // Compute the output ...
  real_type input = mInputPort.getRealValue();
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
    real_type input = mInputPort.getRealValue();
    if (mState.size() == 1) {
      /// On dimensional exponetial integrator ...
      real_type z = -dt*dot(mDenNorm, mState);
      // Well, pade approximation is the right thing, but for now ...
      if (fabs(exp(z) - 1) <= sqrt(Limits<real_type>::epsilon()))
        mState(0) += dt*(input + z);
      else
        mState(0) += dt*(exp(z)-1)/z * (input + z);
    } else {
      Vector tmpState(mState);
      mState(0) += dt*(input - dot(mDenNorm, tmpState));
      for (unsigned i = 1; i < rows(mState); ++i)
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
