/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "MatrixConcat.h"

#include "TypeInfo.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(MatrixConcat, AbstractModel)
  END_OPENFDM_OBJECT_DEF

MatrixConcat::Context::Context(const MatrixConcat* matrixConcat,
                               const InputValueVector& inputValues,
                               NumericPortValue* outputValue) :
  mMatrixConcat(matrixConcat),
  mInputValues(inputValues),
  mOutputValue(outputValue)
{
}

MatrixConcat::Context::~Context()
{
}
  
const MatrixConcat&
MatrixConcat::Context::getNode() const
{
  return *mMatrixConcat;
}

const PortValue*
MatrixConcat::Context::getPortValue(const Port& port) const
{
  if (mMatrixConcat->mOutputPort == &port)
    return mOutputValue;
  OpenFDMAssert(mInputValues.size() == mMatrixConcat->mInputPorts.size());
  for (unsigned i = 0; i < mInputValues.size(); ++i)
    if (mMatrixConcat->mInputPorts[i] == &port)
      return mInputValues[i];
  return 0;
}

ContinousStateValue*
MatrixConcat::Context::getStateValue(const ContinousStateInfo&)
{
  return 0;
}

ContinousStateValue*
MatrixConcat::Context::getStateDerivative(const ContinousStateInfo&)
{
  return 0;
}
  
void
MatrixConcat::Context::init(const /*Init*/Task&)
{
}

void
MatrixConcat::Context::output(const Task& task)
{
  unsigned concatDimension = mMatrixConcat->getConcatDimension();
  
  // May be have two contexts for each if clause??
  if (concatDimension == 0) {
    unsigned currentPos = 0;
    for (unsigned i = 0; i < mInputValues.size(); ++i) {
      const Matrix& m = mInputValues[i]->getValue();
      Size sz = size(m);
      mOutputValue->getValue()(Range(currentPos, currentPos + sz(0) - 1),
                               Range(0, sz(1) - 1)) = m;
      currentPos += sz(0);
    }
  } else {
    unsigned currentPos = 0;
    for (unsigned i = 0; i < mInputValues.size(); ++i) {
      const Matrix& m = mInputValues[i]->getValue();
      Size sz = size(m);
      mOutputValue->getValue()(Range(0, sz(0) - 1),
                               Range(currentPos, currentPos + sz(1) - 1)) = m;
      currentPos += sz(1);
    }
  }
}

void
MatrixConcat::Context::update(const DiscreteTask&)
{
}

void
MatrixConcat::Context::derivative(const Task&)
{
}

MatrixConcat::MatrixConcat(const std::string& name) :
  AbstractModel(name),
  mOutputPort(new OutputPort(this, "output", Size(0, 0), false)),
  mConcatDimension(0)
{
}

MatrixConcat::~MatrixConcat(void)
{
}
  
MatrixConcat::Context*
MatrixConcat::newModelContext(PortValueList& portValueList) const
{
  if (mInputPorts.empty()) {
    Log(Initialization, Info)
      << "No input ports in models with multiple inputs!" << std::endl;
    return 0;
  }
  
  NumericPortValue* outputPortValue = portValueList.getPortValue(mOutputPort);
  if (!outputPortValue) {
    Log(Initialization, Warning)
      << "Output port value not connected for model \"" << getName()
      << "\"!" << std::endl;
    return 0;
  }
  InputValueVector inputValues;
  
  // Look into all ports and get the best size match.
  Size sz = size(outputPortValue->getValue());
  unsigned concatSum = 0;
  for (unsigned i = 0; i < mInputPorts.size(); ++i) {
    NumericPortValue* npv = portValueList.getPortValue(mInputPorts[i]);
    inputValues.push_back(npv);
    Size sz2 = size(npv->getValue());
    // If the ports size is undefined, use the size variabe here.
    unsigned j = getNonConcatDimension();
    if (sz(j) == 0) {
      sz(j) = sz2(j);
    } else {
      if (sz2(j) != 0 && sz(j) != sz2(j)) {
        Log(Initialization, Info)
          << "Port size " << sz2 << "of port " << j
          << " does not match for model \"" << getName() << "\"!" << std::endl;
        return 0;
      }
    }
    concatSum += sz2(getConcatDimension());
  }
  if (sz(getConcatDimension()) == 0) {
    sz(getConcatDimension()) = concatSum;
  } else if (sz(getConcatDimension()) != concatSum) {
    Log(Initialization, Info)
      << "Port size sum does not match for model \""
      << getName() << "\"!" << std::endl;
    return 0;
  }
  
  // Ok, success in checking output and input ports, set them all.
  if (!portValueList.setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Info) << "Port sizes do not match!" << std::endl;
    return 0;
  }
  for (unsigned i = 0; i < mInputPorts.size(); ++i) {
    Size sz2 = mInputPorts[i]->getSize();
    sz2(getNonConcatDimension()) = sz(getNonConcatDimension());
    if (!portValueList.setOrCheckPortSize(mInputPorts[i], sz2)) {
      Log(Initialization, Info) << "Port sizes do not match!" << std::endl;
      return 0;
    }
  }
  
  return new Context(this, inputValues, outputPortValue);
}

const InputPort*
MatrixConcat::addInputPort(const std::string& name, unsigned dimension)
{
  Size size(0, 0);
  size(getConcatDimension()) = dimension;
  mInputPorts.push_back(new InputPort(this, name, size, true));
  return mInputPorts.back();
}

} // namespace OpenFDM
