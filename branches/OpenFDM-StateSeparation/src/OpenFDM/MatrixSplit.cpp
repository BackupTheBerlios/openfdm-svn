/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "MatrixSplit.h"

#include "TypeInfo.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(MatrixSplit, AbstractModel)
  DEF_OPENFDM_PROPERTY(Unsigned, SplitDimension, Serialized)
  END_OPENFDM_OBJECT_DEF

MatrixSplit::Context::Context(const MatrixSplit* matrixSplit,
                              const OutputValueVector& outputValues,
                              NumericPortValue* inputValue) :
  mMatrixSplit(matrixSplit),
  mOutputValues(outputValues),
  mInputValue(inputValue)
{
}

MatrixSplit::Context::~Context()
{
}
  
const MatrixSplit&
MatrixSplit::Context::getNode() const
{
  return *mMatrixSplit;
}

const PortValue*
MatrixSplit::Context::getPortValue(const Port& port) const
{
  if (mMatrixSplit->mInputPort == &port)
    return mInputValue;
  OpenFDMAssert(mOutputValues.size() == mMatrixSplit->mOutputPorts.size());
  for (unsigned i = 0; i < mOutputValues.size(); ++i)
    if (mMatrixSplit->mOutputPorts[i] == &port)
      return mOutputValues[i];
  return 0;
}

ContinousStateValue*
MatrixSplit::Context::getStateValue(const ContinousStateInfo&)
{
  return 0;
}

ContinousStateValue*
MatrixSplit::Context::getStateDerivative(const ContinousStateInfo&)
{
  return 0;
}
  
void
MatrixSplit::Context::init(const /*Init*/Task&)
{
}

void
MatrixSplit::Context::output(const Task& task)
{
  unsigned concatDimension = mMatrixSplit->getSplitDimension();
  
  // May be have two contexts for each if clause??
  if (concatDimension == 0) {
    unsigned currentPos = 0;
    for (unsigned i = 0; i < mOutputValues.size(); ++i) {
      Matrix& m = mOutputValues[i]->getValue();
      Size sz = size(m);
      m = mInputValue->getValue()(Range(currentPos, currentPos + sz(0) - 1),
                                  Range(0, sz(1) - 1));
      currentPos += sz(0);
    }
  } else {
    unsigned currentPos = 0;
    for (unsigned i = 0; i < mOutputValues.size(); ++i) {
      Matrix& m = mOutputValues[i]->getValue();
      Size sz = size(m);
      m = mInputValue->getValue()(Range(0, sz(0) - 1),
                                  Range(currentPos, currentPos + sz(1) - 1));
      currentPos += sz(1);
    }
  }
}

void
MatrixSplit::Context::update(const DiscreteTask&)
{
}

void
MatrixSplit::Context::derivative(const Task&)
{
}

MatrixSplit::MatrixSplit(const std::string& name) :
  AbstractModel(name),
  mInputPort(new InputPort(this, "input", Size(0, 0), true)),
  mSplitDimension(0)
{
}

MatrixSplit::~MatrixSplit(void)
{
}
  
MatrixSplit::Context*
MatrixSplit::newModelContext(PortValueList& portValueList) const
{
  if (mOutputPorts.empty()) {
    Log(Initialization, Info)
      << "No input ports in models with multiple inputs!" << std::endl;
    return 0;
  }
  
  NumericPortValue* outputPortValue = portValueList.getPortValue(mInputPort);
  if (!outputPortValue) {
    Log(Initialization, Warning)
      << "Input port value not connected for model \"" << getName()
      << "\"!" << std::endl;
    return 0;
  }
  OutputValueVector inputValues;
  
  // Look into all ports and get the best size match.
  Size sz = size(outputPortValue->getValue());
  unsigned concatSum = 0;
  for (unsigned i = 0; i < mOutputPorts.size(); ++i) {
    NumericPortValue* npv = portValueList.getPortValue(mOutputPorts[i]);
    inputValues.push_back(npv);
    Size sz2 = size(npv->getValue());
    // If the ports size is undefined, use the size variabe here.
    unsigned j = getNonSplitDimension();
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
    concatSum += sz2(getSplitDimension());
  }
  if (sz(getSplitDimension()) == 0) {
    sz(getSplitDimension()) = concatSum;
  } else if (sz(getSplitDimension()) != concatSum) {
    Log(Initialization, Info)
      << "Port size sum does not match for model \""
      << getName() << "\"!" << std::endl;
    return 0;
  }
  
  // Ok, success in checking output and input ports, set them all.
  if (!portValueList.setOrCheckPortSize(mInputPort, sz)) {
    Log(Initialization, Info) << "Port sizes do not match!" << std::endl;
    return 0;
  }
  for (unsigned i = 0; i < mOutputPorts.size(); ++i) {
    Size sz2 = mOutputPorts[i]->getSize();
    sz2(getNonSplitDimension()) = sz(getNonSplitDimension());
    if (!portValueList.setOrCheckPortSize(mOutputPorts[i], sz2)) {
      Log(Initialization, Info) << "Port sizes do not match!" << std::endl;
      return 0;
    }
  }
  
  return new Context(this, inputValues, outputPortValue);
}

const OutputPort*
MatrixSplit::addOutputPort(const std::string& name, unsigned dimension)
{
  Size size(0, 0);
  size(getSplitDimension()) = dimension;
  mOutputPorts.push_back(new OutputPort(this, name, size, true));
  return mOutputPorts.back();
}

} // namespace OpenFDM
