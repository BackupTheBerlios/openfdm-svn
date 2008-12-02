/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "SimpleDirectModel.h"

#include <sstream>
#include "Matrix.h"
#include "AbstractModel.h"
#include "ModelContext.h"

namespace OpenFDM {

SimpleDirectModel::Context::Context(const SimpleDirectModel* model,
                                    const InputValueVector& inputValues,
                                    NumericPortValue* outputValue) :
  mModel(model),
  mInputValues(inputValues),
  mOutputValue(outputValue)
{
}

SimpleDirectModel::Context::~Context()
{
}
    
const SimpleDirectModel&
SimpleDirectModel::Context::getNode() const
{
  return *mModel;
}

const PortValue*
SimpleDirectModel::Context::getPortValue(const PortInfo& portInfo) const
{
  if (mModel->mOutputPort == &portInfo)
    return mOutputValue;
  OpenFDMAssert(mInputValues.size() == mModel->mInputPorts.size());
  for (unsigned i = 0; i < mInputValues.size(); ++i)
    if (mModel->mInputPorts[i] == &portInfo)
      return mInputValues[i];
  return 0;
}

ContinousStateValue*
SimpleDirectModel::Context::getStateValue(const ContinousStateInfo&)
{
  return 0;
}

ContinousStateValue*
SimpleDirectModel::Context::getStateDerivative(const ContinousStateInfo&)
{
  return 0;
}

void
SimpleDirectModel::Context::initOutput(const /*Init*/Task&)
{
  mModel->output(*this);
}

void
SimpleDirectModel::Context::output(const Task&)
{
  mModel->output(*this);
}

void
SimpleDirectModel::Context::update(const DiscreteTask&)
{
}

void
SimpleDirectModel::Context::derivative(const Task&)
{
}

BEGIN_OPENFDM_OBJECT_DEF(SimpleDirectModel, AbstractModel)
  END_OPENFDM_OBJECT_DEF

SimpleDirectModel::SimpleDirectModel(const std::string& name) :
  AbstractModel(name),
  mOutputPort(new OutputPortInfo(this, "output", Size(0, 0), false))
{
}

SimpleDirectModel::~SimpleDirectModel()
{
}

AbstractModelContext*
SimpleDirectModel::newModelContext(PortValueList& portValueList) const
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
  
  /// FIXME, hmm, this can be made generically on the port infos directly??
  // May be some 'Simple pointwise model' type that just allows equal
  // numeric ports in size and type??
  
  // Look into all ports and get the best size match.
  Size sz = size(outputPortValue->getValue());
  for (unsigned i = 0; i < mInputPorts.size(); ++i) {
    NumericPortValue* npv = portValueList.getPortValue(mInputPorts[i]);
    inputValues.push_back(npv);
    Size sz2 = size(npv->getValue());
    // If the ports size is undefined, use the size variabe here.
    for (unsigned j = 0; j < 2; ++j) {
      if (sz(j) == 0) {
        sz(j) = sz2(j);
      } else {
        if (sz2(j) != 0 && sz(j) != sz2(j)) {
          Log(Initialization, Info)
            << "Port size " << j << " does not match for model \""
            << getName() << "\"!" << std::endl;
          return 0;
        }
      }
    }
  }
  
  // Ok, success in checking output and input ports, set them all.
  if (!portValueList.setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Info) << "Port sizes do not match!" << std::endl;
    return 0;
  }
  for (unsigned i = 0; i < mInputPorts.size(); ++i) {
    if (!portValueList.setOrCheckPortSize(mInputPorts[i], sz)) {
      Log(Initialization, Info) << "Port sizes do not match!" << std::endl;
      return 0;
    }
  }
  
  return new Context(this, inputValues, outputPortValue);
}

void
SimpleDirectModel::setNumInputPorts(unsigned numInputPorts)
{
  unsigned oldnum = getNumInputPorts();
  for (; oldnum < numInputPorts; ++oldnum) {
    std::stringstream s;
    s << "input" << oldnum;
    addInputPort(s.str());
  }
  for (; numInputPorts < oldnum; --oldnum)
    removeInputPort(getInputPort(oldnum-1));
}

PortId
SimpleDirectModel::addInputPort(const std::string& name)
{
  mInputPorts.push_back(new InputPortInfo(this, name, Size(0, 0), true));
  return PortId(mInputPorts.back());
}

void
SimpleDirectModel::removeInputPort(const PortId& portId)
{
  InputPortVector::iterator i = mInputPorts.begin();
  while (i != mInputPorts.end()) {
    if (portId != PortId(*i)) {
      ++i;
      continue;
    }
    (*i)->clear();
    i = mInputPorts.erase(i);
  }
}

} // namespace OpenFDM
