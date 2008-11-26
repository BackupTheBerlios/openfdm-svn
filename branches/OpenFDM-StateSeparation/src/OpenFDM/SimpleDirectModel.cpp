/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "SimpleDirectModel.h"

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

ModelContext*
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
        if (sz(j) != sz2(j)) {
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
  
  SharedPtr<Context> context;
  context = new Context(this, inputValues, outputPortValue);
  /// FIXME, this is for legacy lport value stuff ...
  for (unsigned i = 0; i < getNumPorts(); ++i) {
    PortValue* portValue = portValueList.getPortValue(i);
    if (!portValue) {
      Log(Model, Error) << "No port value given for model \"" << getName()
                        << "\" and port \"" << getPort(i)->getName()
                        << "\"" << endl;
      return 0;
    }
    context->setPortValue(*getPort(i), portValue);
  }
  
  /// FIXME, should not need to do that either ...
  if (!context->allocStates()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return 0;
  }
  
  return context.release();
}

} // namespace OpenFDM
