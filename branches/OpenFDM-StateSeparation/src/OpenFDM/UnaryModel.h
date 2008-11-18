/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_UnaryModel_H
#define OpenFDM_UnaryModel_H

#include <string>

#include "AbstractModel.h"
#include "ModelContext.h"

namespace OpenFDM {

class UnaryModel : public AbstractModel {
  OPENFDM_OBJECT(UnaryModel, AbstractModel);
public:
  UnaryModel(const std::string& name);
  virtual ~UnaryModel();

protected:
  template<typename UM>
  ModelContext* newModelContext(UM* um, PortValueList& portValueList) const
  {
    Size sz = size(portValueList.getPortValue(mInputPort)->getValue());
    Log(Initialization, Debug)
      << "Size for Model \"" << getName()
      << "\" is detemined by the input port with size: "
      << trans(sz) << std::endl;
    if (!portValueList.setOrCheckPortSize(mOutputPort, sz)) {
      Log(Initialization, Warning)
        << "Size for output port from Model \"" << getName()
        << "\" does not match!" << std::endl;
      return 0;
    }

    SharedPtr<Context<UM> > context;
    context = new Context<UM>(um, portValueList.getPortValue(mInputPort),
                              portValueList.getPortValue(mOutputPort));
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
    if (!context->allocStates()) {
      Log(Model, Warning) << "Could not alloc for model \""
                          << getName() << "\"" << endl;
      return 0;
    }
    return context.release();
  }

  template<typename UM>
  class Context : public ModelContext {
  public:
    Context(const UM* unaryModel, const NumericPortValue* inputValue,
            NumericPortValue* outputValue) :
      mUnaryModel(unaryModel),
      mInputValue(inputValue),
      mOutputValue(outputValue)
    { }
    virtual ~Context()
    { }
    
    virtual const UM& getNode() const
    { return *mUnaryModel; }
    
    virtual void initOutput(const /*Init*/Task&)
    { mUnaryModel->output(mInputValue->getValue(), mOutputValue->getValue()); }
    virtual void output(const Task&)
    { mUnaryModel->output(mInputValue->getValue(), mOutputValue->getValue()); }
    virtual void update(const DiscreteTask&)
    { }
    virtual void derivative(const Task&)
    { }
    
  private:
    Context();
    Context(const Context&);
    Context& operator=(const Context&);
    
    SharedPtr<const UM> mUnaryModel;
    SharedPtr<const NumericPortValue> mInputValue;
    SharedPtr<NumericPortValue> mOutputValue;
  };

private:
  SharedPtr<InputPortInfo> mInputPort;
  SharedPtr<OutputPortInfo> mOutputPort;
};

} // namespace OpenFDM

#endif
