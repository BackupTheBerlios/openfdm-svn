/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SimpleDirectModel_H
#define OpenFDM_SimpleDirectModel_H

#include <string>

#include "Matrix.h"
#include "AbstractModel.h"
#include "ModelContext.h"

namespace OpenFDM {

class SimpleDirectModel : public AbstractModel {
  OPENFDM_OBJECT(SimpleDirectModel, AbstractModel);
public:
  SimpleDirectModel(const std::string& name);
  virtual ~SimpleDirectModel();

  class Context;
  virtual AbstractModelContext* newModelContext(PortValueList& portValueList) const;
  virtual void output(Context& context) const = 0;

  typedef std::vector<SharedPtr<const NumericPortValue> > InputValueVector;
  class Context : public AbstractModelContext {
  public:
    Context(const SimpleDirectModel* model, const InputValueVector& inputValues,
            NumericPortValue* outputValue);
    virtual ~Context();
    
    virtual const SimpleDirectModel& getNode() const;
    virtual const PortValue* getPortValue(const PortInfo&) const;

    virtual ContinousStateValue* getStateValue(const ContinousStateInfo&);
    virtual ContinousStateValue* getStateDerivative(const ContinousStateInfo&);
    
    virtual void init(const /*Init*/Task&);
    virtual void output(const Task&);
    virtual void update(const DiscreteTask&);
    virtual void derivative(const Task&);
    
    const Matrix& getInputValue(unsigned i)
    {
      OpenFDMAssert(i < mInputValues.size());
      return mInputValues[i]->getValue();
    }
    Matrix& getOutputValue()
    { return mOutputValue->getValue(); }

  private:
    SharedPtr<const SimpleDirectModel> mModel;
    InputValueVector mInputValues;
    SharedPtr<NumericPortValue> mOutputValue;
  };

  unsigned getNumInputPorts() const;
  const InputPortInfo* getInputPort(unsigned i) const;
  const OutputPortInfo* getOutputPort() const;

protected:
  void setNumInputPorts(unsigned numInputPorts);
  const InputPortInfo* addInputPort(const std::string& name);
  void removeInputPort(const InputPortInfo* portInfo);

private:
  typedef std::vector<SharedPtr<InputPortInfo> > InputPortVector;
  InputPortVector mInputPorts;
  SharedPtr<OutputPortInfo> mOutputPort;
};

} // namespace OpenFDM

#endif
