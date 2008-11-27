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

  unsigned getNumInputPorts() const
  { return mInputPorts.size(); }

  class Context;
  virtual ModelContext* newModelContext(PortValueList& portValueList) const;
  virtual void output(Context& context) const = 0;

  typedef std::vector<SharedPtr<const NumericPortValue> > InputValueVector;
  class Context : public ModelContext {
  public:
    Context(const SimpleDirectModel* model, const InputValueVector& inputValues,
            NumericPortValue* outputValue);
    virtual ~Context();
    
    virtual const SimpleDirectModel& getNode() const;
    virtual const PortValue* getPortValue(const PortInfo&) const;
    virtual void setPortValue(const PortInfo&, PortValue*);
    
    virtual void initOutput(const /*Init*/Task&);
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

// protected:
  PortId addInputPort(const std::string& name)
  {
    mInputPorts.push_back(new InputPortInfo(this, name, Size(0, 0), true));
    return PortId(mInputPorts.back());
  }
  void removeInputPort(const PortId& portId)
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

private:
  typedef std::vector<SharedPtr<InputPortInfo> > InputPortVector;
  InputPortVector mInputPorts;
  SharedPtr<OutputPortInfo> mOutputPort;
};

} // namespace OpenFDM

#endif
