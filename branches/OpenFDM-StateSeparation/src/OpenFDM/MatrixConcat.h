/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixConcat_H
#define OpenFDM_MatrixConcat_H

#include <string>

#include "Types.h"
#include "AbstractModel.h"
#include "ModelContext.h"

namespace OpenFDM {

class MatrixConcat : public AbstractModel {
  OPENFDM_OBJECT(MatrixConcat, AbstractModel);
private:
  class Context;
public:
  MatrixConcat(const std::string& name);
  virtual ~MatrixConcat(void);
  
  virtual Context* newModelContext(PortValueList& portValueList) const;

  unsigned getConcatDimension() const
  { return mConcatDimension; }
  void setConcatDimension(unsigned concatDimension)
  { mConcatDimension = concatDimension; }

  const InputPort* addInputPort(const std::string& name, unsigned dimension = 1);
  
private:
  typedef std::vector<SharedPtr<const NumericPortValue> > InputValueVector;

  class Context : public AbstractModelContext {
  public:
    Context(const MatrixConcat* matrixConcat,
            const InputValueVector& inputValues,
            NumericPortValue* outputValue);
    virtual ~Context();
  
    virtual const MatrixConcat& getNode() const;
    virtual const PortValue* getPortValue(const Port& port) const;
  
    virtual ContinousStateValue* getStateValue(const ContinousStateInfo&);
    virtual ContinousStateValue* getStateDerivative(const ContinousStateInfo&);
  
    virtual void init(const /*Init*/Task&);
    virtual void output(const Task& task);
    virtual void update(const DiscreteTask&);
    virtual void derivative(const Task&);
  private:
    SharedPtr<const MatrixConcat> mMatrixConcat;
    InputValueVector mInputValues;
    SharedPtr<NumericPortValue> mOutputValue;
  };

  unsigned getNonConcatDimension() const
  { return (mConcatDimension == 0) ? 1 : 0; }

  SharedPtr<OutputPort> mOutputPort;

  typedef std::vector<SharedPtr<InputPort> > InputPortVector;
  InputPortVector mInputPorts;

  unsigned mConcatDimension;
};

} // namespace OpenFDM

#endif
