/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixSplit_H
#define OpenFDM_MatrixSplit_H

#include <string>

#include "Types.h"
#include "AbstractModel.h"
#include "ModelContext.h"

namespace OpenFDM {

class MatrixSplit : public AbstractModel {
  OPENFDM_OBJECT(MatrixSplit, AbstractModel);
private:
  class Context;
public:
  MatrixSplit(const std::string& name);
  virtual ~MatrixSplit(void);
  
  virtual Context* newModelContext(PortValueList& portValueList) const;

  unsigned getSplitDimension() const
  { return mSplitDimension; }
  void setSplitDimension(unsigned concatDimension)
  { mSplitDimension = concatDimension; }

  const OutputPort* addOutputPort(const std::string& name, unsigned dimension = 1);
  
private:
  typedef std::vector<SharedPtr<NumericPortValue> > OutputValueVector;

  class Context : public AbstractModelContext {
  public:
    Context(const MatrixSplit* matrixSplit,
            const OutputValueVector& outputValues,
            NumericPortValue* inputValue);
    virtual ~Context();
  
    virtual const MatrixSplit& getNode() const;
    virtual const PortValue* getPortValue(const Port& port) const;
  
    virtual ContinousStateValue* getStateValue(const ContinousStateInfo&);
    virtual ContinousStateValue* getStateDerivative(const ContinousStateInfo&);
  
    virtual void init(const /*Init*/Task&);
    virtual void output(const Task& task);
    virtual void update(const DiscreteTask&);
    virtual void derivative(const Task&);
  private:
    SharedPtr<const MatrixSplit> mMatrixSplit;
    OutputValueVector mOutputValues;
    SharedPtr<const NumericPortValue> mInputValue;
  };

  unsigned getNonSplitDimension() const
  { return (mSplitDimension == 0) ? 1 : 0; }

  SharedPtr<InputPort> mInputPort;

  typedef std::vector<SharedPtr<OutputPort> > OutputPortVector;
  OutputPortVector mOutputPorts;

  unsigned mSplitDimension;
};

} // namespace OpenFDM

#endif
