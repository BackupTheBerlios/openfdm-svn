/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelContext_H
#define OpenFDM_ModelContext_H

#include <list>
#include "SharedPtr.h"
#include "LeafContext.h"
#include "AbstractModel.h"

namespace OpenFDM {

class Task;
class ContinousTask;
class DiscreteTask;
class InitTask;

/// This one is used to execute the simulation system
class AbstractModelContext : public LeafContext {
public:
  AbstractModelContext() {}
  virtual ~AbstractModelContext() {}

  virtual const AbstractModel& getNode() const = 0;

  void initOutput(const /*Init*/Task& task)
  {
    init(task);
    output(task);
  }
  virtual void init(const /*Init*/Task& task) = 0;
  virtual void output(const Task& task) = 0;
  virtual void update(const DiscreteTask& discreteTask) = 0;
  virtual void derivative(const Task&) = 0;

private:
  AbstractModelContext(const AbstractModelContext&);
  AbstractModelContext& operator=(const AbstractModelContext&);
};



class ModelContext : public AbstractModelContext {
public:
  ModelContext() {}
  virtual ~ModelContext();

  /// might vanish???
  PortValueList& getPortValueList()
  { return mPortValueList; }

  virtual void setContinousStateSize(const MatrixStateInfo& stateInfo,
                                     const Size& sz)
  {
    mContinousState[stateInfo].resize(sz(0), sz(1));
    mContinousStateDerivative[stateInfo].resize(sz(0), sz(1));
  }

  virtual bool allocStates()
  {
    unsigned numContinousStates = getNode().getNumContinousStateValues();
    for (unsigned i = 0; i < numContinousStates; ++i) {
      const ContinousStateInfo* continousStateInfo;
      continousStateInfo = getNode().getContinousStateInfo(i);
      mContinousState.setValue(*continousStateInfo, *this);
      mContinousStateDerivative.setValue(*continousStateInfo, *this);
    }
    unsigned numDiscreteStates = getNode().getNumDiscreteStateValues();
    for (unsigned i = 0; i < numDiscreteStates; ++i) {
      const StateInfo* stateInfo;
      stateInfo = getNode().getDiscreteStateInfo(i);
      mDiscreteState.setValue(*stateInfo, *this);
    }
    return true;
  }

  virtual ContinousStateValue* getStateValue(const ContinousStateInfo& info)
  { return mContinousState.getValue(info); }
  virtual ContinousStateValue* getStateDerivative(const ContinousStateInfo& info)
  { return mContinousStateDerivative.getValue(info); }

  /// Set port value for the given port.
  virtual const PortValue* getPortValue(const PortInfo& portInfo) const
  {  return mPortValueList.getPortValue(portInfo); }
  virtual void setPortValue(const PortInfo& portInfo, PortValue* portValue)
  { mPortValueList.setPortValue(portInfo.getIndex(), portValue); }

  
protected:
  // PortValues
  PortValueList mPortValueList;

  // Continous States
  ContinousStateValueVector mContinousState;
  ContinousStateValueVector mContinousStateDerivative;
  // Discrete States
  DiscreteStateValueVector mDiscreteState;

private:
  ModelContext(const ModelContext&);
  ModelContext& operator=(const ModelContext&);
};

class ModelContextList : public std::list<SharedPtr<AbstractModelContext> > {
public:
  typedef std::list<SharedPtr<AbstractModelContext> > list_type;

  void initOutput(const /*Init*/Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->initOutput(task);
  }
  void output(const Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->output(task);
  }
  void update(const DiscreteTask& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->update(task);
  }
  void derivative(const Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->derivative(task);
  }
};

} // namespace OpenFDM

#endif
