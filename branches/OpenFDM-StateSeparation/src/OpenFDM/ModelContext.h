/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelContext_H
#define OpenFDM_ModelContext_H

#include <list>
#include "SharedPtr.h"
#include "LeafContext.h"
#include "Model.h"

namespace OpenFDM {

class Task;
class DiscreteTask;
class ContinousTask;

//// This one is used to execute the simulation system
class ModelContext : public LeafContext {
public:
  ModelContext(const Model* model);
  virtual ~ModelContext();

  virtual const Model& getNode() const;

  bool alloc()
  { return mModel->alloc(*this); }
  void init()
  { mModel->init(mDiscreteState, mContinousState, mPortValueList); }
  void output(const Task&)
  { mModel->output(mDiscreteState, mContinousState, mPortValueList); }
  void update(const DiscreteTask& discreteTask)
  { mModel->update(discreteTask, mDiscreteState, mContinousState, mPortValueList); }

//   void derivative()
//   { mModel->derivative(mDiscreteState,
//                        mContinousState,
//                        mPortValueList,
//                        mContinousStateDerivative); }

  // Return true if this model directly depends on one of models outputs
  bool dependsOn(const ModelContext& modelContext) const;

private:
  ModelContext();
  ModelContext(const ModelContext&);
  ModelContext& operator=(const ModelContext&);

  SharedPtr<const Model> mModel;
};

class ModelContextList : public std::list<SharedPtr<ModelContext> > {
public:
  typedef std::list<SharedPtr<ModelContext> > list_type;

  bool alloc() const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      if (!(*i)->alloc())
        return false;
    return true;
  }
  void init(const Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i) {
      (*i)->init();
      (*i)->output(task);
    }
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
};

} // namespace OpenFDM

#endif