/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelContext_H
#define OpenFDM_ModelContext_H

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

} // namespace OpenFDM

#endif
