/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Model.h"

#include "ConstNodeVisitor.h"
#include "LogStream.h"
#include "ModelContext.h"
#include "NodeVisitor.h"

namespace OpenFDM {

class Model::Context : public ModelContext {
public:
  Context(const Model* model) :
    mModel(model)
  {
  }
  virtual ~Context()
  { }

  virtual const Model& getNode() const
  { return *mModel; }

  bool alloc()
  {
    if (!allocStates())
      return false;
    return mModel->alloc(*this);
  }
  virtual void init(const /*Init*/Task& task)
  {
    mModel->init(task, mDiscreteState, mContinousState, mPortValueList);
  }
  virtual void output(const Task& task)
  {
    mModel->output(task, mDiscreteState, mContinousState, mPortValueList);
  }
  virtual void update(const DiscreteTask& discreteTask)
  {
    mModel->update(discreteTask, mDiscreteState, mContinousState, mPortValueList);
  }

  virtual void derivative(const Task&)
  {
    mModel->derivative(mDiscreteState, mContinousState, mPortValueList,
                       mContinousStateDerivative);
  }

private:
  Context();
  Context(const Context&);
  Context& operator=(const Context&);

  SharedPtr<const Model> mModel;
};

BEGIN_OPENFDM_OBJECT_DEF(Model, AbstractModel)
  END_OPENFDM_OBJECT_DEF

Model::Model(const std::string& name) :
  AbstractModel(name)
{
}

Model::~Model()
{
}

ModelContext*
Model::newModelContext(PortValueList& portValueList) const
{
  SharedPtr<Context> context = new Context(this);
  for (unsigned i = 0; i < getNumPorts(); ++i) {
    PortValue* portValue = portValueList.getPortValue(i);
    if (!portValue) {
      Log(Model, Error) << "No port value given for model \"" << getName()
                        << "\" and port \"" << getPort(i)->getName()
                        << "\"" << endl;
      return false;
    }
    context->setPortValue(*getPort(i), portValue);
  }
  if (!context->alloc()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return false;
  }
  return context.release();
}

} // namespace OpenFDM
