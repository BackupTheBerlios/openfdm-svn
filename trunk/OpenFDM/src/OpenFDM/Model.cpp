/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Model.h"

#include "LogStream.h"
#include "ModelVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Model, Object)
  DEF_OPENFDM_ROPROP(Unsigned, NumContinousStates)
  DEF_OPENFDM_ROPROP(Unsigned, NumDiscreteStates)
  DEF_OPENFDM_ROPROP(Bool, DirectFeedThrough)
  END_OPENFDM_OBJECT_DEF

Model::Model(const std::string& name) :
  Object(name),
  mNumContinousStates(0l),
  mNumDiscreteStates(0l),
  mDirectFeedThrough(false),
  mEnabled(true),
  mNextEnabled(true),
  mEnablePort(new NumericPortAcceptor(this)),
  mDisableMode(Hold)
{
}

Model::~Model(void)
{
}

void
Model::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
}

void
Model::ascend(ModelVisitor& visitor)
{
  SharedPtr<ModelGroup> parentGroup = mParentModel.lock();
  if (!parentGroup)
    return;
  parentGroup->accept(visitor);
}

const ModelGroup*
Model::toModelGroup(void) const
{
  return 0;
}

ModelGroup*
Model::toModelGroup(void)
{
  return 0;
}

const Input*
Model::toInput(void) const
{
  return 0;
}

Input*
Model::toInput(void)
{
  return 0;
}

const Output*
Model::toOutput(void) const
{
  return 0;
}

Output*
Model::toOutput(void)
{
  return 0;
}

const Interact*
Model::toInteract(void) const
{
  return 0;
}

Interact*
Model::toInteract(void)
{
  return 0;
}

bool
Model::init(void)
{
  if (mEnablePort)
    mEnablePortHandle = mEnablePort->toRealPortHandle();
  else
    mEnablePortHandle = 0;
  return true;
}

void
Model::output(const TaskInfo&)
{
}

void
Model::update(const TaskInfo&)
{
}

void
Model::setState(const StateStream& state)
{
}

void
Model::getState(StateStream& state) const
{
}

void
Model::getStateDeriv(StateStream& stateDeriv)
{
}

void
Model::setDiscreteState(const StateStream& state)
{
}

void
Model::getDiscreteState(StateStream& state) const
{
}

bool
Model::dependsDirectOn(Model* model)
{
  if (!mDirectFeedThrough)
    return false;

  // FIXME HACK, outputs of interacts only depend on its state ...
  // FIXME is this always true??
  Interact* interact = model->toInteract();
  if (interact)
    return false;

  // return true if any output of model is connected to any input of this
  for (unsigned i = 0; i < getNumInputPorts(); ++i) {
    for (unsigned j = 0; j < model->getNumOutputPorts(); ++j) {
      if (getInputPort(i)->getPortInterface() ==
          model->getOutputPort(j)->getPortInterface())
        return true;
    }
  }

  return false;
}

const std::string&
Model::getInputPortName(unsigned i) const
{
  OpenFDMAssert(i < mInputPorts.size());
  return mInputPorts[i]->getName();
}

NumericPortAcceptor*
Model::getInputPort(const std::string& name)
{
  // Check if this one exists and return its value.
  InputPortVector::iterator it = mInputPorts.begin();
  while (it != mInputPorts.end()) {
    if ((*it)->getName() == name)
      return *it;
    ++it;
  }
  return 0;
}

NumericPortProvider*
Model::getOutputPort(unsigned i)
{
  if (mOutputPorts.size() <= i) {
    Log(Model, Error) << "Output port index " << i << "out of range in \""
                      << getName() << "\"" << endl;
    return 0;
  }

  return mOutputPorts[i];
}

NumericPortProvider*
Model::getOutputPort(const std::string& name)
{
  // Check if this one exists and return its value.
  OutputPortVector::iterator it = mOutputPorts.begin();
  while (it != mOutputPorts.end()) {
    if ((*it)->getName() == name)
      return *it;
    ++it;
  }

  Log(Model, Error) << "Output port name \"" << name << "\" not found in \""
                    << getName() << "\"" << endl;
  return 0;
}

const std::string&
Model::getOutputPortName(unsigned i) const
{
  OpenFDMAssert(i < mOutputPorts.size());
  return mOutputPorts[i]->getName();
}

class ModelPathStringCollector :
    public ModelVisitor {
public:
  virtual void apply(Model& model)
  {
    // First go up and collect the path above.
    // When we are back here append this.
    ascend(model);
    path += "/";
    path += model.getName();
  }
  std::string path;
};

std::string
Model::getPathString(void)
{
  ModelPathStringCollector modelPathStringCollector;
  accept(modelPathStringCollector);
  return modelPathStringCollector.path;
}

class ModelPathCollector :
    public ModelVisitor {
public:
  virtual void apply(Model& model)
  { ascend(model); }
  virtual void apply(ModelGroup& modelGroup)
  {
    // First go up and collect the path above.
    // When we are back here append this.
    ascend(modelGroup);
    path.push_back(&modelGroup);
  }
  Model::Path path;
};

Model::Path
Model::getPath()
{
  ModelPathCollector modelPathCollector;
  ascend(modelPathCollector);
  return modelPathCollector.path;
}

const ModelGroup*
Model::getParent(void) const
{
  return mParentModel;
}

ModelGroup*
Model::getParent(void)
{
  return mParentModel;
}

void
Model::setNumInputPorts(unsigned num)
{
  // Ok, strange, but required ...
  unsigned oldSize = mInputPorts.size();
  mInputPorts.resize(num);
  for (; oldSize < mInputPorts.size(); ++oldSize) {
    mInputPorts[oldSize] = new NumericPortAcceptor(this);
  }
}

void
Model::setInputPortName(unsigned i, const std::string& name)
{
  OpenFDMAssert(i < mInputPorts.size());
  mInputPorts[i]->setName(name);
}

void
Model::setNumOutputPorts(unsigned num)
{
  // Ok, strange, but required ...
  unsigned oldSize = mOutputPorts.size();
  mOutputPorts.resize(num);
  for (; oldSize < mOutputPorts.size(); ++oldSize) {
    mOutputPorts[oldSize] = new NumericPortProvider(this);
  }
}

void
Model::setOutputPort(unsigned i, const std::string& name,
                     PortInterface* portInterface)
{
  OpenFDMAssert(i < mOutputPorts.size());

  NumericPortProvider* portProvider = new NumericPortProvider(this);
  portProvider->setPortInterface(portInterface);
  portProvider->setName(name);
  mOutputPorts[i] = portProvider;
}

void
Model::setParent(ModelGroup* model)
{
  if (mParentModel) {
    mParentModel->adjustNumDiscreteStates(0, getNumDiscreteStates());
    mParentModel->adjustNumContinousStates(0, getNumContinousStates());
  }
  mParentModel = model;
  if (mParentModel) {
    mParentModel->adjustNumDiscreteStates(getNumDiscreteStates(), 0);
    mParentModel->adjustNumContinousStates(getNumContinousStates(), 0);
  } else {
    unsigned num = getNumInputPorts();
    for (unsigned i = 0; i < num; ++i)
      mInputPorts[i]->removeAllConnections();
    num = getNumOutputPorts();
    for (unsigned i = 0; i < num; ++i)
      mOutputPorts[i]->removeAllConnections();
  }
}

void
Model::setNumContinousStates(unsigned numContinousStates)
{
  if (mParentModel)
    mParentModel->adjustNumContinousStates(numContinousStates,
                                           mNumContinousStates);
  mNumContinousStates = numContinousStates;
}

void
Model::setNumDiscreteStates(unsigned numDiscreteStates)
{
  if (mParentModel)
    mParentModel->adjustNumDiscreteStates(numDiscreteStates,
                                          mNumDiscreteStates);
  mNumDiscreteStates = numDiscreteStates;
}

void
Model::setEnvironment(Environment* environment)
{
}

void
Model::setEnabledUnconditional(bool enabled)
{
  if (mEnabled) {
    switch (mDisableMode) {
    case ResetHold:
      /// If disabled, the models output/state is initialized
      init();
      break;
    default:
      break;
    }
  } else {
    switch (mDisableMode) {
    case HoldReset:
      /// If disabled, the models output/state is just held. On reenable, the
      /// the model is initialized
      /// If disabled, the models output/state is initialized
      init();
      break;
    default:
      break;
    }
  }

  mEnabled = enabled;
}

void
Model::adjustNumContinousStates(unsigned newCount, unsigned oldCount)
{
  unsigned numContinousStates = getNumContinousStates();
  OpenFDMAssert(oldCount <= numContinousStates);
  numContinousStates -= oldCount;
  numContinousStates += newCount;
  setNumContinousStates(numContinousStates);
}

void
Model::adjustNumDiscreteStates(unsigned newCount, unsigned oldCount)
{
  unsigned numDiscreteStates = getNumDiscreteStates();
  OpenFDMAssert(oldCount <= numDiscreteStates);
  numDiscreteStates -= oldCount;
  numDiscreteStates += newCount;
  setNumDiscreteStates(numDiscreteStates);
}

} // namespace OpenFDM
