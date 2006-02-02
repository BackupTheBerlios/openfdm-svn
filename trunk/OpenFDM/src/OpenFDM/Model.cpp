/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "LogStream.h"
#include "ModelVisitor.h"
#include "Model.h"

namespace OpenFDM {

const SampleTime SampleTime::PerTimestep(-2);
const SampleTime SampleTime::Inherited(-1);
const SampleTime SampleTime::Continous(0);

BEGIN_OPENFDM_OBJECT_DEF(Model)
  END_OPENFDM_OBJECT_DEF

Model::Model(const std::string& name) :
  Object(name),
  mNumContinousStates(0l),
  mNumDiscreteStates(0l),
  mDirectFeedThrough(false),
  mMultiBodyAcceleration(false)
{
  addProperty("numContinousStates",
              Property(this, &Model::getNumContinousStates));
  addProperty("numDiscreteStates",
              Property(this, &Model::getNumDiscreteStates));
  addProperty("directFeedThrough",
              Property(this, &Model::getDirectFeedThrough));
}

Model::~Model(void)
{
}

void
Model::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
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

const Joint*
Model::toJoint(void) const
{
  return 0;
}

Joint*
Model::toJoint(void)
{
  return 0;
}

const MobileRootJoint*
Model::toMobileRootJoint(void) const
{
  return 0;
}

MobileRootJoint*
Model::toMobileRootJoint(void)
{
  return 0;
}

bool
Model::init(void)
{
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

void
Model::evalFunction(real_type t, const Vector& v, Vector& out)
{
  /// FIXME Hmm, may be different ...
  StateStream stateStream(v);
  setState(v);

  TaskInfo taskInfo;
  taskInfo.setTime(t);
  taskInfo.addSampleTime(SampleTime::Continous);
  output(taskInfo);

  stateStream.reset();
  getStateDeriv(stateStream);
  out = stateStream.getState();
}

void
Model::evalJacobian(real_type t, const Vector& v, Matrix& jac)
{
  unsigned nStates = getNumContinousStates();

  // Create space ...
  // FIXME
  jac.resize(nStates, nStates);

  // Get the function value at the current position.
  Vector fv(nStates);
  evalFunction(t, v, fv);

  real_type sqrteps = 1e4*sqrt(Limits<real_type>::epsilon());

  Vector tmpv = v;
  Vector tmpfv(nStates);
  for (unsigned i = 1; i <= nStates; ++i) {
    tmpv(i) += sqrteps;

    // Evaluate then function ...
    evalFunction(t, tmpv, tmpfv);

    // ... and compute the differencequotient to approximate the derivative.
    jac(Range(1, nStates), i) = (1/sqrteps)*(tmpfv-fv);

    // Restore the original value.
    tmpv(i) = v(i);
  }
}

const std::string&
Model::getInputPortName(unsigned i) const
{
  OpenFDMAssert(i < mInputPorts.size());
  return mInputPorts[i]->getName();
}

Port*
Model::getInputPort(const std::string& name)
{
  // Check if this one exists and return its value.
  std::vector<SharedPtr<Port> >::iterator it = mInputPorts.begin();
  while (it != mInputPorts.end()) {
    if ((*it)->getName() == name)
      return *it;
    ++it;
  }
  return 0;
}

Port*
Model::getOutputPort(unsigned i)
{
  if (mOutputPorts.size() <= i) {
    Log(Model, Error) << "Output port index " << i << "out of range in \""
                      << getName() << "\"" << endl;
    return 0;
  }

  return mOutputPorts[i];
}

Port*
Model::getOutputPort(const std::string& name)
{
  // Check if this one exists and return its value.
  std::vector<SharedPtr<Port> >::iterator it = mOutputPorts.begin();
  while (it != mOutputPorts.end()) {
    if ((*it)->getName() == name)
      return *it;
    ++it;
  }

  Log(Model, Error) << "Output port name " << name << "not found in \""
                    << getName() << "\"" << endl;
  return 0;
}

const std::string&
Model::getOutputPortName(unsigned i) const
{
  OpenFDMAssert(i < mOutputPorts.size());
  return mOutputPorts[i]->getName();
}

class ModelPathCollector :
    public ModelVisitor {
public:
  virtual ~ModelPathCollector(void)
  { }
  virtual void apply(Model& model)
  { ascend(model); path += "/"; path += model.getName(); }
  std::string path;
};

std::string
Model::getPathString(void)
{
  ModelPathCollector modelPathCollector;
  accept(modelPathCollector);
  return modelPathCollector.path;
}

void
Model::setNumInputPorts(unsigned num)
{
  // Ok, strange, but required ...
  unsigned oldSize = mInputPorts.size();
  mInputPorts.resize(num);
  for (; oldSize < mInputPorts.size(); ++oldSize)
    mInputPorts[oldSize] = new Port;
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
  for (; oldSize < mOutputPorts.size(); ++oldSize)
    mOutputPorts[oldSize] = new Port;
}

void
Model::setOutputPort(unsigned i, const std::string& name,
                     PortInterface* portInterface)
{
  OpenFDMAssert(i < mOutputPorts.size());
  Port* port = new Port;
  port->setPortInterface(portInterface);
  port->setName(name);
  mOutputPorts[i] = port;
}

Environment*
Model::getEnvironment(void) const
{
  if (mParentModel)
    return mParentModel->getEnvironment();
  return 0;
}

void
Model::setParent(Model* model)
{
  if (mParentModel) {
    mParentModel->adjustNumDiscreteStates(0, getNumDiscreteStates());
    mParentModel->adjustNumContinousStates(0, getNumContinousStates());
  }
  mParentModel = model;
  if (mParentModel) {
    mParentModel->adjustNumDiscreteStates(getNumDiscreteStates(), 0);
    mParentModel->adjustNumContinousStates(getNumContinousStates(), 0);
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
