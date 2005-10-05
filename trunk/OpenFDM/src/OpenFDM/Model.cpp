/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "LogStream.h"
#include "ModelGroup.h"
#include "Model.h"

namespace OpenFDM {

Model::Model(const std::string& name) :
  mNumContinousStates(0l),
  mNumDiscreteStates(0l),
  mDirectFeedThrough(false),
  mName(name)
{
  addProperty("name",
              Property(this, &Model::getName, &Model::setName));
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

bool
Model::init(void)
{
  return true;
}

void
Model::output(void)
{
}

void
Model::update(real_type dt)
{
}

void
Model::setState(real_type t, const Vector& state, unsigned offset)
{
}

void
Model::getState(Vector& state, unsigned offset) const
{
}

void
Model::getStateDeriv(Vector& stateDeriv, unsigned offset)
{
}

void
Model::evalJacobian(real_type t, const Vector& v,
                   Matrix& jac, unsigned offset)
{
  unsigned nStates = getNumContinousStates();

  // Create space ...
  // FIXME
  jac.resize(nStates, nStates);

  // Get the function value at the current position.
  Vector fv(nStates);
  setState(t, v, 0);
  getStateDeriv(fv, 0);

  real_type sqrteps = 1e4*sqrt(Limits<real_type>::epsilon());

  Vector tmpv = v;
  Vector tmpfv(nStates);
  for (unsigned i = 1; i <= nStates; ++i) {
    tmpv(i) += sqrteps;

    // Evaluate then function ...
    setState(t, tmpv, 0);
    getStateDeriv(tmpfv, 0);

    // ... and compute the differencequotient to approximate the derivative.
    jac(Range(offset + 1, offset + nStates), offset + i)
      = (1/sqrteps)*(tmpfv-fv);

    // Restore the original value.
    tmpv(i) = v(i);
  }
}

void
Model::setDiscreteState(const Vector& state, unsigned offset)
{
}

void
Model::getDiscreteState(Vector& state, unsigned offset) const
{
}

const std::string&
Model::getInputPortName(unsigned i) const
{
  OpenFDMAssert(i < mInputPorts.size());
  return mInputPorts[i].name;
}

bool
Model::setInputPort(unsigned i, const Property& prop)
{
  if (mInputPorts.size() <= i) {
    Log(Model, Error) << "Input port index " << i << "out of range in \""
                      << getName() << "\"" << endl;
    return false;
  }

  mInputPorts[i].property = prop;
  inputPortChanged(i);
  return true;
}

bool
Model::setInputPort(const std::string& name, const Property& prop)
{
  for (unsigned i = 0; i < mInputPorts.size(); ++i) {
    if (mInputPorts[i].name == name) {
      setInputPort(i, prop);
      return true;
    }
  }

  Log(Model, Error) << "Input port name " << name << "not found in \""
                    << getName() << "\"" << endl;
  return false;
}

const Property&
Model::getInputPort(const std::string& name) const
{
  // Check if this one exists and return its value.
  std::vector<Port>::const_iterator it = mInputPorts.begin();
  while (it != mInputPorts.end()) {
    if ((*it).name == name)
      return (*it).property;
    ++it;
  }
  // FIXME
  return mInputPorts.front().property;
}

Property&
Model::getInputPort(const std::string& name)
{
  // Check if this one exists and return its value.
  std::vector<Port>::iterator it = mInputPorts.begin();
  while (it != mInputPorts.end()) {
    if ((*it).name == name)
      return (*it).property;
    ++it;
  }
  // FIXME
  return mInputPorts.front().property;
}

Property
Model::getOutputPort(unsigned i) const
{
  if (mOutputPorts.size() <= i) {
    Log(Model, Error) << "Output port index " << i << "out of range in \""
                      << getName() << "\"" << endl;
    return Property();
  }

  return mOutputPorts[i].property;
}

const std::string&
Model::getOutputPortName(unsigned i) const
{
  OpenFDMAssert(i < mOutputPorts.size());
  return mOutputPorts[i].name;
}

Property
Model::getOutputPort(const std::string& name) const
{
  // Check if this one exists and return its value.
  std::vector<Port>::const_iterator it = mOutputPorts.begin();
  while (it != mOutputPorts.end()) {
    if ((*it).name == name)
      return (*it).property;
    ++it;
  }

  Log(Model, Error) << "Output port name " << name << "not found in \""
                    << getName() << "\"" << endl;
  return Property();
}

bool
Model::dependsDirectOn(const Model* const model) const
{
  // If we have no direct feedthrought flag, we cannot depend on any input
  // port directly.
  if (!getDirectFeedThrough())
    return false;
  // Check if the given model is the source of any input property.
  std::vector<Port>::const_iterator it = mInputPorts.begin();
  while (it != mInputPorts.end()) {
    if (model == (*it).property.getObject())
      return true;
    ++it;
  }
  return false;
}

void
Model::setNumInputPorts(unsigned num)
{
  mInputPorts.resize(num);
  inputPortChanged(num);
}

void
Model::setInputPortName(unsigned i, const std::string& name)
{
  OpenFDMAssert(i < mInputPorts.size());
  mInputPorts[i].name = name;
  inputPortChanged(i);
}

void
Model::inputPortChanged(unsigned)
{
}

void
Model::setNumOutputPorts(unsigned num)
{
  mOutputPorts.resize(num);
}

void
Model::setOutputPort(unsigned i, const std::string& name, const Property& prop)
{
  OpenFDMAssert(i < mOutputPorts.size());
  mOutputPorts[i].property = prop;
  mOutputPorts[i].name = name;
}

void
Model::setParent(ModelGroup* modelGroup)
{
  if (mParentModel) {
    mParentModel->adjustNumDiscreteStates(0, getNumDiscreteStates());
    mParentModel->adjustNumContinousStates(0, getNumContinousStates());
  }
  mParentModel = modelGroup;
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
