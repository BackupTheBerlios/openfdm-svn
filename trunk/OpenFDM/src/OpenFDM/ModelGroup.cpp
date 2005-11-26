/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Object.h"
#include "Model.h"
#include "Property.h"
#include "Vector.h"
#include "LogStream.h"
#include "ModelGroup.h"

namespace OpenFDM {

ModelGroup::ModelGroup(const std::string& name) :
  Model(name)
{
}

ModelGroup::~ModelGroup(void)
{
  // Remove all references to this group.
  while (!mModels.empty())
    removeModel(mModels.front());
  // Make sure we can count right ...
  OpenFDMAssert(getNumDiscreteStates() == 0u);
  OpenFDMAssert(getNumContinousStates() == 0u);
}

const ModelGroup*
ModelGroup::toModelGroup(void) const
{
  return this;
}

ModelGroup*
ModelGroup::toModelGroup(void)
{
  return this;
}

const Model*
ModelGroup::getModel(unsigned idx) const
{
  if (idx < mModels.size())
    return mModels[idx];
  else
    return 0;
}

Model*
ModelGroup::getModel(unsigned idx)
{
  if (idx < mModels.size())
    return mModels[idx];
  else
    return 0;
}

const Model*
ModelGroup::getModel(const std::string& name) const
{
  return getModel(getModelIndex(name));
}

Model*
ModelGroup::getModel(const std::string& name)
{
  return getModel(getModelIndex(name));
}

unsigned
ModelGroup::getModelIndex(const std::string& name) const
{
  unsigned idx = 0u;
  ModelList::const_iterator it = mModels.begin();
  while (it != mModels.end()) {
    if ((*it)->getName() == name)
      return idx;
    ++it;
    ++idx;
  }
  return idx;
}

unsigned
ModelGroup::addModel(Model* model)
{
  // cannot add no model ...
  if (!model) {
    Log(Model, Warning)
      << "Trying to add zero OpenFDM::Model pointer "
      "to OpenFDM::ModelGroup \"" << getName() << "\"" << endl;
    return ~0u;
  }

  // cannot add a model to two groups.
  if (model->getParent()) {
    Log(Model, Warning)
      << "While adding the OpenFDM::Model \"" << model->getName()
      << "\" to OpenFDM::ModelGroup \"" << getName()
      << "\": Model is already attached!" << endl;
    return ~0u;
  }

  // Check if we already have a model with the same name
  /// FIXME: we need to make sure that duplicate names cannot occure
  /// by attaching and than setting the duplicate name!
  for (unsigned i = 0; i < mModels.size(); ++i) {
    if (model->getName() == mModels[i]->getName()) {
      Log(Model, Warning)
        << "While adding the OpenFDM::Model \"" << model->getName()
        << "\" to OpenFDM::ModelGroup \"" << getName()
        << "\": Model with the same name is already attached!" << endl;
    }
  }

  // Update the number of states.
  model->setParent(this);

  // add to the model list.
  mModels.push_back(model);

  return mModels.size()-1;
}

void
ModelGroup::removeModel(Model* model)
{
  // cannot remove no model ...
  if (!model) {
    Log(Model, Warning)
      << "Trying to remove zero OpenFDM::Model pointer "
      "from OpenFDM::ModelGroup \"" << getName() << "\"" << endl;
    return;
  }

  // cannot remove if we are not its parent.
  if (model->getParent() != this) {
    Log(Model, Warning)
      << "Trying to remove OpenFDM::Model \"" << model->getName()
      << "\" from OpenFDM::ModelGroup \"" << getName() << "\"" << endl;
    return;
  }

  ModelList::iterator it = mModels.begin();
  while (it != mModels.end()) {
    if ((*it) == model)
      break;
    ++it;
  }
  // Ooops, we have not found the child?
  if (it == mModels.end()) {
    Log(Model, Warning)
      << "Trying to remove OpenFDM::Model \"" << model->getName()
      << "\" from OpenFDM::ModelGroup \"" << getName()
      << "\": cannot find model in this group!" << endl;
    return;
  }

  // remove the backreference to this group
  // this also updates the number of states
  model->setParent(0);

  // remove from the model list.
  // note that erasing might delete the model object, thus delete it past
  // correction of the number of states.
  mModels.erase(it);
}

bool
ModelGroup::init(void)
{
  // Try to resolve direct feedthrough dependencies.
  // Bail out if not possible.
  if (!sortModels())
    return false;
  // Just init all children.
  ModelList::iterator it;
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    Model* model = *it;

    if (!model->init()) {
      Log(Model, Error) << "Initialization of \"" << model->getName()
                        << "\" failed!"<< endl;
      return false;
    }
    // Now, tell the current model group about the sample times in
    // this child model.
    SampleTimeSet sampleTimes = model->getSampleTimeSet();
    SampleTimeSet::const_iterator sit;
    for (sit = sampleTimes.begin(); sit != sampleTimes.end(); ++sit)
      addSampleTime(*sit);
  }
  return true;
}

void
ModelGroup::output(const TaskInfo& taskInfo)
{
  if (!nonZeroIntersection(taskInfo.getSampleTimeSet(), getSampleTimeSet()))
    return;

  // Just do output on all children.
  ModelList::iterator it;
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    (*it)->output(taskInfo);
  }
}

void
ModelGroup::update(const TaskInfo& taskInfo)
{
  if (!nonZeroIntersection(taskInfo.getSampleTimeSet(), getSampleTimeSet()))
    return;

  // Just update all children.
  ModelList::iterator it;
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    (*it)->update(taskInfo);
  }
}

void
ModelGroup::setState(const Vector& state, unsigned offset)
{
  OpenFDMAssert(offset + getNumContinousStates() <= rows(state));

  ModelList::iterator it;
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    unsigned nStates = (*it)->getNumContinousStates();
    if (0 < nStates) {
      OpenFDMAssert(offset + nStates <= rows(state));
      (*it)->setState(state, offset);
      offset += nStates;
    }
  }
}

void
ModelGroup::getState(Vector& state, unsigned offset) const
{
  OpenFDMAssert(offset + getNumContinousStates() <= rows(state));

  ModelList::const_iterator it;
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    unsigned nStates = (*it)->getNumContinousStates();
    if (0 < nStates) {
      OpenFDMAssert(offset + nStates <= rows(state));
      (*it)->getState(state, offset);
      offset += nStates;
    }
  }
}

void
ModelGroup::getStateDeriv(Vector& stateDeriv, unsigned offset)
{
  OpenFDMAssert(offset + getNumContinousStates() <= rows(stateDeriv));

  ModelList::iterator it;
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    unsigned nStates = (*it)->getNumContinousStates();
    if (0 < nStates) {
      OpenFDMAssert(offset + nStates <= rows(stateDeriv));
      (*it)->getStateDeriv(stateDeriv, offset);
      offset += nStates;
    }
  }
}

void
ModelGroup::setDiscreteState(const Vector& state, unsigned offset)
{
  OpenFDMAssert(offset + getNumDiscreteStates() <= rows(state));

  ModelList::iterator it;
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    unsigned nStates = (*it)->getNumDiscreteStates();
    if (0 < nStates) {
      OpenFDMAssert(offset + nStates <= rows(state));
      (*it)->setDiscreteState(state, offset);
      offset += nStates;
    }
  }
}

void
ModelGroup::getDiscreteState(Vector& state, unsigned offset) const
{
  OpenFDMAssert(offset + getNumDiscreteStates() <= rows(state));

  ModelList::const_iterator it;
  for (it = mModels.begin(); it != mModels.end(); ++it) {
    unsigned nStates = (*it)->getNumDiscreteStates();
    if (0 < nStates) {
      OpenFDMAssert(offset + nStates <= rows(state));
      (*it)->getDiscreteState(state, offset);
      offset += nStates;
    }
  }
}

bool
ModelGroup::appendDependecies(const Model* firstModel, Model* model, ModelList& newList)
{
  // If the model in question does not have dependencies, stop.
  if (!model->getDirectFeedThrough())
    return true;

  // Check, all inputs for dependencies.
  unsigned numInputs = model->getNumInputPorts();
  for (unsigned i = 0; i < numInputs; ++i) {
    Property prop = model->getInputPort(i)->getProperty();
    // might happen if some inputs are disabled
    if (!prop.isValid())
      continue;

    // Get the dependent model.
    Object* object = prop.getObject();

    // Check if it is still in the list to be scheduled.
    ModelList::iterator it = mModels.begin();
    while (it != mModels.end()) {
      if (*it == object)
        break;
      ++it;
    }
    if (it == mModels.end())
      continue;

    // Detect a circular dependency.
    if (*it == firstModel) {
      Log(Model, Warning)
        << "Detected circilar model dependency.\nRunning with a sample "
        "delay at input of \"" << (*it)->getName() << "\"!" << endl;
      return false;
    }

    // We need to store that one here since the iterator possibly invalidates
    // during the next append dependency call
    shared_ptr<Model> tmpModel = *it;
    mModels.erase(it);

    // Now recurse into that model.
    if (!appendDependecies(firstModel, tmpModel, newList))
      return false;
 
    // Finally, past all the dependent models are already in the list,
    // push that one in question.
    newList.push_back(tmpModel);
  }
}

bool
ModelGroup::sortModels(void)
{
  // TODO: use better sort algorithm.
  ModelList newList;
  while (!mModels.empty()) {
    if (!appendDependecies(mModels.front(), mModels.front(), newList))
      return false;
    // Finally, past all the dependent models are already in the list,
    // push that one in question.
    newList.push_back(mModels.front());
    mModels.erase(mModels.begin());
  }
  // Now the new ordered list is the current one.
  mModels.swap(newList);

  return true;
}

} // namespace OpenFDM
