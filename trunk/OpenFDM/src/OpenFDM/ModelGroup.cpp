/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Object.h"
#include "Model.h"
#include "Vector.h"
#include "LogStream.h"
#include "ModelVisitor.h"
#include "ModelGroup.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(ModelGroup, Model)
  END_OPENFDM_OBJECT_DEF

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

void
ModelGroup::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
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
ModelGroup::getModelIndex(const Model* model) const
{
  unsigned idx = 0u;
  ModelList::const_iterator it = mModels.begin();
  while (it != mModels.end()) {
    if ((*it) == model)
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

} // namespace OpenFDM
