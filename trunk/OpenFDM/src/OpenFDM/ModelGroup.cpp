/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "ModelGroup.h"

#include "Object.h"
#include "Model.h"
#include "Vector.h"
#include "LogStream.h"
#include "ModelVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(ModelGroup, Node)
  END_OPENFDM_OBJECT_DEF

ModelGroup::ModelGroup(const std::string& name) :
  Node(name)
{
}

ModelGroup::~ModelGroup(void)
{
  // Remove all references to this group.
  while (!mModels.empty())
    removeModel(mModels.front());
}

void
ModelGroup::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
}

void
ModelGroup::traverse(ModelVisitor& visitor)
{
  for (ModelList::iterator it = mModels.begin(); it != mModels.end(); ++it)
    (*it)->accept(visitor);
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

const Node*
ModelGroup::getModel(unsigned idx) const
{
  if (idx < mModels.size())
    return mModels[idx];
  else
    return 0;
}

Node*
ModelGroup::getModel(unsigned idx)
{
  if (idx < mModels.size())
    return mModels[idx];
  else
    return 0;
}

const Node*
ModelGroup::getModel(const std::string& name) const
{
  return getModel(getModelIndex(name));
}

Node*
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
ModelGroup::getModelIndex(const Node* const model) const
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
ModelGroup::addModel(Node* model, bool allowRename)
{
  // cannot add no model ...
  if (!model) {
    Log(Model, Warning)
      << "Trying to add zero OpenFDM::Model pointer "
      "to OpenFDM::ModelGroup \"" << getName() << "\"" << endl;
    return ~0u;
  }

  // cannot add a model to two groups. FIXME, will not be true in the future
  if (0 < model->getNumParents()) {
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
      if (allowRename) {
        model->setName(model->getName() + "r");
        return addModel(model, allowRename);
      } else {
        Log(Model, Warning)
          << "While adding the OpenFDM::Model \"" << model->getName()
          << "\" to OpenFDM::ModelGroup \"" << getName()
          << "\": Model with the same name is already attached!" << endl;
        return ~0u;
      }
    }
  }

  // Update the number of states.
  model->addParent(this);

  // add to the model list.
  mModels.push_back(model);

  return mModels.size()-1;
}

void
ModelGroup::removeModel(Node* model)
{
  // cannot remove no model ...
  if (!model) {
    Log(Model, Warning)
      << "Trying to remove zero OpenFDM::Model pointer "
      "from OpenFDM::ModelGroup \"" << getName() << "\"" << endl;
    return;
  }

  for (;;) {
    ModelList::iterator it = mModels.begin();
    while (it != mModels.end()) {
      if ((*it) == model)
        break;
      ++it;
    }
    // Termination condition ...
    if (it == mModels.end())
      return;

    // cannot remove if we are not its parent.
    unsigned parentIdx = 0;
    for (; parentIdx < model->getNumParents(); ++parentIdx) {
      SharedPtr<Node> parentNode = model->getParent(parentIdx).lock();
      if (parentNode == this)
        break;
    }
    OpenFDMAssert(parentIdx < model->getNumParents());

    // remove the backreference to this group
    // this also updates the number of states
    model->removeParent(parentIdx);
  
    // remove from the model list.
    // note that erasing might delete the model object, thus delete it past
    // correction of the number of states.
    mModels.erase(it);
  }
}

void
ModelGroup::addConnection(Connection* connection)
{
  ConnectionList::iterator i;
  i = std::find(mConnections.begin(), mConnections.end(), connection);
  if (i != mConnections.end())
    return;
  mConnections.push_back(connection);
}

void
ModelGroup::removeConnection(Connection* connection)
{
  ConnectionList::iterator i;
  i = std::find(mConnections.begin(), mConnections.end(), connection);
  if (i == mConnections.end())
    return;
  mConnections.erase(i);
}

unsigned
ModelGroup::getConnectionIndex(const Connection* const connection) const
{
  for (unsigned i = 0; i < mConnections.size(); ++i) {
    if (mConnections[i] == connection)
      return i;
  }
  return ~0u;
}

Connection*
ModelGroup::getConnection(unsigned i)
{
  if (mConnections.size() <= i)
    return 0;
  return mConnections[i];
}

const Connection*
ModelGroup::getConnection(unsigned i) const
{
  if (mConnections.size() <= i)
    return 0;
  return mConnections[i];
}

Model::Path
ModelGroup::getGroupPath() /* FIXME const*/
{
  Path path = getPath();
  path.push_back(this);
  return path;
}

} // namespace OpenFDM
