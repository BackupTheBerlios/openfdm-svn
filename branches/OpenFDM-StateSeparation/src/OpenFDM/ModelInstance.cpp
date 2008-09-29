/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "ModelInstance.h"

namespace OpenFDM {

ModelInstance::ModelInstance(const NodePath& nodePath, const Model* model) :
  AbstractNodeInstance(nodePath),
  mModelContext(new ModelContext(model))
{
}

ModelInstance::~ModelInstance()
{
}

ModelContext&
ModelInstance::getNodeContext()
{
  return *mModelContext;
}

const ModelContext&
ModelInstance::getNodeContext() const
{
  return *mModelContext;
}

} // namespace OpenFDM
