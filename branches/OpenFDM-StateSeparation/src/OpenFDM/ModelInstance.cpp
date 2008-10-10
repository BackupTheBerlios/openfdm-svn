/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "ModelInstance.h"

namespace OpenFDM {

ModelInstance::ModelInstance(const NodePath& nodePath,
                             const SampleTime& sampleTime, const Model* model) :
  AbstractNodeInstance(nodePath, sampleTime),
  mModelContext(new ModelContext(model))
{
}

ModelInstance::~ModelInstance()
{
}

bool
ModelInstance::dependsOn(const ModelInstance& modelInstance) const
{
  return mModelContext->dependsOn(*modelInstance.mModelContext);
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
