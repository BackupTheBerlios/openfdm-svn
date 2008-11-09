/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "ModelInstance.h"

namespace OpenFDM {

ModelInstance::ModelInstance(const NodePath& nodePath,
                             const SampleTime& sampleTime, const Model* model) :
  AbstractNodeInstance(nodePath, sampleTime),
  mModelContext(model->newModelContext())
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
