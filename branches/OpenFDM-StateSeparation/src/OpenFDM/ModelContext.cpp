/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "ModelContext.h"

namespace OpenFDM {

ModelContext::ModelContext(const Model* model) :
  mModel(model)
{
  OpenFDMAssert(mModel);
}

ModelContext::~ModelContext()
{
}

const Model&
ModelContext::getNode() const
{
  return *mModel;
}

} // namespace OpenFDM
