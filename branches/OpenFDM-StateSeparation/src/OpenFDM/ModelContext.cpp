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

bool ModelContext::dependsOn(const ModelContext& modelContext) const
{
  unsigned numPorts = mModel->getNumPorts();
  for (unsigned i = 0; i < numPorts; ++i) {
    const InputPortInfo* inputPortInfo;
    inputPortInfo = mModel->getPort(i)->toInputPortInfo();
    if (!inputPortInfo)
      continue;
    if (!inputPortInfo->getDirectInput())
      continue;
    const PortValue* portValue = getPortValueList().getPortValue(i);
    if (!portValue)
      continue;
    unsigned otherNumPorts = modelContext.mModel->getNumPorts();
    for (unsigned j = 0; j < otherNumPorts; ++j) {
      if (!modelContext.mModel->getPort(j)->toOutputPortInfo())
        continue;
      
      const PortValue* otherPortValue;
      otherPortValue = modelContext.getPortValueList().getPortValue(j);
      if (portValue != otherPortValue)
        continue;
      
      return true;
    }
  }
  return false;
}

} // namespace OpenFDM
