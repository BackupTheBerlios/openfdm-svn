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
    const AcceptorPortInfo* acceptorPortInfo;
    acceptorPortInfo = mModel->getPort(i)->toAcceptorPortInfo();
    if (!acceptorPortInfo)
      continue;
    if (!acceptorPortInfo->getDirectInput())
      continue;
    const PortValue* portValue = getPortValueList().getPortValue(i);
    if (!portValue)
      continue;
    unsigned otherNumPorts = modelContext.mModel->getNumPorts();
    for (unsigned j = 0; j < otherNumPorts; ++j) {
      if (!modelContext.mModel->getPort(j)->toProviderPortInfo())
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
