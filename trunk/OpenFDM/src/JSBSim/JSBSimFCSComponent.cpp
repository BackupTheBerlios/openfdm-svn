/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/Gain.h>

#include "JSBSimFCSComponent.h"

namespace OpenFDM {

JSBSimFCSComponent::JSBSimFCSComponent(const std::string& name, bool normOut)
{
  mModelGroup = new ModelGroup(name);

  mModelGroup->setNumOutputPorts(1);
  getOutputPort()->setName("Output");

  if (normOut) {
    mModelGroup->setNumOutputPorts(2);
    getOutputNormPort()->setName("OutputNorm");
  }
}

JSBSimFCSComponent::~JSBSimFCSComponent(void)
{
}

Port*
JSBSimFCSComponent::getOutputPort(void)
{
  return mModelGroup->getOutputPort(0);
}

Port*
JSBSimFCSComponent::getOutputNormPort(void)
{
  return mModelGroup->getOutputPort(1);
}

} //namespace OpenFDM
