/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "JSBSimFCSComponent.h"

#include <OpenFDM/GroupOutput.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/Gain.h>

namespace OpenFDM {

JSBSimFCSComponent::JSBSimFCSComponent(const std::string& name)
{
  mModelGroup = new ModelGroup(name);
}

JSBSimFCSComponent::~JSBSimFCSComponent(void)
{
}

NumericPortProvider*
JSBSimFCSComponent::getOutputPort(void)
{
  return mModelGroup->getOutputPort(0);
}

NumericPortProvider*
JSBSimFCSComponent::getOutputNormPort(void)
{
  return mModelGroup->getOutputPort(1);
}

NumericPortAcceptor*
JSBSimFCSComponent::getInternalOutputPort(void)
{
  if (mInternalOutputPort)
    return mInternalOutputPort;

  GroupOutput* groupOutput = new GroupOutput("Output");
  getModelGroup()->addModel(groupOutput);
  mInternalOutputPort = groupOutput->getInputPort(0);
  return mInternalOutputPort;
}

NumericPortAcceptor*
JSBSimFCSComponent::getInternalOutputNormPort(void)
{
  if (mInternalOutputNormPort)
    return mInternalOutputNormPort;

  GroupOutput* groupOutput = new GroupOutput("OutputNorm");
  getModelGroup()->addModel(groupOutput);
  mInternalOutputNormPort = groupOutput->getInputPort(0);
  return mInternalOutputNormPort;
}

} //namespace OpenFDM
