/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "JSBSimFCSComponent.h"

#include <OpenFDM/GroupOutput.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Gain.h>

namespace OpenFDM {

JSBSimFCSComponent::JSBSimFCSComponent(const std::string& name)
{
  mGroup = new Group(name);
}

JSBSimFCSComponent::~JSBSimFCSComponent(void)
{
}

const Port*
JSBSimFCSComponent::getOutputPort(void)
{
  unsigned index = getOutputModel()->getExternalPortIndex();
  return mGroup->getPort(index);
}

const Port*
JSBSimFCSComponent::getOutputNormPort(void)
{
  unsigned index = getOutputNormModel()->getExternalPortIndex();
  return mGroup->getPort(index);
}

const Port*
JSBSimFCSComponent::getInternalOutputPort(void)
{
  return getOutputModel()->getPort("input");
}

const Port*
JSBSimFCSComponent::getInternalOutputNormPort(void)
{
  return getOutputNormModel()->getPort("input");
}

GroupOutput*
JSBSimFCSComponent::getOutputModel(void)
{
  if (mOutputModel)
    return mOutputModel;

  mOutputModel = new GroupOutput("Output");
  getGroup()->addChild(mOutputModel);
  mOutputModel->setExternalPortName("output");
  return mOutputModel;
}

GroupOutput*
JSBSimFCSComponent::getOutputNormModel(void)
{
  if (mOutputNormModel)
    return mOutputNormModel;

  mOutputNormModel = new GroupOutput("OutputNorm");
  getGroup()->addChild(mOutputNormModel);
  mOutputNormModel->setExternalPortName("outputNorm");
  return mOutputNormModel;
}

} //namespace OpenFDM
