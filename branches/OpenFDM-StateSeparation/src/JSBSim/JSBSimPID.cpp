/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "JSBSimPID.h"

#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/TimeDerivative.h>

namespace OpenFDM {

JSBSimPID::JSBSimPID(const std::string& name) :
  JSBSimFCSComponent(name)
{
  // The error input of the JSBSim PID controller
  GroupInput* groupInput = new GroupInput("Input");
  getGroup()->addChild(groupInput);


  mKIProduct = new Product("Integrator Gain");
  mKIProduct->setNumFactors(2);
  getGroup()->addChild(mKIProduct);
  getGroup()->connect(mKIProduct->getInputPort(0),
                      groupInput->getPort("output"));

  DiscreteIntegrator* mIntegrator = new DiscreteIntegrator("Integrator");
  getGroup()->addChild(mIntegrator);
  getGroup()->connect(mKIProduct->getOutputPort(),
                      mIntegrator->getPort("input"));

  
  mKPProduct = new Product("Proportional Gain");
  mKPProduct->setNumFactors(2);
  getGroup()->addChild(mKPProduct);
  getGroup()->connect(mKPProduct->getInputPort(0),
                      groupInput->getPort("output"));


  mKDProduct = new Product("Derivative Gain");
  mKDProduct->setNumFactors(2);
  getGroup()->addChild(mKDProduct);
  getGroup()->connect(mKDProduct->getInputPort(0),
                      groupInput->getPort("output"));

  TimeDerivative* mDerivative = new TimeDerivative("Derivative");
  getGroup()->addChild(mDerivative);
  getGroup()->connect(mKDProduct->getOutputPort(),
                      mDerivative->getPort("input"));

  Summer* mControlSum = new Summer("Control Sum");
  mControlSum->setNumSummands(3);
  getGroup()->addChild(mControlSum);
  getGroup()->connect(mIntegrator->getPort("output"),
                      mControlSum->getInputPort(0));
  getGroup()->connect(mKPProduct->getPort("output"),
                      mControlSum->getInputPort(1));
  getGroup()->connect(mDerivative->getPort("output"),
                      mControlSum->getInputPort(2));

  // That single output port is this one
  getGroup()->connect(getInternalOutputPort(),
                      mControlSum->getOutputPort());
}

JSBSimPID::~JSBSimPID(void)
{
}

void
JSBSimPID::setKI(real_type ki)
{
  if (mKIGroupInput) {
    getGroup()->removeChild(mKIGroupInput);
    mKIGroupInput = 0;
  }
  if (mKIConnect) {
    getGroup()->removeConnect(mKIConnect);
    mKIConnect = 0;
  }
  mKIConstant = new ConstModel("ki", ki);
  getGroup()->addChild(mKIConstant);
  mKIConnect = getGroup()->connect(mKIConstant->getPort("output"),
                                   mKIProduct->getInputPort(1));
}

const Port*
JSBSimPID::getKIPort()
{
  if (mKIConstant) {
    getGroup()->removeChild(mKIConstant);
    mKIConstant = 0;
  }
  if (mKIConnect) {
    getGroup()->removeConnect(mKIConnect);
    mKIConnect = 0;
  }
  mKIGroupInput = new GroupInput("ki");
  getGroup()->addChild(mKIGroupInput);
  mKIGroupInput->setExternalPortName("ki");
  mKIConnect = getGroup()->connect(mKIGroupInput->getPort("output"),
                                   mKIProduct->getInputPort(1));
  
  return getGroup()->getPort(mKIGroupInput->getExternalPortIndex());
}

void
JSBSimPID::setKP(real_type kp)
{
  if (mKPGroupInput) {
    getGroup()->removeChild(mKPGroupInput);
    mKPGroupInput = 0;
  }
  if (mKPConnect) {
    getGroup()->removeConnect(mKPConnect);
    mKPConnect = 0;
  }
  mKPConstant = new ConstModel("kp", kp);
  getGroup()->addChild(mKPConstant);
  mKPConnect = getGroup()->connect(mKPConstant->getPort("output"),
                                   mKPProduct->getInputPort(1));
}

const Port*
JSBSimPID::getKPPort()
{
  if (mKPConstant) {
    getGroup()->removeChild(mKPConstant);
    mKPConstant = 0;
  }
  if (mKPConnect) {
    getGroup()->removeConnect(mKPConnect);
    mKPConnect = 0;
  }
  mKPGroupInput = new GroupInput("kp");
  getGroup()->addChild(mKPGroupInput);
  mKPGroupInput->setExternalPortName("kp");
  mKPConnect = getGroup()->connect(mKPGroupInput->getPort("output"),
                                   mKPProduct->getInputPort(1));

  return getGroup()->getPort(mKPGroupInput->getExternalPortIndex());
}

void
JSBSimPID::setKD(real_type kd)
{
  if (mKDGroupInput) {
    getGroup()->removeChild(mKDGroupInput);
    mKDGroupInput = 0;
  }
  if (mKDConnect) {
    getGroup()->removeConnect(mKDConnect);
    mKDConnect = 0;
  }
  mKDConstant = new ConstModel("kd", kd);
  getGroup()->addChild(mKDConstant);
  mKDConnect = getGroup()->connect(mKDConstant->getPort("output"),
                                   mKDProduct->getInputPort(1));
}

const Port*
JSBSimPID::getKDPort()
{
  if (mKDConstant) {
    getGroup()->removeChild(mKDConstant);
    mKDConstant = 0;
  }
  if (mKDConnect) {
    getGroup()->removeConnect(mKDConnect);
    mKDConnect = 0;
  }
  mKDGroupInput = new GroupInput("kd");
  getGroup()->addChild(mKDGroupInput);
  mKDGroupInput->setExternalPortName("kd");
  mKDConnect = getGroup()->connect(mKDGroupInput->getPort("output"),
                                   mKDProduct->getInputPort(1));

  return getGroup()->getPort(mKDGroupInput->getExternalPortIndex());
}

} //namespace OpenFDM
