/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/Summer.h>

#include "JSBSimKinemat.h"

namespace OpenFDM {

JSBSimKinemat::JSBSimKinemat(const std::string& name) :
  JSBSimFCSComponent(name, true),
  mNoScale(false)
{
  // A KINEMAT is done as a first order ODE packed into a discrete system
  // The derivative is limited to match the avarage movement speed of the
  // KINEMAT. This is not exactly like JSBSim does that, but it is
  // sufficient for now.
  //
  // -|InputGain|-|InputSaturation|-|
  //                                |
  //  -------------------------------
  //  |
  //  o-|TableLookup|-o-|Minus|-| FIXME: that table lookup is still missing
  //  |               |         |
  //  +-|ErrorGain|--|Min|----|Max|-|Integrator|-o-|Outgain|-
  //  |                                          |
  //  -----------|FeedbackGain|-------------------
  //
  // FIXME: simplify
  // FIXME: implement tables
  // FIXME: implement triggers for initialization of the integrator at the
  // first time
  mInputGain = new Gain("Input Gain");
  getModelGroup()->addModel(mInputGain);
  mInputGain->setGain(1);
  
  mInputSaturation = new Saturation("Input Saturation");
  getModelGroup()->addModel(mInputSaturation);
  mInputSaturation->getInputPort(0)->connect(mInputGain->getOutputPort(0));
  
  Summer* inputError = new Summer("Input Sum");
  getModelGroup()->addModel(inputError);
  inputError->getInputPort(0)->connect(mInputSaturation->getOutputPort(0));
  inputError->setNumSummands(2);
  
  Gain* errorGain = new Gain("Error Gain");
  getModelGroup()->addModel(errorGain);
  errorGain->setGain(100);
  errorGain->getInputPort(0)->connect(inputError->getOutputPort(0));
  
  mKinematRateLimit = new Saturation("Rate Limit");
  getModelGroup()->addModel(mKinematRateLimit);
  mKinematRateLimit->getInputPort(0)->connect(errorGain->getOutputPort(0));
  
  DiscreteIntegrator* integrator = new DiscreteIntegrator("Integrator");
  getModelGroup()->addModel(integrator);
  integrator->getInputPort(0)->connect(mKinematRateLimit->getOutputPort(0));
  Matrix tmp(1, 1);
  tmp(1, 1) = 1;
  integrator->setInitialValue(tmp);
  
  Gain* feedbackGain = new Gain("Feedback Gain");
  getModelGroup()->addModel(feedbackGain);
  feedbackGain->setGain(-1);
  feedbackGain->getInputPort(0)->connect(integrator->getOutputPort(0));
  inputError->getInputPort(1)->connect(feedbackGain->getOutputPort(0));

  mOutputNormGain = new Gain("Output Norm Gain");
  getModelGroup()->addModel(mOutputNormGain);
  mOutputNormGain->setGain(1);
  mOutputNormGain->getInputPort(0)->connect(integrator->getOutputPort(0));


  // Now connect the input and the output to this groups in and outputs
  getModelGroup()->setNumInputPorts(1);
  getModelGroup()->getInputPort(0)->setName("Input");
  mInputGain->getInputPort(0)->connect(getModelGroup()->getInputPort(0));

  getOutputPort()->connect(integrator->getOutputPort(0));
  getOutputNormPort()->connect(mOutputNormGain->getOutputPort(0));
}

JSBSimKinemat::~JSBSimKinemat(void)
{
}

void
JSBSimKinemat::setRateLimit(real_type rateLimit)
{
  rateLimit = fabs(rateLimit);
  Matrix tmp(1, 1);
  tmp(1, 1) = -rateLimit;
  mKinematRateLimit->setMinSaturation(tmp);
  tmp(1, 1) = rateLimit;
  mKinematRateLimit->setMaxSaturation(tmp);
}

void
JSBSimKinemat::setMinValue(real_type minValue)
{
  Matrix tmp(1, 1);
  tmp(1, 1) = minValue;
  mInputSaturation->setMinSaturation(tmp);
}

void
JSBSimKinemat::setMaxValue(real_type maxValue)
{
  if (mNoScale) {
    mInputGain->setGain(1);
  } else {
    mInputGain->setGain(maxValue);
  }
  mOutputNormGain->setGain(1/maxValue);

  Matrix tmp(1, 1);
  tmp(1, 1) = maxValue;
  mInputSaturation->setMaxSaturation(tmp);
}

void
JSBSimKinemat::setNoScale(bool noScale)
{
  mNoScale = noScale;
  if (mNoScale) {
    mInputGain->setGain(1);
  } else {
    Matrix maxValue = mInputSaturation->getMaxSaturation();
    mInputGain->setGain(maxValue(1, 1));
  }
}

} //namespace OpenFDM
