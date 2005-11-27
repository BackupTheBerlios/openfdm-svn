/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <OpenFDM/Summer.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Gain.h>

#include "LegacyKinemat.h"

namespace OpenFDM {

LegacyKinemat::LegacyKinemat(const std::string& name) :
  ModelGroup(name),
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
  //  +-|ErrorGain|-|RateLimit|-|Integrator|-o-|Outgain|-
  //  |                                      |
  //  -----------|FeedbackGain|---------------
  //
  // FIXME: simplify
  // FIXME: implement tables
  // FIXME: implement triggers for initialization of the integrator at the
  // first time
  mInputGain = new Gain("Input Gain");
  addModel(mInputGain);
  mInputGain->setGain(1);
  
  mInputSaturation = new Saturation("Input Saturation");
  addModel(mInputSaturation);
  mInputSaturation->getInputPort(0)->connect(mInputGain->getOutputPort(0));
  
  Summer* inputError = new Summer("Input Sum");
  addModel(inputError);
  inputError->getInputPort(0)->connect(mInputSaturation->getOutputPort(0));
  inputError->setNumSummands(2);
  
  Gain* errorGain = new Gain("Error Gain");
  addModel(errorGain);
  errorGain->setGain(100);
  errorGain->getInputPort(0)->connect(inputError->getOutputPort(0));
  
  mKinematRateLimit = new Saturation("Rate Limit");
  addModel(mKinematRateLimit);
  mKinematRateLimit->getInputPort(0)->connect(errorGain->getOutputPort(0));
  
  DiscreteIntegrator* integrator = new DiscreteIntegrator("Integrator");
  addModel(integrator);
  integrator->getInputPort(0)->connect(mKinematRateLimit->getOutputPort(0));
  Matrix tmp(1, 1);
  tmp(1, 1) = 1;
  integrator->setInitialValue(tmp);
  
  Gain* feedbackGain = new Gain("Feedback Gain");
  addModel(feedbackGain);
  feedbackGain->setGain(-1);
  feedbackGain->getInputPort(0)->connect(integrator->getOutputPort(0));
  inputError->getInputPort(1)->connect(feedbackGain->getOutputPort(0));

  mOutputGain = new Gain("Normalize Gain");
  addModel(mOutputGain);
  mOutputGain->setGain(1);
  mOutputGain->getInputPort(0)->connect(integrator->getOutputPort(0));


  // Now connect the input and the output to this groups in and outputs
  setNumInputPorts(1);
  getInputPort(0)->setName("Input");
  mInputGain->getInputPort(0)->connect(getInputPort(0));

  setNumOutputPorts(1);
  getOutputPort(0)->setName("Output");
  getOutputPort(0)->connect(mOutputGain->getOutputPort(0));
}

LegacyKinemat::~LegacyKinemat(void)
{
}

void
LegacyKinemat::setRateLimit(real_type rateLimit)
{
  rateLimit = fabs(rateLimit);
  Matrix tmp(1, 1);
  tmp(1, 1) = -rateLimit;
  mKinematRateLimit->setMinSaturation(tmp);
  tmp(1, 1) = rateLimit;
  mKinematRateLimit->setMaxSaturation(tmp);
}

void
LegacyKinemat::setMinValue(real_type minValue)
{
  Matrix tmp(1, 1);
  tmp(1, 1) = minValue;
  mInputSaturation->setMinSaturation(tmp);
}

void
LegacyKinemat::setMaxValue(real_type maxValue)
{
  mInputGain->setGain(maxValue);
  if (!mNoScale)
    mOutputGain->setGain(1/maxValue);

  Matrix tmp(1, 1);
  tmp(1, 1) = maxValue;
  mInputSaturation->setMaxSaturation(tmp);
}

void
LegacyKinemat::setNoScale(bool noScale)
{
  mNoScale = noScale;
  if (mNoScale)
    mOutputGain->setGain(1);
  else
    mOutputGain->setGain(1/mInputGain->getGain());
}

} //namespace OpenFDM
