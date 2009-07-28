/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "JSBSimKinemat.h"

#include <OpenFDM/GroupInput.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/Summer.h>

namespace OpenFDM {

JSBSimKinemat::JSBSimKinemat(const std::string& name) :
  JSBSimFCSComponent(name),
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
  //  o-|BreakPointVector|-o-|Minus|-| FIXME: that table lookup is still missing
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
  getGroup()->addChild(mInputGain);
  mInputGain->setGain(1);
  
  mInputSaturation = new Saturation("Input Saturation");
  getGroup()->addChild(mInputSaturation);
  getGroup()->connect(mInputSaturation->getPort("input"),
                      mInputGain->getPort("output"));
  
  Summer* inputError = new Summer("Input Sum");
  getGroup()->addChild(inputError);
  getGroup()->connect(inputError->getPort("input0"),
                      mInputSaturation->getPort("output"));
  inputError->setNumSummands(2);
  
  Gain* errorGain = new Gain("Error Gain");
  getGroup()->addChild(errorGain);
  errorGain->setGain(100);
  getGroup()->connect(errorGain->getPort("input"),
                      inputError->getPort("output"));
  
  mKinematRateLimit = new Saturation("Rate Limit");
  getGroup()->addChild(mKinematRateLimit);
  getGroup()->connect(mKinematRateLimit->getPort("input"),
                      errorGain->getPort("output"));
  
  DiscreteIntegrator* integrator = new DiscreteIntegrator("Integrator");
  getGroup()->addChild(integrator);
  getGroup()->connect(integrator->getPort("input"),
                      mKinematRateLimit->getPort("output"));
  Matrix tmp(1, 1);
  tmp(0, 0) = 1;
  integrator->setInitialValue(tmp);
  getGroup()->connect(integrator->getPort("initialValue"),
                      mInputSaturation->getPort("output"));
  
  Gain* feedbackGain = new Gain("Feedback Gain");
  getGroup()->addChild(feedbackGain);
  feedbackGain->setGain(-1);
  getGroup()->connect(feedbackGain->getPort("input"),
                      integrator->getPort("output"));
  getGroup()->connect(inputError->getPort("input1"),
                      feedbackGain->getPort("output"));

  mOutputNormGain = new Gain("Output Norm Gain");
  getGroup()->addChild(mOutputNormGain);
  mOutputNormGain->setGain(1);
  getGroup()->connect(mOutputNormGain->getPort("input"),
                      integrator->getPort("output"));

  // Now connect the input and the output to this groups in and outputs
  GroupInput* groupInput = new GroupInput("Input");
  getGroup()->addChild(groupInput);
  getGroup()->connect(mInputGain->getPort("input"),
                      groupInput->getPort("output"));
 
  // That single output port is this one
  getGroup()->connect(getInternalOutputPort(),
                      integrator->getPort("output"));
  getGroup()->connect(getInternalOutputNormPort(),
                      mOutputNormGain->getPort("output"));
}

JSBSimKinemat::~JSBSimKinemat(void)
{
}

void
JSBSimKinemat::setRateLimit(real_type rateLimit)
{
  rateLimit = fabs(rateLimit);
  Matrix tmp(1, 1);
  tmp(0, 0) = -rateLimit;
  mKinematRateLimit->setMinSaturation(tmp);
  tmp(0, 0) = rateLimit;
  mKinematRateLimit->setMaxSaturation(tmp);
}

void
JSBSimKinemat::setMinValue(real_type minValue)
{
  Matrix tmp(1, 1);
  tmp(0, 0) = minValue;
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
  tmp(0, 0) = maxValue;
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
    mInputGain->setGain(maxValue(0, 0));
  }
}

} //namespace OpenFDM
