/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <iostream>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/SimulationTime.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/System.h>
#include "ErrorCollectorCallback.h"

using namespace OpenFDM;

int
main(int argc, char *argv[])
{
  real_type rate = 0.01;

  SharedPtr<Group> group = new Group("Group");

  ConstModel* constModel = new ConstModel("Constant", 1);
  group->addChild(constModel);

  Integrator* integrator = new Integrator("Integrator");
  integrator->setInitialValue(0);
  group->addChild(integrator);
  group->connect(constModel->getPort("output"), integrator->getPort("input"));

  SimulationTime* simulationTime = new SimulationTime("Simulation Time");
  group->addChild(simulationTime);

  Summer* summer = new Summer("Error to exact Solution");
  group->addChild(summer);
  summer->setNumSummands(2);
  summer->setInputSign(0, Summer::Plus);
  group->connect(simulationTime->getPort("output"), summer->getPort("input0"));
  summer->setInputSign(1, Summer::Minus);
  group->connect(integrator->getPort("output"), summer->getPort("input1"));


  Output* output = new Output("Error Output");
  SharedPtr<ErrorCollectorCallback> errors;
  errors = new ErrorCollectorCallback;
  output->setCallback(errors);
//   output->addSampleTime(rate);
  group->addChild(output);
  group->connect(summer->getPort("output"), output->getPort("input"));

  SharedPtr<System> system = new System("Constant Integration");
  system->setNode(group);

  if (!system->init())
    return EXIT_FAILURE;

  system->simulate(10);

  real_type expectedRoundoff;
  expectedRoundoff = 64*Limits<real_type>::epsilon()*system->getTime()/rate;
  real_type error = errors->error();
  if (expectedRoundoff < error) {
    std::cerr << "Maximum simulation time error " << error
              << " exceeds limit of expected roundoff "
              << expectedRoundoff << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Maximum simulation time error " << error
            << " passes limit of expected roundoff "
            << expectedRoundoff << std::endl;

  return EXIT_SUCCESS;
}
