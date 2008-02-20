/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <iostream>
#include <OpenFDM/ConstModel.h>
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
  real_type rate = 0.1;

  SharedPtr<System> system = new System("Constant Integration");

  Matrix m(1, 1);
  m(0, 0) = 1;

  ConstModel* constModel = new ConstModel("Constant", m);
  system->addModel(constModel);

  Integrator* integrator = new Integrator("Integrator");
  system->addModel(integrator);
  m(0, 0) = 0;
  integrator->setInitialValue(m);

  Connection::connect(integrator->getInputPort(0),
                      constModel->getOutputPort(0));

  SimulationTime* simulationTime = new SimulationTime("Simulation Time");
  system->addModel(simulationTime);

  Summer* summer = new Summer("Error to exact Solution");
  system->addModel(summer);
  summer->setNumSummands(2);
  summer->setInputSign(0, Summer::Plus);
  Connection::connect(simulationTime->getOutputPort(0),
                      summer->getInputPort(0));
  summer->setInputSign(1, Summer::Minus);
  Connection::connect(integrator->getOutputPort(0),
                      summer->getInputPort(1));

  Output* output = new Output("Error Output");
  SharedPtr<ErrorCollectorCallback> errors;
  errors = new ErrorCollectorCallback;
  output->setCallback(errors);
  output->addSampleTime(rate);
  system->addModel(output);
  Connection::connect(output->getInputPort(0), summer->getOutputPort(0));

  if (!system->init()) {
    std::cout << "Could not initialize the system" << std::endl;
    return EXIT_FAILURE;
  }

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
