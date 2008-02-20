/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <iostream>
#include <OpenFDM/DoPri5.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/SimulationTime.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/System.h>
#include <OpenFDM/UnaryFunctionModel.h>
#include "ErrorCollectorCallback.h"

using namespace OpenFDM;

int
main(int argc, char *argv[])
{
  real_type omega = 2;

  SharedPtr<System> system = new System("Harmonic Oszilator");
  system->addSampleTime(0.01);
  system->setTimestepper(new DoPri5);
  
  Integrator* integrator0 = new Integrator("Acceleration Integrator");
  Matrix initialValue(1, 1);
  initialValue(0, 0) = 0;
  integrator0->setInitialValue(initialValue);
  system->addModel(integrator0);

  Integrator* integrator1 = new Integrator("Velocity Integrator");
  initialValue(0, 0) = 1;
  integrator1->setInitialValue(initialValue);
  system->addModel(integrator1);

  Gain* gain = new Gain("Gain");
  gain->setGain(-omega*omega);
  system->addModel(gain);

  Connection::connect(gain->getInputPort(0),
                      integrator1->getOutputPort(0));
  Connection::connect(integrator1->getInputPort(0),
                      integrator0->getOutputPort(0));
  Connection::connect(integrator0->getInputPort(0),
                      gain->getOutputPort(0));

  SimulationTime* simulationTime = new SimulationTime("Simulation Time");
  system->addModel(simulationTime);

  gain = new Gain("Cosinus Input Gain");
  system->addModel(gain);
  gain->setGain(omega);
  Connection::connect(gain->getInputPort(0),
                      simulationTime->getOutputPort(0));

  UnaryFunctionModel* cosFunction
    = new UnaryFunctionModel("Exact Pos Solution", UnaryFunctionModel::Cos);
  system->addModel(cosFunction);
  Connection::connect(cosFunction->getInputPort(0),
                      gain->getOutputPort(0));

  UnaryFunctionModel* sinFunction
    = new UnaryFunctionModel("Exact Vel Solution", UnaryFunctionModel::Sin);
  system->addModel(sinFunction);
  Connection::connect(sinFunction->getInputPort(0),
                      gain->getOutputPort(0));

  gain = new Gain("Cosinus Output Gain");
  system->addModel(gain);
  gain->setGain(-omega);
  Connection::connect(gain->getInputPort(0),
                      sinFunction->getOutputPort(0));


  Summer* summer0 = new Summer("Position Error to exact Solution");
  system->addModel(summer0);
  summer0->setNumSummands(2);
  summer0->setInputSign(0, Summer::Plus);
  Connection::connect(cosFunction->getOutputPort(0),
                      summer0->getInputPort(0));
  summer0->setInputSign(1, Summer::Minus);
  Connection::connect(integrator1->getOutputPort(0),
                      summer0->getInputPort(1));

  Output* output = new Output("Position Error Output");
  SharedPtr<ErrorCollectorCallback> posErrorCallback;
  posErrorCallback = new ErrorCollectorCallback;
  output->setCallback(posErrorCallback);
  output->addSampleTime(0.1);
  system->addModel(output);
  Connection::connect(output->getInputPort(0), summer0->getOutputPort(0));


  Summer* summer1 = new Summer("Velocity Error to exact Solution");
  system->addModel(summer1);
  summer1->setNumSummands(2);
  summer1->setInputSign(0, Summer::Plus);
  Connection::connect(gain->getOutputPort(0),
                      summer1->getInputPort(0));
  summer1->setInputSign(1, Summer::Minus);
  Connection::connect(integrator0->getOutputPort(0),
                      summer1->getInputPort(1));

  output = new Output("Velocity Error Output");
  SharedPtr<ErrorCollectorCallback> velErrorCallback;
  velErrorCallback = new ErrorCollectorCallback;
  output->setCallback(velErrorCallback);
  output->addSampleTime(0.1);
  system->addModel(output);
  Connection::connect(output->getInputPort(0), summer1->getOutputPort(0));

  if (!system->init()) {
    std::cout << "Could not initialize the system" << std::endl;
    return EXIT_FAILURE;
  }

  system->simulate(10);

  real_type posError = posErrorCallback->error();
  if (1e-6 < posError) {
    std::cerr << "Position error of " << posError << " is too big" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Passed position error check with error = "
            << posError << std::endl;
  real_type velError = velErrorCallback->error();
  if (1e-6 < velError) {
    std::cerr << "Velocity error of " << velError << " is too big" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Passed velocity error check with error = "
            << velError << std::endl;

  return EXIT_SUCCESS;
}
