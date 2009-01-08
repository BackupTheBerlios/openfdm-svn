/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <iostream>
#include <OpenFDM/DoPri5.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/SimulationTime.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/System.h>
#include <OpenFDM/UnaryFunction.h>
#include "ErrorCollectorCallback.h"

using namespace OpenFDM;

int
main(int argc, char *argv[])
{
  real_type omega = 2;

  SharedPtr<Group> group = new Group("Group");

  // Build up a harmonic oszilator
  Gain* gain = new Gain("gain", -omega*omega);
  group->addChild(gain);
  Integrator* integrator0 = new Integrator("Velocity Integrator");
  integrator0->setInitialValue(omega);
  group->addChild(integrator0);
  Integrator* integrator1 = new Integrator("Position Integrator");
  integrator1->setInitialValue(0);
  group->addChild(integrator1);

  group->connect(gain->getPort("output"), integrator0->getPort("input"));
  group->connect(integrator0->getPort("output"), integrator1->getPort("input"));
  group->connect(integrator1->getPort("output"), gain->getPort("input"));

  // Build up the exact solution
  SimulationTime* simulationTime = new SimulationTime("Simulation Time");
  group->addChild(simulationTime);

  Gain* cosInputGain = new Gain("Cosinus Input Gain", omega);
  group->addChild(cosInputGain);
  group->connect(simulationTime->getPort("output"), cosInputGain->getPort("input"));

  UnaryFunction* cosFunction;
  cosFunction = new UnaryFunction("Exact Vel Solution", UnaryFunction::Cos);
  group->addChild(cosFunction);
  group->connect(cosInputGain->getPort("output"), cosFunction->getPort("input"));

  Gain* velOutputGain = new Gain("Velocity Output Gain", omega);
  group->addChild(velOutputGain);
  group->connect(cosFunction->getPort("output"), velOutputGain->getPort("input"));

  UnaryFunction* sinFunction;
  sinFunction = new UnaryFunction("Exact Pos Solution", UnaryFunction::Sin);
  group->addChild(sinFunction);
  group->connect(cosInputGain->getPort("output"), sinFunction->getPort("input"));


  // Now build the differences

  Summer* summer0 = new Summer("Velocity Error to exact Solution");
  group->addChild(summer0);
  summer0->setNumSummands(2);
  summer0->setInputSign(0, Summer::Plus);
  group->connect(velOutputGain->getPort("output"), summer0->getPort("input0"));
  summer0->setInputSign(1, Summer::Minus);
  group->connect(integrator0->getPort("output"), summer0->getPort("input1"));

  Output* output0 = new Output("Velocity Error Output");
  SharedPtr<ErrorCollectorCallback> velErrorCallback;
  velErrorCallback = new ErrorCollectorCallback;
  output0->setCallback(velErrorCallback);
  group->addChild(output0);
  group->connect(summer0->getPort("output"), output0->getPort("input"));


  Summer* summer1 = new Summer("Position Error to exact Solution");
  group->addChild(summer1);
  summer1->setNumSummands(2);
  summer1->setInputSign(0, Summer::Plus);
  group->connect(sinFunction->getPort("output"), summer1->getPort("input0"));
  summer1->setInputSign(1, Summer::Minus);
  group->connect(integrator1->getPort("output"), summer1->getPort("input1"));

  Output* output1 = new Output("Position Error Output");
  SharedPtr<ErrorCollectorCallback> posErrorCallback;
  posErrorCallback = new ErrorCollectorCallback;
  output1->setCallback(posErrorCallback);
  group->addChild(output1);
  group->connect(summer1->getPort("output"), output1->getPort("input"));

  SharedPtr<System> system = new System("Harmonic Oszilator");
  system->setNode(group);

  if (!system->init()) {
    std::cout << "Could not initialize the system" << std::endl;
    return EXIT_FAILURE;
  }

  system->simulate(10);

  real_type posError = posErrorCallback->error();
  std::cout << "Position error check with error = " << posError << std::endl;
  real_type velError = velErrorCallback->error();
  std::cout << "Velocity error check with error = " << velError << std::endl;
  if (1e-6 < posError) {
    std::cerr << "Position error of is too big!" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Passed position error check." << std::endl;
  if (1e-6 < velError) {
    std::cerr << "Velocity error of is too big!" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Passed velocity error check." << std::endl;

  return EXIT_SUCCESS;
}
