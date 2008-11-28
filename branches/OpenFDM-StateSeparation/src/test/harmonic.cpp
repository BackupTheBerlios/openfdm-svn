/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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
  Group::NodeId gainId = group->addChild(new Gain("gain", -omega*omega));
  Integrator* integrator0 = new Integrator("Velocity Integrator");
  integrator0->setInitialValue(omega);
  Group::NodeId integrator0Id = group->addChild(integrator0);
  Integrator* integrator1 = new Integrator("Position Integrator");
  integrator1->setInitialValue(0);
  Group::NodeId integrator1Id = group->addChild(integrator1);

  group->connect(gainId, "output", integrator0Id, "input");
  group->connect(integrator0Id, "output", integrator1Id, "input");
  group->connect(integrator1Id, "output", gainId, "input");

  // Build up the exact solution
  SimulationTime* simulationTime = new SimulationTime("Simulation Time");
  Group::NodeId simulationTimeId = group->addChild(simulationTime);

  Gain* cosInputGain = new Gain("Cosinus Input Gain", omega);
  Group::NodeId cosInputGainId = group->addChild(cosInputGain);
  group->connect(simulationTimeId, "output", cosInputGainId, "input");

  UnaryFunction* cosFunction;
  cosFunction = new UnaryFunction("Exact Vel Solution", UnaryFunction::Cos);
  Group::NodeId cosFunctionId = group->addChild(cosFunction);
  group->connect(cosInputGainId, "output", cosFunctionId, "input");

  Gain* velOutputGain = new Gain("Velocity Output Gain", omega);
  Group::NodeId velOutputGainId = group->addChild(velOutputGain);
  group->connect(cosFunctionId, "output", velOutputGainId, "input");

  UnaryFunction* sinFunction;
  sinFunction = new UnaryFunction("Exact Pos Solution", UnaryFunction::Sin);
  Group::NodeId sinFunctionId = group->addChild(sinFunction);
  group->connect(cosInputGainId, "output", sinFunctionId, "input");


  // Now build the differences

  Summer* summer0 = new Summer("Velocity Error to exact Solution");
  Group::NodeId summer0Id = group->addChild(summer0);
  summer0->setNumSummands(2);
  summer0->setInputSign(0, Summer::Plus);
  group->connect(velOutputGainId, "output", summer0Id, "input0");
  summer0->setInputSign(1, Summer::Minus);
  group->connect(integrator0Id, "output", summer0Id, "input1");

  Output* output0 = new Output("Velocity Error Output");
  SharedPtr<ErrorCollectorCallback> velErrorCallback;
  velErrorCallback = new ErrorCollectorCallback;
  output0->setCallback(velErrorCallback);
  Group::NodeId output0Id = group->addChild(output0);
  group->connect(summer0Id, "output", output0Id, "input");


  Summer* summer1 = new Summer("Position Error to exact Solution");
  Group::NodeId summer1Id = group->addChild(summer1);
  summer1->setNumSummands(2);
  summer1->setInputSign(0, Summer::Plus);
  group->connect(sinFunctionId, "output", summer1Id, "input0");
  summer1->setInputSign(1, Summer::Minus);
  group->connect(integrator1Id, "output", summer1Id, "input1");

  Output* output1 = new Output("Position Error Output");
  SharedPtr<ErrorCollectorCallback> posErrorCallback;
  posErrorCallback = new ErrorCollectorCallback;
  output1->setCallback(posErrorCallback);
  Group::NodeId output1Id = group->addChild(output1);
  group->connect(summer1Id, "output", output1Id, "input");

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
