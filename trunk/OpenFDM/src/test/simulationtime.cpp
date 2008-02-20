/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <vector>
#include <iostream>
#include <OpenFDM/Output.h>
#include <OpenFDM/SimulationTime.h>
#include <OpenFDM/System.h>

using namespace OpenFDM;

class CollectOutputCallback : public Output::Callback {
public:
  virtual void setValue(real_type value)
  { values.push_back(value); }
  void print() const
  {
    std::vector<real_type>::const_iterator i;
    for (i = values.begin(); i != values.end(); ++i)
      std::cout << *i << std::endl;
  }
  std::vector<real_type> values;
};

int
main(int argc, char *argv[])
{
  real_type rate = 0.1;

  SharedPtr<System> system = new System("Simulation Time System");
  
  SimulationTime* simulationTime = new SimulationTime("Simulation Time");
  system->addModel(simulationTime);

  Output* output = new Output("Simulation Time Output");
  system->addModel(output);
  SharedPtr<CollectOutputCallback> simTimeCallback = new CollectOutputCallback;
  output->setCallback(simTimeCallback);
  output->addSampleTime(rate);
  Connection::connect(output->getInputPort(0),
                      simulationTime->getOutputPort(0));

  if (!system->init()) {
    std::cout << "Could not initialize the system" << std::endl;
    return EXIT_FAILURE;
  }

  system->simulate(10);

  Vector errors(simTimeCallback->values.size());
  for (unsigned i = 0; i < simTimeCallback->values.size(); ++i) {
    errors(i) = simTimeCallback->values[i] - i*rate;
  }

  real_type expectedRoundoff;
  expectedRoundoff = Limits<real_type>::epsilon()*system->getTime()/rate;
  real_type error = normInf(errors);
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
