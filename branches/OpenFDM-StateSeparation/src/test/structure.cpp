#include <OpenFDM/Gain.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Delay.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/GroupInput.h>
#include <OpenFDM/GroupOutput.h>
#include <OpenFDM/LibraryNode.h>
#include <OpenFDM/LibraryModel.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>

using namespace OpenFDM;

// Build a system with a single gain component referencing itself
bool testSelfReferencingDirectInput()
{
  SharedPtr<Group> group = new Group("Group");
  Gain* gain = new Gain("Gain");
  group->addChild(gain);
  group->connect(gain->getPort("output"), gain->getPort("input"));

  SharedPtr<System> system = new System("Self referencing Gain");
  system->setNode(group);

  if (system->init()) {
    std::cerr << "Detection of self referencing direct input loops failed!"
              << std::endl;
    return false;
  }
  return true;
}

// build up a cyclic direct input loop involving more than one model
bool testCyclicDependency()
{
  SharedPtr<Group> group = new Group("Group");
  Gain* gain1 = new Gain("Gain 1");
  group->addChild(gain1);
  Gain* gain2 = new Gain("Gain 1");
  group->addChild(gain2);
  group->connect(gain1->getPort("output"), gain2->getPort("input"));
  group->connect(gain2->getPort("output"), gain1->getPort("input"));

  SharedPtr<System> system = new System("Cyclic loop");
  system->setNode(group);

  if (system->init()) {
    std::cerr << "Detection of direct input loops failed!"
              << std::endl;
    return false;
  }
  return true;
}

// build up a cyclic direct input loop involving more than one model
bool testCyclicDependencyWithGroup1()
{
  SharedPtr<Group> group1 = new Group("Group 1");
  GroupInput* groupInput = new GroupInput("Input 1");
  group1->addChild(groupInput);
  GroupOutput* groupOutput = new GroupOutput("Output 1");
  group1->addChild(groupOutput);
  group1->connect(groupInput->getPort("output"), groupOutput->getPort("input"));

  SharedPtr<Group> group = new Group("Group");
  group->addChild(group1);
  group->connect(group1->getPort("output"), group1->getPort("input"));

  SharedPtr<System> system = new System("Cyclic loop through groups");
  system->setNode(group);

  if (system->init()) {
    std::cerr << "Detection of direct input loops failed!"
              << std::endl;
    return false;
  }
  return true;
}

bool testCyclicDependencyWithGroup2()
{
  SharedPtr<Group> group1 = new Group("Group 1");
  GroupInput* groupInput1 = new GroupInput("Input 1");
  group1->addChild(groupInput1);
  Gain* gain1 = new Gain("Gain 1");
  group1->addChild(gain1);
  GroupOutput* groupOutput1 = new GroupOutput("Output 1");
  group1->addChild(groupOutput1);
  group1->connect(groupInput1->getPort("output"), gain1->getPort("input"));
  group1->connect(gain1->getPort("output"), groupOutput1->getPort("input"));

  SharedPtr<Group> group2 = new Group("Group 2");
  GroupInput* groupInput2 = new GroupInput("Input 2");
  group2->addChild(groupInput2);
  Gain* gain2 = new Gain("Gain 2");
  group2->addChild(gain2);
  GroupOutput* groupOutput2 = new GroupOutput("Output 2");
  group2->addChild(groupOutput2);
  group2->connect(groupInput2->getPort("output"), gain2->getPort("input"));
  group2->connect(gain2->getPort("output"), groupOutput2->getPort("input"));

  SharedPtr<Group> group = new Group("Group");
  group->addChild(group1);
  group->addChild(group2);
  group->connect(group1->getPort("output"), group2->getPort("input"));
  group->connect(group2->getPort("output"), group1->getPort("input"));

  SharedPtr<System> system = new System("Cyclic loop through groups");
  system->setNode(group);

  if (system->init()) {
    std::cerr << "Detection of direct input loops failed!"
              << std::endl;
    return false;
  }
  return true;
}

int main()
{
  // Check a self referencing gain model, to see if cyclic loops
  // are properly detected
  if (!testSelfReferencingDirectInput())
    return EXIT_FAILURE;

  // Check for cyclic loop analysis
  if (!testCyclicDependency())
    return EXIT_FAILURE;

  // Check for cyclic loop analysis through groups.
  // Als kind of checks if connections through groups work
  if (!testCyclicDependencyWithGroup1())
    return EXIT_FAILURE;
  if (!testCyclicDependencyWithGroup2())
    return EXIT_FAILURE;

  std::cout << "PASSED" << std::endl;

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

