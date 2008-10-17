#include <OpenFDM/Gain.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Delay.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/LibraryNode.h>
#include <OpenFDM/LibraryModel.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>

using namespace OpenFDM;

// Build a system with a single gain component referencing itself
bool testSelfReferencingDirectInput()
{
  SharedPtr<Group> group = new Group("Group");
  Group::NodeId gain = group->addChild(new Gain("Gain"));
  group->connect(gain, "output", gain, "input");

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
  Group::NodeId gain1 = group->addChild(new Gain("Gain 1"));
  Group::NodeId gain2 = group->addChild(new Gain("Gain 2"));
  group->connect(gain1, "output", gain2, "input");
  group->connect(gain2, "output", gain1, "input");

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
  Group::NodeId groupInput1 = group1->addChild(new GroupInput("Input 1"));
  Group::NodeId groupOutput1 = group1->addChild(new GroupOutput("Output 1"));
  group1->connect(groupInput1, "output", groupOutput1, "input");

  SharedPtr<Group> group = new Group("Group");
  Group::NodeId groupId1 = group->addChild(group1);
  group->connect(groupId1, "output", groupId1, "input");

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
  Group::NodeId groupInput1 = group1->addChild(new GroupInput("Input 1"));
  Group::NodeId gain1 = group1->addChild(new Gain("Gain 1"));
  Group::NodeId groupOutput1 = group1->addChild(new GroupOutput("Output 1"));
  group1->connect(groupInput1, "output", gain1, "input");
  group1->connect(gain1, "output", groupOutput1, "input");

  SharedPtr<Group> group2 = new Group("Group 2");
  Group::NodeId groupInput2 = group2->addChild(new GroupInput("Input 2"));
  Group::NodeId gain2 = group2->addChild(new Gain("Gain 2"));
  Group::NodeId groupOutput2 = group2->addChild(new GroupOutput("Output 2"));
  group2->connect(groupInput2, "output", gain2, "input");
  group2->connect(gain2, "output", groupOutput2, "input");

  SharedPtr<Group> group = new Group("Group");
  Group::NodeId groupId1 = group->addChild(group1);
  Group::NodeId groupId2 = group->addChild(group2);
  group->connect(groupId1, "output", groupId2, "input");
  group->connect(groupId2, "output", groupId1, "input");

  SharedPtr<System> system = new System("Cyclic loop through groups");
  system->setNode(group);

  if (system->init()) {
    std::cerr << "Detection of direct input loops failed!"
              << std::endl;
    return false;
  }
  return true;
}

Node* buildContinousExample()
{
  SharedPtr<Group> group = new Group("G0");
  Group::NodeId gain = group->addChild(new Gain("gain", -1));
  Integrator* i1 = new Integrator("I1");
  Matrix v(1, 1);
  v(0, 0) = 1;
  i1->setInitialValue(v);
  Group::NodeId integrator1 = group->addChild(i1);
  Group::NodeId integrator2 = group->addChild(new Integrator("I2"));
  Group::NodeId output = group->addChild(new Output("O"));
  Group::NodeId delay = group->addChild(new Delay("D"));
  Group::NodeId outputDelay = group->addChild(new Output("OD"));

  group->connect(integrator1, "output", integrator2, "input");
  group->connect(integrator2, "output", gain, "input");
  group->connect(gain, "output", integrator1, "input");
  group->connect(integrator2, "output", output, "input");
  group->connect(gain, "output", delay, "input");
  group->connect(delay, "output", outputDelay, "input");

  Group::NodeId groupOutputNode = group->addChild(new GroupOutput("GIO"));
  group->connect(integrator2, "output", groupOutputNode, "input");

  SharedPtr<Group> topGroup = new Group("G1");
  Group::NodeId child = topGroup->addChild(group);

  Group::NodeId output0 = topGroup->addChild(new Output("Output"));
  topGroup->connect(child, 0, output0, 0);

  return topGroup.release();
}

Node* buildDiscreteExample()
{
  SharedPtr<Group> group = new Group("G0");
  Group::NodeId gain = group->addChild(new Gain("gain", -1));
  DiscreteIntegrator* di1 = new DiscreteIntegrator("I1");
  Matrix v(1, 1);
  v(0, 0) = 10;
  di1->setInitialValue(v);
  Group::NodeId integrator1 = group->addChild(di1);
  Group::NodeId integrator2 = group->addChild(new DiscreteIntegrator("I2"));
  Group::NodeId output = group->addChild(new Output("O"));
  Group::NodeId delay = group->addChild(new Delay("D"));
  Group::NodeId outputDelay = group->addChild(new Output("OD"));

  group->connect(integrator1, "output", integrator2, "input");
  group->connect(integrator2, "output", gain, "input");
  group->connect(gain, "output", integrator1, "input");
  group->connect(integrator2, "output", output, "input");
  group->connect(gain, "output", delay, "input");
  group->connect(delay, "output", outputDelay, "input");

  return group.release();
}

Node* buildLibraryNodeExample()
{
  SharedPtr<Node> node = buildDiscreteExample();
  SharedPtr<LibraryModel> libraryModel = new LibraryModel("Library Model");
  libraryModel->setNode(node);
  
  SharedPtr<LibraryNode> libraryNode1 = new LibraryNode("Library Node 1");
  libraryNode1->setLibraryModel(libraryModel);

  SharedPtr<LibraryNode> libraryNode2 = new LibraryNode("Library Node 2");
  libraryNode2->setLibraryModel(libraryModel);

  SharedPtr<Group> group = new Group("Group");
  group->addChild(libraryNode1);
  group->addChild(libraryNode2);
  return group.release();
}

Node* buildSimpleMechanicExample()
{
  /// sensible test cases:
  /// drop: gravity
  /// throw: just this test with a different start condition
  /// harmonic oszilator: compare with 2nd order linear system
  /// arrow: see if the tip stays in front
  /// satellit: coriolis
  /// paris pendulum: coriolis

  SharedPtr<Group> group = new Group("G");
  Group::NodeId rootJoint = group->addChild(new MobileRootJoint("Root Joint"));
  Group::NodeId rigidBody = group->addChild(new RigidBody("Rigid Body"));
  Group::NodeId mass = group->addChild(new Mass("Mass", 1, InertiaMatrix(1, 0, 0, 1, 0, 1)));

  group->connect(rootJoint, "link", rigidBody, "link");
  group->connect(rigidBody, "link2", mass, "link");

  return group.release();
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


//   SharedPtr<System> system = new System("System", buildContinousExample());
//   SharedPtr<System> system = new System("System", buildDiscreteExample());
//   SharedPtr<System> system = new System("System", buildLibraryNodeExample());
  SharedPtr<System> system = new System("System", buildSimpleMechanicExample());

  system->attach(SystemOutput::newDefaultSystemOutput("system.h5"));

  if (!system->init())
    return 1;

  system->simulate(10);

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

