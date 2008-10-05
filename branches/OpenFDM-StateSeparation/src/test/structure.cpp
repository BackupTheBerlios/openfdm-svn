#include <OpenFDM/Gain.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Delay.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/LibraryNode.h>
#include <OpenFDM/LibraryModel.h>
#include <OpenFDM/System.h>

#include "HDF5Writer.h"

using namespace OpenFDM;

// Build a system with a single gain component referencing itself
bool testSelfReferencingDirectInput()
{
  Group* group = new Group("group");
  Group::NodeId gain = group->addChild(new Gain("gain"));
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

  Group::NodeId groupOutputNode = group->addProviderPort();
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

int main()
{
  SharedPtr<System> system = new System("System", buildContinousExample());
//   SharedPtr<System> system = new System("System", buildDiscreteExample());
//   SharedPtr<System> system = new System("System", buildLibraryNodeExample());

  if (!system->init())
    return 1;

  HDF5Log log("system.h5");
  log.attachTo(system);
  log.output(system->getTime());

  double h = 0.01;
  while (system->getTime() < 10) {
    system->simulate(system->getTime() + h);
    log.output(system->getTime());
  }

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

