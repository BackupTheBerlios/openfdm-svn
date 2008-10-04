#include <OpenFDM/Gain.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Delay.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/System.h>

using namespace OpenFDM;

Node* buildGroupExample()
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

  //FIXME: broken naming
//   Group::NodeId groupOutputNode = group->addAcceptorPort();
  Group::NodeId groupOutputNode = group->addProviderPort();
  group->connect(integrator2, "output", groupOutputNode, "input");

  SharedPtr<Group> topGroup = new Group("G1");
  Group::NodeId child0 = topGroup->addChild(group);
  Group::NodeId child1 = topGroup->addChild(group);

  Group::NodeId output0 = topGroup->addChild(new Output("O2"));
  topGroup->connect(child0, 0, output0, 0);
  Group::NodeId output1 = topGroup->addChild(new Output("O3"));
  topGroup->connect(child1, 0, output1, 0);

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

int main()
{
//   SharedPtr<System> system = new System("System", buildGroupExample());
  SharedPtr<System> system = new System("System", buildDiscreteExample());

  if (!system->init())
    return 1;

  NodeInstanceList::const_iterator i;
  for (i = system->getNodeInstanceList().begin();
       i != system->getNodeInstanceList().end(); ++i) {
    std::cout << (*i)->getNodeNamePath() << std::endl;
    for (unsigned k = 0; k < (*i)->getNode().getNumPorts(); ++k) {
      std::cout << "  " << (*i)->getNode().getPort(k)->getName() << " "
                << (*i)->getPortValueList().getPortValue(k);
      const NumericPortValue* npv =
        dynamic_cast<const NumericPortValue*>((*i)->getPortValueList().getPortValue(k));
      if (npv)
        std::cout << " " << npv->getValue();
      std::cout << std::endl;
    }
  }

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

