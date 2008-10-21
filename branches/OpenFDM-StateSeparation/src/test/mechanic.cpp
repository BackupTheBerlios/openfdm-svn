#include <OpenFDM/Group.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>

using namespace OpenFDM;

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

  group->connect(rootJoint, "link", rigidBody, "link0");
  group->connect(rigidBody, "link1", mass, "link");

  return group.release();
}

int main()
{
  SharedPtr<System> system = new System("System", buildSimpleMechanicExample());

  system->attach(SystemOutput::newDefaultSystemOutput("system.h5"));

  if (!system->init())
    return 1;

  system->simulate(10);

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

