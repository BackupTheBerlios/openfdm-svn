#include <OpenFDM/Group.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/FixedRootJoint.h>
#include <OpenFDM/UniversalJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>

using namespace OpenFDM;

int main()
{
  // Model of the paris pendulum or foucault pendulum to test coriolis effects.

  SharedPtr<Group> group = new Group("Foucault");
  Group::NodeId root = group->addChild(new FixedRootJoint("Root"));
  Group::NodeId universal = group->addChild(new UniversalJoint("Universal"));
  Group::NodeId rigidBody = group->addChild(new RigidBody("Rigid Body"));
  InertiaMatrix inertia(1, 0, 0, 1, 0, 1);
  Group::NodeId mass = group->addChild(new Mass("Mass", 1, inertia));

  group->connect(root, 0, universal, 0);
  group->connect(universal, 1, rigidBody, 0);
  group->connect(rigidBody, 1, mass, 0);

  SharedPtr<System> system = new System("System", group);

  system->attach(SystemOutput::newDefaultSystemOutput("foucault.h5"));

  if (!system->init())
    return 1;

  system->simulate(24*60*60);

  return 0;
}
