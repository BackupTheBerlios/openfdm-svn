#include <iostream>

#include <OpenFDM/Group.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/PrismaticJoint.h>
#include <OpenFDM/RevoluteJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/System.h>

using namespace OpenFDM;

void addBodiesRecursive(unsigned numLevels, unsigned numBodies,
                        RigidBody* rigidBody, Group* group)
{
  if (!numLevels)
    return;

  for (unsigned i = 0; i < numBodies; ++i) {
    RevoluteJoint* revoluteJoint = new RevoluteJoint("RevoluteJoint");
    group->addChild(revoluteJoint);
    group->connect(revoluteJoint->getPort("link0"),
                   rigidBody->addLink("childLink"));
    
    RigidBody* rigidBody2 = new RigidBody("RigidBody");
    group->addChild(rigidBody2);
    group->connect(revoluteJoint->getPort("link1"),
                   rigidBody2->addLink("parentLink"));
    
    Mass* mass = new Mass("Mass", 1, InertiaMatrix(1, 0, 0, 1, 0, 1));
    group->addChild(mass);
    group->connect(mass->getPort("link"), rigidBody2->addLink("massLink"));
    
    addBodiesRecursive(numLevels - 1, numBodies, rigidBody2, group);
  }
}

bool testBodyTreeModel1(unsigned numLevels, unsigned numBodies)
{
  SharedPtr<Group> group = new Group("Group");

  MobileRootJoint* mechanicRootJoint = new MobileRootJoint("RootJoint");
  group->addChild(mechanicRootJoint);

  RigidBody* rigidBody = new RigidBody("RigidBody");
  group->addChild(rigidBody);
  group->connect(mechanicRootJoint->getPort("link"),
                 rigidBody->addLink("rootLink"));
  
  Mass* mass = new Mass("Mass", 1, InertiaMatrix(1, 0, 0, 1, 0, 1));
  group->addChild(mass);
  group->connect(mass->getPort("link"), rigidBody->addLink("massLink"));

  addBodiesRecursive(numLevels, numBodies, rigidBody, group);

  SharedPtr<System> system = new System("Closed kinematic Loop 1");
  system->setNode(group);

  if (!system->init()) {
    std::cerr << "Initialization of single body model failed!" << std::endl;
    return false;
  }
  return true;
}


bool testClosedKinematicLoop1()
{
  SharedPtr<Group> group = new Group("Group");

  MobileRootJoint* mechanicRootJoint = new MobileRootJoint("RootJoint");
  group->addChild(mechanicRootJoint);

  PrismaticJoint* prismaticJoint = new PrismaticJoint("PrismaticJoint");
  group->addChild(prismaticJoint);
  group->connect(mechanicRootJoint->getPort("link"),
                 prismaticJoint->getPort("link0"));
  group->connect(mechanicRootJoint->getPort("link"),
                 prismaticJoint->getPort("link1"));

  SharedPtr<System> system = new System("Closed kinematic Loop 1");
  system->setNode(group);

  if (system->init()) {
    std::cerr << "Detection of klosed kinematic loop failed!" << std::endl;
    return false;
  }
  return true;
}

bool testClosedKinematicLoop2()
{
  SharedPtr<Group> group = new Group("Group");

  MobileRootJoint* mechanicRootJoint = new MobileRootJoint("RootJoint");
  group->addChild(mechanicRootJoint);

  RigidBody* rigidBody = new RigidBody("RigidBody");
  group->addChild(rigidBody);
  group->connect(mechanicRootJoint->getPort("link"),
                 rigidBody->addLink("rootLink"));

  PrismaticJoint* prismaticJoint = new PrismaticJoint("PrismaticJoint");
  group->addChild(prismaticJoint);
  group->connect(rigidBody->addLink("jointLink0"),
                 prismaticJoint->getPort("link0"));
  group->connect(rigidBody->addLink("jointLink1"),
                 prismaticJoint->getPort("link1"));

  SharedPtr<System> system = new System("Closed kinematic Loop 1");
  system->setNode(group);

  if (system->init()) {
    std::cerr << "Detection of klosed kinematic loop failed!" << std::endl;
    return false;
  }
  return true;
}

int main()
{
  // Test a single body mass model
  if (!testBodyTreeModel1(0, 1))
    return EXIT_FAILURE;

  // Tests with multiple linear (numBodies = 1) and
  // tree (1 < numBodies) chained bodies
  for (unsigned numBodies = 1; numBodies < 4; ++numBodies) {
    for (unsigned numLevels = 1; numLevels < 4; ++numLevels) {
      if (!testBodyTreeModel1(numLevels, numBodies))
        return EXIT_FAILURE;
    }
  }

  // Some simple explicit kinematic loops
  if (!testClosedKinematicLoop1())
    return EXIT_FAILURE;
  if (!testClosedKinematicLoop2())
    return EXIT_FAILURE;

  std::cout << "PASSED" << std::endl;

  return EXIT_SUCCESS;
}
