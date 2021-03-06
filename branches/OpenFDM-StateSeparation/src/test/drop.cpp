#include <iostream>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/ExternalInteract.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>
#include <OpenFDM/Time.h>

using namespace OpenFDM;

Node* buildDrop()
{
  // A simple free falling mass.
  SharedPtr<Group> group = new Group("Group");

  MobileRootJoint* mobileRootJoint = new MobileRootJoint("Root Joint");
  mobileRootJoint->setInitialPosition(Vector3(0, 0, -10000));
  mobileRootJoint->setInitialAngularVelocity(Vector3(1, 1, 1));
  group->addChild(mobileRootJoint);

  RigidBody* rigidBody = new RigidBody("Rigid Body");
  rigidBody->addLink("externalInteractLink");
  group->addChild(rigidBody);

  Mass* mass = new Mass("Mass", 1, InertiaMatrix(1, 0, 0, 1, 0, 1));
  group->addChild(mass);

  ExternalInteract* externalInteract = new ExternalInteract("ExternalInteract");
  externalInteract->setPosition(mass->getPosition());
  externalInteract->setEnableAllOutputs(true);
  group->addChild(externalInteract);

  group->connect(mobileRootJoint->getPort("link"),
                 rigidBody->getPort("link0"));
  group->connect(rigidBody->getPort("link1"),
                 mass->getPort("link"));
  group->connect(rigidBody->getPort("externalInteractLink"),
                 externalInteract->getPort("link"));

  return group.release();
}

int main()
{
  SharedPtr<System> system = new System("System", buildDrop());

  system->attach(SystemOutput::newDefaultSystemOutput("drop"));

  OpenFDM::TimeCounter timeCounter;
  timeCounter.start();

  if (!system->init())
    return 1;

  system->simulate(45);

  timeCounter.stop();
  std::cout << "Execution time: " << timeCounter.getTime()
            << "s (" << double(timeCounter.getTime())/system->getTime() << ")"
            << std::endl;

  return 0;
}

