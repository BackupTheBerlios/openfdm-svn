#include <OpenFDM/ConstModel.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/InternalInteract.h>
#include <OpenFDM/LinearSpringDamper.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/RevoluteJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/ExternalInteract.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>

using namespace OpenFDM;

/// sensible test cases:
/// drop: gravity
/// throw: just this test with a different start condition
/// harmonic oszilator: compare with 2nd order linear system
/// arrow: see if the tip stays in front
/// satellit: coriolis
/// paris pendulum: coriolis

Node* buildSimpleMechanicExample()
{
  SharedPtr<Group> group = new Group("G");

  MobileRootJoint* mobileRootJoint = new MobileRootJoint("Root Joint");
  group->addChild(mobileRootJoint);

  RigidBody *rigidBody = new RigidBody("Rigid Body");
  rigidBody->addLink("link2");
  rigidBody->addLink("externalInteractLink");
  rigidBody->addLink("internalInteractLink");
  rigidBody->addLink("internalInteractLink2");
  group->addChild(rigidBody);
  InertiaMatrix inertia(1, 0, 0, 1, 0, 1);
  Mass* mass = new Mass("Mass", 1, inertia);
  group->addChild(mass);
  RevoluteJoint* revoluteJoint = new RevoluteJoint("Revolute Joint");
  revoluteJoint->setEnableExternalForce(true);
  group->addChild(revoluteJoint);
  RigidBody *rigidBody2 = new RigidBody("Rigid Body 2");
  rigidBody2->addLink("externalInteractLink");
  rigidBody2->addLink("internalInteractLink");
  rigidBody2->addLink("internalInteractLink2");
  group->addChild(rigidBody2);
  Mass* mass2 = new Mass("Mass 2", 1, inertia);
  group->addChild(mass2);

  ExternalInteract* externalInteract = new ExternalInteract("ExternalInteract");
  externalInteract->setPosition(mass->getPosition());
  externalInteract->setEnableAllOutputs(true);
  group->addChild(externalInteract);

  ExternalInteract* externalInteract2 = new ExternalInteract("ExternalInteract 2");
  externalInteract2->setPosition(mass2->getPosition());
  externalInteract2->setEnableAllOutputs(true);
  group->addChild(externalInteract2);

  group->connect(mobileRootJoint->getPort("link"), rigidBody->getPort("link0"));
  group->connect(rigidBody->getPort("link1"), mass->getPort("link"));
  group->connect(rigidBody->getPort("link2"), revoluteJoint->getPort("link0"));
  group->connect(revoluteJoint->getPort("link1"), rigidBody2->getPort("link0"));
  group->connect(rigidBody2->getPort("link1"), mass2->getPort("link"));
  group->connect(rigidBody->getPort("externalInteractLink"), externalInteract->getPort("link"));
  group->connect(rigidBody2->getPort("externalInteractLink"), externalInteract2->getPort("link"));

  ConstModel* jointForce = new ConstModel("Joint Force", 1);
  group->addChild(jointForce);

  group->connect(jointForce->getPort("output"),
                 revoluteJoint->getPort("force"));

  InternalInteract* internalInteract = new InternalInteract("InternalInteract");
  internalInteract->setPosition0(Vector3(0, 0, 1));
  internalInteract->setPosition1(Vector3(0, 0, 0.8));
  internalInteract->setEnableAllOutputs(true);
  internalInteract->setEnableForce(true);
  group->addChild(internalInteract);
  group->connect(internalInteract->getPort("link0"),
                 rigidBody->getPort("internalInteractLink"));
  group->connect(internalInteract->getPort("link1"),
                 rigidBody2->getPort("internalInteractLink"));


  InternalInteract* internalInteract2 = new InternalInteract("InternalInteract2");
  internalInteract2->setPosition0(Vector3(0, 0, 0.8));
  internalInteract2->setPosition1(Vector3(0, 0, 1));
  internalInteract2->setEnableAllOutputs(true);
  group->addChild(internalInteract2);
  group->connect(internalInteract2->getPort("link1"),
                 rigidBody->getPort("internalInteractLink2"));
  group->connect(internalInteract2->getPort("link0"),
                 rigidBody2->getPort("internalInteractLink2"));

  LinearSpringDamper* damper = new LinearSpringDamper("LinearSpringDamper");
  damper->setSpringConstant(0.5);
  damper->setDamperConstant(1);
  group->addChild(damper);
  group->connect(damper->getPort("velocity"),
                 internalInteract->getPort("velocity"));
  group->connect(damper->getPort("position"),
                 internalInteract->getPort("distance"));
  group->connect(damper->getPort("force"),
                 internalInteract->getPort("force"));

  return group.release();
}

int main()
{
  SharedPtr<System> system = new System("System", buildSimpleMechanicExample());

  system->attach(SystemOutput::newDefaultSystemOutput("mechanic"));

  if (!system->init())
    return 1;

  system->simulate(10);

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

