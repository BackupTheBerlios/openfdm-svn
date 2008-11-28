#include <OpenFDM/ConstModel.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/RevoluteJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/Sensor.h>
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

  MobileRootJoint* mobileRootJoint = new MobileRootJoint("Root Joint");
  Group::NodeId rootJointId = group->addChild(mobileRootJoint);
  RigidBody* rigidBody = new RigidBody("Rigid Body");
  rigidBody->addLink("sensorLink");
  Group::NodeId rigidBodyId = group->addChild(rigidBody);
  Mass* mass = new Mass("Mass", 1, InertiaMatrix(1, 0, 0, 1, 0, 1));
  Group::NodeId massId = group->addChild(mass);

  Sensor* sensor = new Sensor("Sensor");
  sensor->setPosition(mass->getPosition());
  sensor->setEnableAll(true);
  Group::NodeId sensorId = group->addChild(sensor);

  group->connect(rootJointId, "link", rigidBodyId, "link0");
  group->connect(rigidBodyId, "link1", massId, "link");
  group->connect(rigidBodyId, "sensorLink", sensorId, "link");

  return group.release();
}

Node* buildSimpleMechanicExample2()
{
  SharedPtr<Group> group = new Group("G");

  MobileRootJoint* mobileRootJoint = new MobileRootJoint("Root Joint");
  Group::NodeId rootJointId = group->addChild(mobileRootJoint);

  RigidBody *rigidBody = new RigidBody("Rigid Body");
  rigidBody->addLink("link2");
  rigidBody->addLink("sensorLink");
  Group::NodeId rigidBodyId = group->addChild(rigidBody);
  InertiaMatrix inertia(1, 0, 0, 1, 0, 1);
  Mass* mass = new Mass("Mass", 1, inertia);
  Group::NodeId massId = group->addChild(mass);
  RevoluteJoint* revoluteJoint = new RevoluteJoint("Revolute Joint");
  revoluteJoint->setEnableExternalForce(true);
  Group::NodeId revoluteId = group->addChild(revoluteJoint);
  RigidBody *rigidBody2 = new RigidBody("Rigid Body 2");
  rigidBody2->addLink("sensorLink");
  Group::NodeId rigidBody2Id = group->addChild(rigidBody2);
  Mass* mass2 = new Mass("Mass 2", 1, inertia);
  Group::NodeId mass2Id = group->addChild(mass2);

  Sensor* sensor = new Sensor("Sensor");
  sensor->setPosition(mass->getPosition());
  sensor->setEnableAll(true);
  Group::NodeId sensorId = group->addChild(sensor);

  Sensor* sensor2 = new Sensor("Sensor 2");
  sensor2->setPosition(mass2->getPosition());
  sensor2->setEnableAll(true);
  Group::NodeId sensorId2 = group->addChild(sensor2);

  group->connect(rootJointId, "link", rigidBodyId, "link0");
  group->connect(rigidBodyId, "link1", massId, "link");
  group->connect(rigidBodyId, "link2", revoluteId, "link0");
  group->connect(revoluteId, "link1", rigidBody2Id, "link0");
  group->connect(rigidBody2Id, "link1", mass2Id, "link");
  group->connect(rigidBodyId, "sensorLink", sensorId, "link");
  group->connect(rigidBody2Id, "sensorLink", sensorId2, "link");

  ConstModel* jointForce = new ConstModel("Joint Force", 1);
  Group::NodeId jointForceId = group->addChild(jointForce);

  group->connect(jointForceId, "output", revoluteId, "force");

  return group.release();
}

int main()
{
//   SharedPtr<System> system = new System("System", buildSimpleMechanicExample());
  SharedPtr<System> system = new System("System", buildSimpleMechanicExample2());

  system->attach(SystemOutput::newDefaultSystemOutput("system.h5"));

  if (!system->init())
    return 1;

  system->simulate(10);

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

