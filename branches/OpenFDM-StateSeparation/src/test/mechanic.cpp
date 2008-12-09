#include <OpenFDM/ConstModel.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/InternalSensor.h>
#include <OpenFDM/LinearSpringDamper.h>
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
  group->addChild(mobileRootJoint);
  RigidBody* rigidBody = new RigidBody("Rigid Body");
  rigidBody->addLink("sensorLink");
  group->addChild(rigidBody);
  Mass* mass = new Mass("Mass", 1, InertiaMatrix(1, 0, 0, 1, 0, 1));
  group->addChild(mass);

  Sensor* sensor = new Sensor("Sensor");
  sensor->setPosition(mass->getPosition());
  sensor->setEnableAll(true);
  group->addChild(sensor);

  group->connect(mobileRootJoint->getPort("link"), rigidBody->getPort("link0"));
  group->connect(rigidBody->getPort("link1"), mass->getPort("link"));
  group->connect(rigidBody->getPort("sensorLink"), sensor->getPort("link"));

  return group.release();
}

Node* buildSimpleMechanicExample2()
{
  SharedPtr<Group> group = new Group("G");

  MobileRootJoint* mobileRootJoint = new MobileRootJoint("Root Joint");
  group->addChild(mobileRootJoint);

  RigidBody *rigidBody = new RigidBody("Rigid Body");
  rigidBody->addLink("link2");
  rigidBody->addLink("sensorLink");
  rigidBody->addLink("internalSensorLink");
  rigidBody->addLink("internalSensorLink2");
  group->addChild(rigidBody);
  InertiaMatrix inertia(1, 0, 0, 1, 0, 1);
  Mass* mass = new Mass("Mass", 1, inertia);
  group->addChild(mass);
  RevoluteJoint* revoluteJoint = new RevoluteJoint("Revolute Joint");
  revoluteJoint->setEnableExternalForce(true);
  group->addChild(revoluteJoint);
  RigidBody *rigidBody2 = new RigidBody("Rigid Body 2");
  rigidBody2->addLink("sensorLink");
  rigidBody2->addLink("internalSensorLink");
  rigidBody2->addLink("internalSensorLink2");
  group->addChild(rigidBody2);
  Mass* mass2 = new Mass("Mass 2", 1, inertia);
  group->addChild(mass2);

  Sensor* sensor = new Sensor("Sensor");
  sensor->setPosition(mass->getPosition());
  sensor->setEnableAll(true);
  group->addChild(sensor);

  Sensor* sensor2 = new Sensor("Sensor 2");
  sensor2->setPosition(mass2->getPosition());
  sensor2->setEnableAll(true);
  group->addChild(sensor2);

  group->connect(mobileRootJoint->getPort("link"), rigidBody->getPort("link0"));
  group->connect(rigidBody->getPort("link1"), mass->getPort("link"));
  group->connect(rigidBody->getPort("link2"), revoluteJoint->getPort("link0"));
  group->connect(revoluteJoint->getPort("link1"), rigidBody2->getPort("link0"));
  group->connect(rigidBody2->getPort("link1"), mass2->getPort("link"));
  group->connect(rigidBody->getPort("sensorLink"), sensor->getPort("link"));
  group->connect(rigidBody2->getPort("sensorLink"), sensor2->getPort("link"));

  ConstModel* jointForce = new ConstModel("Joint Force", 1);
  group->addChild(jointForce);

  group->connect(jointForce->getPort("output"),
                 revoluteJoint->getPort("force"));

  InternalSensor* internalSensor = new InternalSensor("Internal Sensor");
  internalSensor->setPosition0(Vector3(0, 0, 1));
  internalSensor->setPosition1(Vector3(0, 0, 0.8));
  internalSensor->setEnableAll(true);
  internalSensor->setEnableForce(true);
  group->addChild(internalSensor);
  group->connect(internalSensor->getPort("link0"),
                 rigidBody->getPort("internalSensorLink"));
  group->connect(internalSensor->getPort("link1"),
                 rigidBody2->getPort("internalSensorLink"));


  InternalSensor* internalSensor2 = new InternalSensor("Internal Sensor 2");
  internalSensor2->setPosition0(Vector3(0, 0, 0.8));
  internalSensor2->setPosition1(Vector3(0, 0, 1));
  internalSensor2->setEnableAll(true);
  group->addChild(internalSensor2);
  group->connect(internalSensor2->getPort("link1"),
                 rigidBody->getPort("internalSensorLink2"));
  group->connect(internalSensor2->getPort("link0"),
                 rigidBody2->getPort("internalSensorLink2"));

  LinearSpringDamper* damper = new LinearSpringDamper("LinearSpringDamper");
  damper->setSpringConstant(0.5);
  damper->setDamperConstant(1);
  group->addChild(damper);
  group->connect(damper->getPort("velocity"),
                 internalSensor->getPort("velocity"));
  group->connect(damper->getPort("position"),
                 internalSensor->getPort("distance"));
  group->connect(damper->getPort("force"),
                 internalSensor->getPort("force"));

  return group.release();
}

int main()
{
//   SharedPtr<System> system = new System("System", buildSimpleMechanicExample());
  SharedPtr<System> system = new System("System", buildSimpleMechanicExample2());

  system->attach(SystemOutput::newDefaultSystemOutput("mechanic"));

  if (!system->init())
    return 1;

  system->simulate(10);

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

