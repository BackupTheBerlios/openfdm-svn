#include <OpenFDM/Group.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/FixedRootJoint.h>
#include <OpenFDM/RotationalJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/Sensor.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>
// #include <OpenFDM/Planet.h>

using namespace OpenFDM;

int main()
{
  // Model of the paris pendulum or foucault pendulum to test coriolis effects.

  // Position of the pendulum
//   Geocentric geocentric()
  // Test the direction of the velocity vector projected to the ground plane

  // FIXME, need usable environment stuff like gravity first ...
  SharedPtr<Group> group = new Group("Foucault");
  FixedRootJoint* fixedRoot = new FixedRootJoint("Root");
  fixedRoot->setPosition(Vector3(0, 0, -10));
  Group::NodeId root = group->addChild(fixedRoot);
  RotationalJoint* rotationalJoint = new RotationalJoint("Rotational");
  Group::NodeId rotational = group->addChild(rotationalJoint);

  RigidBody* rigidBodyNode = new RigidBody("Rigid Body");
  rigidBodyNode->addLink("sensorLink");
  Group::NodeId rigidBody = group->addChild(rigidBodyNode);
  Mass* massModel = new Mass("Mass", 1, InertiaMatrix(1, 0, 0, 1, 0, 1));
  massModel->setPosition(Vector3(1, 0, 0));
  Group::NodeId mass = group->addChild(massModel);

  Sensor* sensorModel = new Sensor("Sensor");
  sensorModel->setPosition(massModel->getPosition());
  sensorModel->setEnablePosition(true);
  sensorModel->setEnableOrientation(true);
  sensorModel->setEnableLinearVelocity(true);
  sensorModel->setEnableAngularVelocity(true);
  sensorModel->setEnableCentrifugalAcceleration(true);
  Group::NodeId sensor = group->addChild(sensorModel);

  group->connect(root, 0, rotational, 0);
  group->connect(rotational, 1, rigidBody, 0);
  group->connect(rigidBody, 1, mass, 0);
  group->connect(rigidBody, "sensorLink", sensor, "link");

  SharedPtr<System> system = new System("System", group);

  system->attach(SystemOutput::newDefaultSystemOutput("foucault.h5"));

  if (!system->init())
    return 1;

//   system->simulate(24*60*60);
  system->simulate(60);

  return 0;
}
