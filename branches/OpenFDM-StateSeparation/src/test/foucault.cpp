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
  FixedRootJoint* fixedRootJoint = new FixedRootJoint("Root");
  fixedRootJoint->setPosition(Vector3(0, 0, 1));
  Group::NodeId fixedRootJointId = group->addChild(fixedRootJoint);
  RotationalJoint* rotationalJoint1 = new RotationalJoint("Rotational Joint 1");
  Group::NodeId rotationalJoint1Id = group->addChild(rotationalJoint1);
  RigidBody* rigidBody1 = new RigidBody("Rigid Body 1");
  rigidBody1->addLink("sensorLink");
  Group::NodeId rigidBody1Id = group->addChild(rigidBody1);

  Mass* mass = new Mass("Mass", 1, InertiaMatrix(1, 0, 0, 1, 0, 1));
  mass->setPosition(Vector3(1, 1, 0));
  Group::NodeId massId = group->addChild(mass);

  Sensor* sensor = new Sensor("Sensor");
  sensor->setPosition(mass->getPosition());
  sensor->setEnablePosition(true);
  sensor->setEnableOrientation(true);
  sensor->setEnableEulerAngles(true);
  sensor->setEnableLinearVelocity(true);
  sensor->setEnableAngularVelocity(true);
  sensor->setEnableCentrifugalAcceleration(true);
  Group::NodeId sensorId = group->addChild(sensor);

  group->connect(fixedRootJointId, 0, rotationalJoint1Id, 0);
  group->connect(rotationalJoint1Id, 1, rigidBody1Id, 0);
  group->connect(rigidBody1Id, 1, massId, 0);
  group->connect(rigidBody1Id, "sensorLink", sensorId, "link");

  SharedPtr<System> system = new System("System", group);

  system->attach(SystemOutput::newDefaultSystemOutput("foucault.h5"));

  if (!system->init())
    return 1;

//   system->simulate(24*60*60);
  system->simulate(60);

  return 0;
}
