#include <OpenFDM/Group.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/FixedRootJoint.h>
#include <OpenFDM/RotationalJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/Sensor.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>
#include <OpenFDM/WGS84Planet.h>

using namespace OpenFDM;

int main()
{
  SharedPtr<WGS84Planet> planet = new WGS84Planet;

  // Model of the paris pendulum or foucault pendulum to test coriolis effects.

  // N 48deg 50.781 E 2deg 20.709
  // 28 kg, 67 meters pendulum
  // Deviation 11deg/h
  // 16,5s cycle time(?back and forth?)

  // Position of the pendulum
  Geodetic geodetic(rad2deg*(48 + 50.781/60), rad2deg*(2 + 20.709/60), 0);
  // Test the direction of the velocity vector projected to the ground plane

  // FIXME, need usable environment stuff like gravity first ...
  SharedPtr<Group> group = new Group("Foucault");
  FixedRootJoint* fixedRootJoint = new FixedRootJoint("Root");
  fixedRootJoint->setPosition(planet->toCart(geodetic));
  fixedRootJoint->setOrientation(planet->getGeodHLOrientation(geodetic));
  group->addChild(fixedRootJoint);
  RotationalJoint* rotationalJoint1 = new RotationalJoint("Rotational Joint 1");
  group->addChild(rotationalJoint1);
  RigidBody* rigidBody1 = new RigidBody("Rigid Body 1");
  rigidBody1->addLink("sensorLink");
  group->addChild(rigidBody1);

  Mass* mass = new Mass("Mass", 28, InertiaMatrix(1, 0, 0, 1, 0, 1));
  mass->setPosition(Vector3(3, 0, 67));
  group->addChild(mass);

  Sensor* sensor = new Sensor("Sensor");
  sensor->setPosition(mass->getPosition());
  sensor->setEnableAllOutputs(true);
  group->addChild(sensor);

  group->connect(fixedRootJoint->getPort(0), rotationalJoint1->getPort(0));
  group->connect(rotationalJoint1->getPort(1), rigidBody1->getPort(0));
  group->connect(rigidBody1->getPort(1), mass->getPort(0));
  group->connect(rigidBody1->getPort("sensorLink"), sensor->getPort("link"));

  SharedPtr<System> system = new System("System", group);

  system->getEnvironment()->setPlanet(planet);
  system->attach(SystemOutput::newDefaultSystemOutput("foucault"));

  if (!system->init())
    return 1;

//   system->simulate(24*60*60);
//   system->simulate(60*60);
  system->simulate(60);

  return 0;
}
