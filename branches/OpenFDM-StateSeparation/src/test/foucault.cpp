#include <iostream>
#include <OpenFDM/Group.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/FixedRootJoint.h>
#include <OpenFDM/RotationalJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/ExternalInteract.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>
#include <OpenFDM/Time.h>
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

  SharedPtr<Group> group = new Group("Foucault");
  FixedRootJoint* fixedRootJoint = new FixedRootJoint("Root");
  fixedRootJoint->setRootPosition(planet->toCart(geodetic));
  fixedRootJoint->setRootOrientation(planet->getGeodHLOrientation(geodetic));
  group->addChild(fixedRootJoint);
  RotationalJoint* rotationalJoint1 = new RotationalJoint("Rotational Joint 1");
  Quaternion orientation = Quaternion::fromAngleAxisDeg(10, Vector3(0, 1, 0));
  rotationalJoint1->setInitialOrientation(orientation);
  group->addChild(rotationalJoint1);
  RigidBody* rigidBody1 = new RigidBody("Rigid Body 1");
  rigidBody1->addLink("externalInteractLink");
  group->addChild(rigidBody1);

  Mass* mass = new Mass("Mass", 28, InertiaMatrix(1e-2, 0, 0, 1e-2, 0, 1e-2));
  mass->setPosition(Vector3(0, 0, 67));
  group->addChild(mass);

  ExternalInteract* externalInteract = new ExternalInteract("ExternalInteract");
  externalInteract->setPosition(mass->getPosition());
  externalInteract->setEnableAllOutputs(true);
  group->addChild(externalInteract);

  group->connect(fixedRootJoint->getPort(0), rotationalJoint1->getPort(0));
  group->connect(rotationalJoint1->getPort(1), rigidBody1->getPort(0));
  group->connect(rigidBody1->getPort(1), mass->getPort(0));
  group->connect(rigidBody1->getPort("externalInteractLink"),
                 externalInteract->getPort("link"));

  SharedPtr<System> system = new System("System", group);

  system->getEnvironment()->setPlanet(planet);
  system->attach(SystemOutput::newDefaultSystemOutput("foucault"));

  OpenFDM::TimeCounter timeCounter;
  timeCounter.start();

  if (!system->init())
    return 1;

//   system->simulate(24*60*60);
//   system->simulate(60*60);
  system->simulate(60);

  timeCounter.stop();
  std::cout << "Execution time: " << timeCounter.getTime()
            << "s (" << double(timeCounter.getTime())/system->getTime() << ")"
            << std::endl;

  return 0;
}
