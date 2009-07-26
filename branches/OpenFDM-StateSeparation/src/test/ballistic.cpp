#include <OpenFDM/ConstModel.h>
#include <OpenFDM/Gain.h>
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

#include <OpenFDM/DynamicPressure.h>
#include <OpenFDM/MachNumber.h>
#include <OpenFDM/WindAxis.h>
#include <OpenFDM/WindAxisForce.h>

using namespace OpenFDM;

Node* buildBallistic()
{
  // A simple free falling mass.
  SharedPtr<Group> group = new Group("Group");

  MobileRootJoint* mobileRootJoint = new MobileRootJoint("Root Joint");
  mobileRootJoint->setInitialLinearVelocity(50/sqrt(2)*Vector3(1, 0, -1));
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

  DynamicPressure* dynamicPressure = new DynamicPressure("DynamicPressure");
  group->addChild(dynamicPressure);

  group->connect(externalInteract->getPort("bodyWindVelocity"),
                 dynamicPressure->getPort("velocity"));
  group->connect(externalInteract->getPort("density"),
                 dynamicPressure->getPort("density"));


  MachNumber* machNumber = new MachNumber("MachNumber");
  group->addChild(machNumber);

  group->connect(externalInteract->getPort("bodyWindVelocity"),
                 machNumber->getPort("velocity"));
  group->connect(externalInteract->getPort("soundSpeed"),
                 machNumber->getPort("soundSpeed"));


  WindAxis* windAxis = new WindAxis("WindAxis");
  group->addChild(windAxis);

  group->connect(externalInteract->getPort("bodyWindVelocity"),
                 windAxis->getPort("bodyVelocity"));


  WindAxisForce* windAxisForce = new WindAxisForce("WindAxisForce");
  group->addChild(windAxisForce);

  group->connect(windAxis->getPort("alpha"),
                 windAxisForce->getPort("alpha"));
  group->connect(windAxis->getPort("beta"),
                 windAxisForce->getPort("beta"));

  externalInteract->setEnableBodyForce(true);
  group->connect(windAxisForce->getPort("bodyForce"),
                 externalInteract->getPort("bodyForce"));

  ConstModel* zeroConst = new ConstModel("ConstModel 0");
  group->addChild(zeroConst);

  group->connect(zeroConst->getPort("output"),
                 windAxisForce->getPort("side"));
  group->connect(zeroConst->getPort("output"),
                 windAxisForce->getPort("lift"));

  Gain* dragCoeficient = new Gain("Drag Coeficient");
  dragCoeficient->setGain(0.01);
  group->addChild(dragCoeficient);

  group->connect(dragCoeficient->getPort("input"),
                 dynamicPressure->getPort("dynamicPressure"));

  group->connect(dragCoeficient->getPort("output"),
                 windAxisForce->getPort("drag"));

  return group.release();
}

int main()
{
  SharedPtr<System> system = new System("System", buildBallistic());

  system->attach(SystemOutput::newDefaultSystemOutput("ballistic"));

  if (!system->init())
    return 1;

  system->simulate(10);

  return 0;
}
