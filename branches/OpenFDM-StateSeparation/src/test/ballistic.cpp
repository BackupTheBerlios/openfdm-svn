#include <OpenFDM/ConstModel.h>
#include <OpenFDM/DynamicPressure.h>
#include <OpenFDM/ExternalInteract.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/MachNumber.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/MatrixConcat.h>
#include <OpenFDM/MatrixSplit.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>
#include <OpenFDM/UnaryFunction.h>
#include <OpenFDM/WindAxis.h>
#include <OpenFDM/WindAxisForce.h>

using namespace OpenFDM;

Node* buildBallistic()
{
  // A simple free falling mass.
  SharedPtr<Group> group = new Group("Group");

  MobileRootJoint* mobileRootJoint = new MobileRootJoint("Root Joint");
  mobileRootJoint->setInitialPosition(Vector3(0, 0, -200));
  mobileRootJoint->setInitialOrientation(Quaternion::fromEulerSeq(1, 45*deg2rad));
  mobileRootJoint->setInitialLinearVelocity(50*Vector3(1, 0, 0));
  mobileRootJoint->setInitialAngularVelocity(Vector3(180*deg2rad, 0, 0));
  group->addChild(mobileRootJoint);

  RigidBody* rigidBody = new RigidBody("Rigid Body");
  rigidBody->addLink("externalInteractLink");
  group->addChild(rigidBody);

  real_type m = 0.01;
  real_type length = 1;
  real_type radius = 0.01;
  InertiaMatrix I = InertiaMatrix::cylinderInertia(m, length, radius);
  Mass* mass = new Mass("Mass", m, I);
  group->addChild(mass);

  ExternalInteract* externalInteract = new ExternalInteract("ExternalInteract");
  externalInteract->setPosition(mass->getPosition() - Vector3(0.3, 0, 0));
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

  UnaryFunction* alphaAbs = new UnaryFunction("Alpha Abs", UnaryFunction::Abs);
  group->addChild(alphaAbs);

  group->connect(alphaAbs->getPort("input"),
                 windAxis->getPort("alpha"));

  UnaryFunction* betaAbs = new UnaryFunction("Beta Abs", UnaryFunction::Abs);
  group->addChild(betaAbs);

  group->connect(betaAbs->getPort("input"),
                 windAxis->getPort("beta"));

  ConstModel* dragAtZeroAlpha = new ConstModel("Drag at zero Alpha", 1);
  group->addChild(dragAtZeroAlpha);

  Summer* dragFactor = new Summer("Drag Factor");
  dragFactor->setNumSummands(3);
  group->addChild(dragFactor);

  group->connect(dragFactor->getPort("input0"),
                 dragAtZeroAlpha->getPort("output"));
  group->connect(dragFactor->getPort("input1"),
                 alphaAbs->getPort("output"));
  group->connect(dragFactor->getPort("input2"),
                 betaAbs->getPort("output"));

  Gain* dragCoeficient = new Gain("Drag Coeficient", 0.0001);
  group->addChild(dragCoeficient);

  group->connect(dragCoeficient->getPort("input"),
                 dragFactor->getPort("output"));

  Product* drag = new Product("Drag");
  drag->setNumFactors(2);
  group->addChild(drag);

  group->connect(drag->getPort("input0"),
                 dynamicPressure->getPort("dynamicPressure"));
  group->connect(drag->getPort("input1"),
                 dragCoeficient->getPort("output"));


  ConstModel* zeroConst = new ConstModel("ConstModel 0");
  group->addChild(zeroConst);

  group->connect(drag->getPort("output"),
                 windAxisForce->getPort("drag"));
  group->connect(zeroConst->getPort("output"),
                 windAxisForce->getPort("side"));
  group->connect(zeroConst->getPort("output"),
                 windAxisForce->getPort("lift"));

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
