/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include <OpenFDM/ConstModel.h>
#include <OpenFDM/FixedRootJoint.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/LinearSpringDamper.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/GroupInput.h>
#include <OpenFDM/GroupMechanicLink.h>
#include <OpenFDM/GroupOutput.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/PrismaticJoint.h>
#include <OpenFDM/RevoluteActuator.h>
#include <OpenFDM/RevoluteJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/SimulationTime.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/System.h>
#include <OpenFDM/SystemOutput.h>
#include <OpenFDM/UnaryFunction.h>
#include <OpenFDM/Pacejka89.h>
#include <OpenFDM/Pacejka94.h>

using namespace OpenFDM;

class MovingGround : public AbstractGround {
public:
  MovingGround(const Vector3& linearVelocity) : mVelocity(linearVelocity)
  { }
  virtual GroundValues getGroundPlane(const Environment&, const real_type&,
                                      const Vector3&) const
  {
    return GroundValues(Plane(Vector3(0, 0, -1), Vector3::zeros()),
                        Vector6(Vector3::zeros(), mVelocity),
                        real_type(1));
  }
private:
  Vector3 mVelocity;
};

// Build up a wheel to put on the testrig.
// When we have proper serialization, load that from a file.
Node*
createWheel()
{
  SharedPtr<Group> group = new Group("Wheel");

  GroupMechanicLink* hubLink = new GroupMechanicLink("Hub Link");
  group->addChild(hubLink);

  RigidBody* rimAndTire = new RigidBody("Rim And Tire");
  group->addChild(rimAndTire);
  group->connect(hubLink->getPort("link"), rimAndTire->getPort("link0"));

  Mass* tireAndRimMass = new Mass("Rim And Tire Mass");
  tireAndRimMass->setMass(9);
  // Realistic ...
//   tireAndRimMass->setInertia(InertiaMatrix(0.4, 0, 0, 1, 0, 0.4));
  // For pac 2002
//   tireAndRimMass->setInertia(InertiaMatrix(0.5, 0, 0, 2, 0, 0.5));
  // For pac 89
  tireAndRimMass->setInertia(InertiaMatrix(0.4, 0, 0, 1, 0, 0.4));
  group->addChild(tireAndRimMass);
  group->connect(rimAndTire->getPort("link1"), tireAndRimMass->getPort("link"));
  
//   Pacejka89* pacejkaTire = new Pacejka89("PacejkaTire");
  Pacejka94* pacejkaTire = new Pacejka94("PacejkaTire");
//   Pacejka2002* pacejkaTire = new Pacejka2002("PacejkaTire");
  pacejkaTire->setWheelRadius(0.313);
  pacejkaTire->setSpringConstant(2e5);
  pacejkaTire->setDampingConstant(sqrt(pacejkaTire->getSpringConstant()));
  group->addChild(pacejkaTire);
  rimAndTire->addLink("link2");
  group->connect(rimAndTire->getPort("link2"), pacejkaTire->getPort("link"));

  return group.release();
}

// Build up the tiretestrig system.
// When we have proper serialization, load that from a file.
Node*
createTireTestrig(Node* wheel)
{
  SharedPtr<Group> group = new Group("Tire Testrig");

  // First build up the mechanical system
  FixedRootJoint* fixedRootJoint = new FixedRootJoint("Fixed Root Joint");
  group->addChild(fixedRootJoint);
  fixedRootJoint->setRootPosition(Vector3(0, 0, -1));

//   PrismaticActuator* prismaticActuator
//     = new PrismaticActuator("Normal Force Actuator");
//   prismaticActuator->setAxis(Vector3::unit(2));
//   prismaticActuator->setMaxVel(Vector3::unit(2));
//   group->addChild(prismaticActuator);

  PrismaticJoint* prismaticJoint = new PrismaticJoint("Normal Force joint");
  prismaticJoint->setAxis(Vector3::unit(2));
  prismaticJoint->setEnableExternalForce(true);
  group->addChild(prismaticJoint);

  Summer* normalForceSum = new Summer("Normal Force Sum");
  normalForceSum->setNumSummands(2);
  group->addChild(normalForceSum);
  group->connect(prismaticJoint->getPort("force"),
                 normalForceSum->getPort("output"));

  ConstModel* normalForce = new ConstModel("Normal Force");
  group->addChild(normalForce);
  group->connect(normalForceSum->getPort("input0"),
                 normalForce->getPort("output"));
 
  LinearSpringDamper* strutDamper = new LinearSpringDamper("Strut Damper");
  strutDamper->setSpringConstant(0);
  strutDamper->setDamperConstant(1000);
  group->addChild(strutDamper);
  group->connect(normalForceSum->getPort("input1"),
                 strutDamper->getPort("force"));
  group->connect(strutDamper->getPort("velocity"),
                 prismaticJoint->getPort("velocity"));
  group->connect(strutDamper->getPort("position"),
                 prismaticJoint->getPort("position"));

  RigidBody* rootMount = new RigidBody("Root Mount");
  group->addChild(rootMount);
  group->connect(rootMount->getPort("link0"), fixedRootJoint->getPort("link"));
  group->connect(rootMount->getPort("link1"), prismaticJoint->getPort("link0"));

  RevoluteActuator* camberActuator = new RevoluteActuator("Camber Actuator");
  camberActuator->setAxis(Vector3(1, 0, 0));
  group->addChild(camberActuator);

  ConstModel* camberAngleModel = new ConstModel("Camber Angle");
  group->addChild(camberAngleModel);
  group->connect(camberActuator->getPort("input"),
                 camberAngleModel->getPort("output"));

  RigidBody* normalForceStrut = new RigidBody("Normal Force Strut");
  group->addChild(normalForceStrut);
  group->connect(normalForceStrut->getPort("link0"),
                 prismaticJoint->getPort("link1"));
  group->connect(normalForceStrut->getPort("link1"),
                 camberActuator->getPort("link0"));

 
  RevoluteActuator* sideActuator = new RevoluteActuator("Sideslip Actuator");
  sideActuator->setAxis(Vector3(0, 0, 1));
  sideActuator->setVelGain(100);
  sideActuator->setVelDotGain(100);
  group->addChild(sideActuator);

  ConstModel* sideslipAngle = new ConstModel("Sideslip Angle");
  group->addChild(sideslipAngle);
  group->connect(sideActuator->getPort("input"),
                 sideslipAngle->getPort("output"));

  RigidBody* camberStrut = new RigidBody("Camber Strut");
  group->addChild(camberStrut);
  group->connect(camberStrut->getPort("link0"),
                 camberActuator->getPort("link1"));
  group->connect(camberStrut->getPort("link1"), sideActuator->getPort("link0"));


  RevoluteJoint* hubJoint = new RevoluteJoint("Hub Joint");
  hubJoint->setAxis(Vector3(0, 1, 0));
  group->addChild(hubJoint);

  RigidBody* hubStrut = new RigidBody("Hub Strut");
  group->addChild(hubStrut);
  group->connect(hubStrut->getPort("link0"), sideActuator->getPort("link1"));
  group->connect(hubStrut->getPort("link1"), hubJoint->getPort("link0"));

  group->addChild(wheel);
  group->connect(hubJoint->getPort("link1"), wheel->getPort("link"));

  return group.release();
}

int
main(int argc, char *argv[])
{
  // Simulate a simple side sweep with
  //   sweep speed alphaSpeed
  //   sweep range alphaRange
  //   at camber Angle
  //   and velocity
  real_type camberAngle = deg2rad*0;
  real_type alphaRange = deg2rad*20;
  real_type alphaSpeed = deg2rad*10;
  real_type velocity = 10;
  real_type normalForce = 4000;

  SharedPtr<Node> wheel = createWheel();
  SharedPtr<Node> testrig = createTireTestrig(wheel);
  SharedPtr<System> system = new System("Tire Testrig", testrig);

  // set the moving ground
  MovingGround* movingGround = new MovingGround(Vector3(velocity, 0, 0));
  system->getEnvironment()->setGround(movingGround);

  // Get the model nodes that are important to drive the testrig
  SharedPtr<Node> sideslipNode;
  sideslipNode = system->getNode("Tire Testrig/Sideslip Angle");
  SharedPtr<Node> sideslipActuator;
  sideslipActuator = system->getNode("Tire Testrig/Sideslip Actuator");
  SharedPtr<Node> camberNode;
  camberNode = system->getNode("Tire Testrig/Camber Angle");
  SharedPtr<Node> normalForceNode;
  normalForceNode = system->getNode("Tire Testrig/Normal Force");

  // Set parametrs for the measurment
  sideslipActuator->setPropertyValue("MaxVel", Variant(real_type(alphaSpeed)));
  camberNode->setPropertyValue("Value", Variant(Matrix(camberAngle)));
  normalForceNode->setPropertyValue("Value", Variant(Matrix(normalForce)));
  sideslipNode->setPropertyValue("Value", Variant(Matrix(0)));

  if (!system->init())
    return EXIT_FAILURE;

  system->simulate(2);
  system->attach(SystemOutput::newDefaultSystemOutput("tiretestrig"));
  
  sideslipNode->setPropertyValue("Value", Variant(Matrix(alphaRange)));
  system->simulate(system->getTime() + alphaRange/alphaSpeed);
  
  sideslipNode->setPropertyValue("Value", Variant(Matrix(-alphaRange)));
  system->simulate(system->getTime() + 2*alphaRange/alphaSpeed);
  
  sideslipNode->setPropertyValue("Value", Variant(Matrix(0)));
  system->simulate(system->getTime() + alphaRange/alphaSpeed);

  return EXIT_SUCCESS;
}
