/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include <OpenFDM/ConstModel.h>
#include <OpenFDM/DoPri5.h>
#include <OpenFDM/FixedRootJoint.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/LinearSpringDamper.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/Group.h>
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
#include <OpenFDM/WheelContact.h>

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

int
main(int argc, char *argv[])
{
  SharedPtr<Group> group = new Group("Tire Testrig");

  // First build up the mechanical system
  FixedRootJoint* fixedRootJoint = new FixedRootJoint("Fixed Root Joint");
  group->addChild(fixedRootJoint);
  fixedRootJoint->setRootPosition(Vector3(0, 0, -1));

  PrismaticJoint* prismaticJoint = new PrismaticJoint("Normal Force joint");
  prismaticJoint->setAxis(Vector3::unit(2));
  prismaticJoint->setEnableExternalForce(true);
  group->addChild(prismaticJoint);

  Summer* normalForceSum = new Summer("Normal Force Sum");
  normalForceSum->setNumSummands(2);
  group->addChild(normalForceSum);
  group->connect(prismaticJoint->getPort("force"),
                 normalForceSum->getPort("output"));

  ConstModel* normalForce = new ConstModel("Normal force");
  normalForce->setScalarValue(2000);
  group->addChild(normalForce);
  group->connect(normalForceSum->getPort("input0"),
                 normalForce->getPort("output"));
 
  LinearSpringDamper* strutDamper = new LinearSpringDamper("Strut Damper");
  strutDamper->setSpringConstant(0);
  strutDamper->setDamperConstant(30);
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
  group->addChild(camberActuator);

  ConstModel* camberAngle = new ConstModel("Camber Angle");
  camberAngle->setScalarValue(0);
  group->addChild(camberAngle);
  group->connect(camberActuator->getPort("input"),
                 camberAngle->getPort("output"));

  RigidBody* normalForceStrut = new RigidBody("Normal Force Strut");
  group->addChild(normalForceStrut);
  group->connect(normalForceStrut->getPort("link0"),
                 prismaticJoint->getPort("link1"));
  group->connect(normalForceStrut->getPort("link1"),
                 camberActuator->getPort("link0"));

 
  RevoluteActuator* sideActuator = new RevoluteActuator("Sideslip Actuator");
  group->addChild(sideActuator);

  ConstModel* sideslipAngle = new ConstModel("Sideslip Angle");
  sideslipAngle->setScalarValue(0);
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

  RigidBody* rimAndTire = new RigidBody("Rim And Tire");
  group->addChild(rimAndTire);
  group->connect(hubJoint->getPort("link1"), rimAndTire->getPort("link0"));

  Mass* tireAndRimMass = new Mass("Rim And Tire Mass");
  tireAndRimMass->setMass(1);
  tireAndRimMass->setInertia(InertiaMatrix(1, 0, 0, 1, 0, 1));
  group->addChild(tireAndRimMass);
  group->connect(rimAndTire->getPort("link1"), tireAndRimMass->getPort("link"));
  
  WheelContact* wheelContact = new WheelContact("Wheel Contact");
  wheelContact->setWheelRadius(0.3);
  wheelContact->setSpringConstant(50000);
  wheelContact->setDampingConstant(sqrt(wheelContact->getSpringConstant())/10);
  group->addChild(wheelContact);
  rimAndTire->addLink("link2");
  group->connect(rimAndTire->getPort("link2"), wheelContact->getPort("link"));

  SharedPtr<System> system = new System("Tire Testrig", group);

  // set the moving ground
  system->getEnvironment()->setGround(new MovingGround(Vector3(10, 0, 0)));

  system->attach(SystemOutput::newDefaultSystemOutput("tiretestrig"));
  if (!system->init())
    return 1;

  system->simulate(1);
  
  return EXIT_SUCCESS;
}
