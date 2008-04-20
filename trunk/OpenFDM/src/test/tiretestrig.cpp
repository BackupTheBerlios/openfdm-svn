/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include <OpenFDM/ConstModel.h>
#include <OpenFDM/DoPri5.h>
#include <OpenFDM/FixedRootJoint.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Gravity.h>
#include <OpenFDM/Ground.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/LinearSpringDamper.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/ModelVisitor.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/PrismaticJoint.h>
#include <OpenFDM/RevoluteActuator.h>
#include <OpenFDM/RevoluteJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/SimulationTime.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/System.h>
#include <OpenFDM/UnaryFunctionModel.h>
#include <OpenFDM/WheelContact.h>

#include "HDF5Writer.h"

using namespace OpenFDM;

class MovingGround : public Ground {
public:
  MovingGround(const Vector3& linearVelocity) : mVelocity(linearVelocity)
  { }
  virtual GroundValues getGroundPlane(real_type, const Vector3& refPos) const
  {
    return GroundValues(Plane(Vector3(0, 0, -1), Vector3::zeros()),
                        Vector6(Vector3::zeros(), mVelocity),
                        real_type(1));
  }
private:
  Vector3 mVelocity;
};

class ZGravity : public Gravity {
public:
  virtual Vector3 gravityAccel(const Vector3& cart) const
  { return Vector3(0, 0, 9.81); }
};

int
main(int argc, char *argv[])
{
  real_type omega = 2;

  SharedPtr<System> system = new System("Tire Testrig");
  system->addSampleTime(real_type(1)/real_type(100));
  system->setTimestepper(new DoPri5);

  // set the moving ground
  system->getEnvironment()->setGround(new MovingGround(Vector3(10, 0, 0)));
  system->getEnvironment()->setGravity(new ZGravity());
  
  // First build up the mechanical system
  FixedRootJoint* fixedRootJoint = new FixedRootJoint("Fixed Root Joint");
  system->addModel(fixedRootJoint);
  fixedRootJoint->setRefPosition(Vector3(0, 0, 1));

  PrismaticJoint* prismaticJoint = new PrismaticJoint("Normal Force joint");
  prismaticJoint->setJointAxis(Vector3::unit(2));
  system->addModel(prismaticJoint);

  Summer* normalForceSum = new Summer("Normal Force Sum");
  normalForceSum->setNumSummands(2);
  normalForceSum->setInputSign(0, Summer::Minus);
  system->addModel(normalForceSum);
  Connection::connect(prismaticJoint->getInputPort(0),
                      normalForceSum->getOutputPort(0));

  ConstModel* normalForce = new ConstModel("Normal force");
  normalForce->setScalarValue(2000);
  system->addModel(normalForce);
  Connection::connect(normalForceSum->getInputPort(0),
                      normalForce->getOutputPort(0));
 
  LinearSpringDamper* strutDamper = new LinearSpringDamper("Strut Damper");
  strutDamper->setSpringConstant(0);
  strutDamper->setDamperConstant(-30);
  system->addModel(strutDamper);
  Connection::connect(normalForceSum->getInputPort(1),
                      strutDamper->getOutputPort(0));
  Connection::connect(strutDamper->getInputPort(0),
                      prismaticJoint->getOutputPort(0));
  Connection::connect(strutDamper->getInputPort(1),
                      prismaticJoint->getOutputPort(1));

  RigidBody* rootMount = new RigidBody("Root Mount");
  system->addModel(rootMount);
  rootMount->setInboardJoint(fixedRootJoint);
  rootMount->addInteract(prismaticJoint);


  RevoluteActuator* camberActuator = new RevoluteActuator("Camber Actuator");
  system->addModel(camberActuator);

  ConstModel* camberAngle = new ConstModel("Camber Angle");
  camberAngle->setScalarValue(0);
  system->addModel(camberAngle);
  Connection::connect(camberActuator->getInputPort(0),
                      camberAngle->getOutputPort(0));

  RigidBody* normalForceStrut = new RigidBody("Normal Force Strut");
  system->addModel(normalForceStrut);
  normalForceStrut->setInboardJoint(prismaticJoint);
  normalForceStrut->addInteract(camberActuator);

 
  RevoluteActuator* sideActuator = new RevoluteActuator("Sideslip Actuator");
  system->addModel(sideActuator);

  ConstModel* sideslipAngle = new ConstModel("Sideslip Angle");
  sideslipAngle->setScalarValue(0);
  system->addModel(sideslipAngle);
  Connection::connect(sideActuator->getInputPort(0),
                      sideslipAngle->getOutputPort(0));

  RigidBody* camberStrut = new RigidBody("Camber Strut");
  system->addModel(camberStrut);
  camberStrut->setInboardJoint(camberActuator);
  camberStrut->addInteract(sideActuator);


  RevoluteJoint* hubJoint = new RevoluteJoint("Hub Joint");
  hubJoint->setJointAxis(Vector3(0, 1, 0));
  system->addModel(hubJoint);

  RigidBody* hubStrut = new RigidBody("Hub Strut");
  system->addModel(hubStrut);
  hubStrut->setInboardJoint(sideActuator);
  hubStrut->addInteract(hubJoint);

  RigidBody* rimAndTire = new RigidBody("Rim And Tire");
  system->addModel(rimAndTire);
  rimAndTire->setInboardJoint(hubJoint);

  Mass* tireAndRimMass = new Mass("Rim And Tire Mass");
  tireAndRimMass->setInertia(1, InertiaMatrix(1, 0, 0, 1, 0, 1));
  system->addModel(tireAndRimMass);
  rimAndTire->addInteract(tireAndRimMass);
  
  WheelContact* wheelContact = new WheelContact("Wheel Contact");
  wheelContact->setWheelRadius(0.3);
  wheelContact->setSpringConstant(1000);
  wheelContact->setSpringDamping(sqrt(wheelContact->getSpringConstant())/10);
  system->addModel(wheelContact);
  rimAndTire->addInteract(wheelContact);

  if (!system->init()) {
    std::cout << "Could not initialize the system" << std::endl;
    return EXIT_FAILURE;
  }

  HDF5Writer hwriter("system.h5");
  system->accept(hwriter);
  while (system->getTime() < 10) {
    system->simulate(system->getTime() + 0.01);
    system->accept(hwriter);
  }
  
  return EXIT_SUCCESS;
}
