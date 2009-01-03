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

class AccelerationTracking : public SingleLinkInteract {
public:
  AccelerationTracking() :
    SingleLinkInteract("AccelerationTracking"),
//     mAccelerationInputPort0(this, "accelerationInput0", Size(3, 1), true),
//     mAccelerationInputPort1(this, "accelerationInput1", Size(3, 1), true),
//     mVelocityPort(this, "velocity", Size(6, 1), true),
    mAccelerationPort(this, "acceleration", Size(6, 1), true)
  { }

class Context : public SingleLinkInteract::Context {
public:
  Context(const AccelerationTracking* accelerationTracking,
          const Environment* environment, PortValueList& portValueList) :
    SingleLinkInteract::Context(accelerationTracking, environment, portValueList),
    mAccelerationTracking(accelerationTracking),
    mLinkRelPos(Vector3::zeros())
  { }
  virtual ~Context() {}
    
  virtual const AccelerationTracking& getNode() const
  { return *mAccelerationTracking; }
  
  virtual void initDesignPosition()
  {
//     mLinkRelPos = mAccelerationTracking->getPosition() - getLink().getDesignPosition();
  }
  
  virtual void velocities(const Task& task)
  {
    mAccelerationTracking->velocity(task, getEnvironment(), mContinousState, mPortValueList);
  }
  virtual void articulation(const Task& task)
  {
//     mAccelerationTracking->articulation(task, getEnvironment(), mContinousState, mPortValueList);
  }
  virtual void accelerations(const Task& task)
  {
    mAccelerationTracking->acceleration(task, getEnvironment(), mContinousState, mPortValueList);
  }
  
private:
  SharedPtr<const AccelerationTracking> mAccelerationTracking;
  Vector3 mLinkRelPos;
};

  virtual MechanicContext* newMechanicContext(const Environment* environment,
                                              PortValueList& portValueList) const
  {
    SharedPtr<Context> context = new Context(this, environment, portValueList);
    if (!context->alloc()) {
      Log(Model, Warning) << "Could not alloc for model \""
                          << getName() << "\"" << endl;
      return 0;
    }
    return context.release();
  }
  
  virtual void initDesignPosition(PortValueList&) const
  {
  }
  virtual void velocity(const Task& task, const Environment& environment,
                        const ContinousStateValueVector&,
                        PortValueList&) const
  {
  }
  virtual void acceleration(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
                            PortValueList& portValues) const
  {
    Vector3 p1(0, 1, 0);
    Vector3 p2(0, -1, 0);

    const Frame& frame = portValues[mMechanicLink].getFrame();

    CoordinateSystem csys0 = portValues[mMechanicLink].getCoordinateSystem();

    CoordinateSystem csys1 = csys0.getRelative(p1);
    CoordinateSystem csys2 = csys0.getRelative(p2);

    Frame frame1;
    frame1.setPosAndVel(frame, p1, Quaternion::unit(), Vector6::zeros());
    frame1.setAccel(frame, Vector6::zeros());
    Frame frame2;
    frame2.setPosAndVel(frame, p2, Quaternion::unit(), Vector6::zeros());
    frame2.setAccel(frame, Vector6::zeros());

    Vector3 refPosition = csys0.getPosition();
    Vector3 gravity = environment.getGravityAcceleration(refPosition);
    gravity = frame.rotFromRef(gravity);

    Vector3 a1 = frame1.getClassicAccel().getLinear() - gravity;
    Vector3 a2 = frame2.getClassicAccel().getLinear() - gravity;
   
    /// Here we have constructed the synthetic example
//     std::cout << trans(a1) << " " << trans(a2) << std::endl;

    Vector6 v = frame.getSpVel();
    Vector6 aExact = frame.getSpAccel();

    Vector3 dp = p2 - p1;
    
    Vector3 omega0 = v.getAngular();
    Vector3 v0 = v.getLinear();
    
//   Vector6 getClassicAccel(void) const
//   {
//     Vector6 iv = getSpVel();
//     return getRelVelDot() + getParentSpAccel() + getHdot()
//       + Vector6(Vector3::zeros(), cross(iv.getAngular(), iv.getLinear()));
//   }

//   /** FIXME belongs into the joints.
//    */
//   Vector6 getHdot(void) const
//   {
//     /**
//        This is the cross product of the inertial spatial velocity
//        vector with the relative spatial velocity vector (motion type
//        cross product). Since the inertial velocity is the transformed
//        inertial velocity of the parent frame plus the relative
//        velocity of the current frame, all the relative velocity
//        components cancel out in this expression. What remains is the
//        transformed spatial velocity of the parent frame cross the
//        relative velocity.
//      */
//     Vector6 pivel = getParentSpVel();
//     return Vector6(cross(pivel.getAngular(), getAngularRelVel()),
//                    cross(pivel.getAngular(), getLinearRelVel()) + 
//                    cross(pivel.getLinear(), getAngularRelVel()));
//   }

//   Vector6 getSpVel(void) const
//   { return getRelVel() + getParentSpVel(); }

//     Vector3 a1_ = aExact.getLinear() - cross(p1, aExact.getAngular())
//       - cross(omega0, cross(p1, omega0)) + cross(omega0, v0) - gravity;
//     Vector3 a2_ = aExact.getLinear() - cross(p2, aExact.getAngular())
//       - cross(omega0, cross(p2, omega0)) + cross(omega0, v0) - gravity;
//     std::cout << trans(a1) << " " << trans(a1_) << std::endl;

    Vector3 dpOmega = -(a2 - a1 + cross(omega0, cross(dp, omega0)));

//     Vector3 dpOmegaExact = cross(dp, aExact.getAngular());
//     std::cout << trans(dpOmega) << " " << trans(dpOmegaExact) << std::endl;


    Vector3 omegaDot = crossKern(dp, dpOmega);

//     std::cout << trans(dpOmega) << " " << trans(cross(dp, omegaDot)) << " " << std::endl;
//     std::cout << trans(omegaDot) << " " << trans(aExact.getAngular()) << std::endl;
    
    Vector3 vDot1 = a1 + cross(p1, omegaDot) + cross(omega0, cross(p1, omega0)) - cross(omega0, v0) + gravity;
    Vector3 vDot2 = a2 + cross(p2, omegaDot) + cross(omega0, cross(p2, omega0)) - cross(omega0, v0) + gravity;

    Vector6 a(omegaDot, vDot1);

    std::cout << trans(a) << " " << trans(frame.getSpAccel()) << std::endl;

    portValues[mAccelerationPort] = a;
  }

protected:
  MatrixInputPort mVelocityPort;
  MatrixInputPort mAccelerationInputPort0;
  MatrixInputPort mAccelerationInputPort1;
  MatrixOutputPort mAccelerationPort;
};


class MobileRootJoint2 : public RootJoint {
  OPENFDM_OBJECT(MobileRootJoint2, RootJoint);
public:
  MobileRootJoint2(const std::string& name) :
    RootJoint(name),
    mAccelerationPort(this, "acceleration", Size(6, 1), true),
    mVelocityPort(this, "velocity", Size(6, 1)),
    mPositionPort(this, "position", Size(3, 1)),
    mOrientationPort(this, "orientation", Size(4, 1)),
    mPositionStateInfo(new Vector3StateInfo),
    mOrientationStateInfo(new Vector4StateInfo),
    mVelocityStateInfo(new Vector6StateInfo)
  {
    addContinousStateInfo(mPositionStateInfo);
    addContinousStateInfo(mOrientationStateInfo);
    addContinousStateInfo(mVelocityStateInfo);
  }
  virtual ~MobileRootJoint2() {}


  void init(const Task&, DiscreteStateValueVector&,
            ContinousStateValueVector& continousState,
            const PortValueList& portValues) const
  {
    continousState[*mPositionStateInfo] = Vector3::zeros();
    continousState[*mOrientationStateInfo] = Quaternion::unit();
    continousState[*mVelocityStateInfo] = Vector6::zeros();
  }

  void initDesignPosition(PortValueList& portValues) const
  {
//     portValues[mMechanicLink].setDesignPosition(Vector3::zeros());
  }

  void velocity(const Task& task, const Environment& environment,
                const ContinousStateValueVector& continousState,
                PortValueList& portValues) const
  {
//     const Environment* environment;
//     environment = portValues[mMechanicLink].getEnvironment();
//     Vector3 angularBaseVelocity = environment->getAngularVelocity(task.getTime());
    
//     Vector3 position = continousState[*mPositionStateInfo];
//     Quaternion orientation = continousState[*mOrientationStateInfo];
//     Vector6 velocity = continousState[*mVelocityStateInfo];
    
//     portValues[mMechanicLink].setCoordinateSystem(CoordinateSystem(position,
//                                                                    orientation));
//     portValues[mMechanicLink].setPosAndVel(angularBaseVelocity,
//                                            position, orientation, velocity);
  }

  void articulation(const Task&, const Environment& environment,
                    const ContinousStateValueVector&, PortValueList&) const
  {
    /// In this case a noop.
  }
  
  void acceleration(const Task& task, const Environment& environment,
                    const ContinousStateValueVector&,
                    PortValueList& portValues) const
  {
//     const Environment* environment;
//     environment = portValues[mMechanicLink].getEnvironment();
//     Vector6 spatialAcceleration = environment->getAcceleration(task.getTime());
    Vector6 spatialAcceleration = Vector6::zeros();
    
//     SpatialInertia inertia = portValues[mMechanicLink].getInertia();
//     Vector6 force = portValues[mMechanicLink].getForce();
    
    spatialAcceleration = portValues[mAccelerationPort];
//     portValues[mMechanicLink].getFrame().setSpAccel(spatialAcceleration);
  }
  
  void derivative(const Environment& environment,
                  const DiscreteStateValueVector&,
                  const ContinousStateValueVector& continousState,
                  const PortValueList& portValues,
                  ContinousStateValueVector& derivatives) const
  {
    Quaternion orientation = continousState[*mOrientationStateInfo];
    Vector6 velocity = continousState[*mVelocityStateInfo];
    
    Vector3 pDot = orientation.backTransform(velocity.getLinear());
    
    // Compute the derivative term originating from the angular velocity.
    // Correction term to keep the quaternion normalized.
    // That is if |q| < 1 add a little radial component outward,
    // if |q| > 1 add a little radial component inward
    Quaternion q = orientation;
    Vector3 angVel = velocity.getAngular();
    Vector4 qderiv = LinAlg::derivative(q, angVel) + 1e1*(normalize(q) - q);
    
//     Vector6 velDeriv = portValues[mMechanicLink].getFrame().getRelVelDot();
    
    derivatives[*mPositionStateInfo] = pDot;
    derivatives[*mOrientationStateInfo] = qderiv;
//     derivatives[*mVelocityStateInfo] = velDeriv;
  }
private:
  MatrixInputPort mAccelerationPort;
  MatrixOutputPort mVelocityPort;
  MatrixOutputPort mPositionPort;
  MatrixOutputPort mOrientationPort;
  
  SharedPtr<Vector3StateInfo> mPositionStateInfo;
  SharedPtr<Vector4StateInfo> mOrientationStateInfo;
  SharedPtr<Vector6StateInfo> mVelocityStateInfo;
};



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
  sensor->setEnableAllOutputs(true);
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
  rigidBody->addLink("accelerationTrackerLink");
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
  sensor->setEnableAllOutputs(true);
  group->addChild(sensor);

  Sensor* sensor2 = new Sensor("Sensor 2");
  sensor2->setPosition(mass2->getPosition());
  sensor2->setEnableAllOutputs(true);
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
  internalSensor->setEnableAllOutputs(true);
  internalSensor->setEnableForce(true);
  group->addChild(internalSensor);
  group->connect(internalSensor->getPort("link0"),
                 rigidBody->getPort("internalSensorLink"));
  group->connect(internalSensor->getPort("link1"),
                 rigidBody2->getPort("internalSensorLink"));


  InternalSensor* internalSensor2 = new InternalSensor("Internal Sensor 2");
  internalSensor2->setPosition0(Vector3(0, 0, 0.8));
  internalSensor2->setPosition1(Vector3(0, 0, 1));
  internalSensor2->setEnableAllOutputs(true);
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


  AccelerationTracking* accelerationTracking = new AccelerationTracking;
  group->addChild(accelerationTracking);
  group->connect(accelerationTracking->getPort("link"),
                 rigidBody->getPort("accelerationTrackerLink"));

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

