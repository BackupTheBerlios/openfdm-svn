/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "JSBSimReader.h"

#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>
#include <stack>

#include <OpenFDM/Vector.h>
#include <OpenFDM/Matrix.h>
#include <OpenFDM/Quaternion.h>

#include <OpenFDM/AirSpring.h>
#include <OpenFDM/Bias.h>
#include <OpenFDM/BinaryFunction.h>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/DeadBand.h>
#include <OpenFDM/Delay.h>
#include <OpenFDM/DiscBrake.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Input.h>
#include <OpenFDM/InternalInteract.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/MaxModel.h>
#include <OpenFDM/MatrixConcat.h>
#include <OpenFDM/MatrixSplit.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/PrismaticJoint.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/RevoluteActuator.h>
#include <OpenFDM/RevoluteJoint.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/SimpleContact.h>
#include <OpenFDM/SimpleGear.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/System.h>
#include <OpenFDM/Table.h>
#include <OpenFDM/TimeDerivative.h>
#include <OpenFDM/TransferFunction.h>
#include <OpenFDM/UnaryFunction.h>
#include <OpenFDM/Unit.h>
#include <OpenFDM/UnitConversion.h>
#include <OpenFDM/WheelContact.h>

#include <OpenFDM/ReaderWriter.h>

#include "JSBSimActuator.h"
#include "JSBSimAerosurfaceScale.h"
#include "JSBSimKinemat.h"
#include "JSBSimPID.h"
#include "JSBSimScheduledGain.h"
#include "JSBSimSwitch.h"

namespace OpenFDM {

static bool
isReal(const std::string& s)
{
  std::stringstream stream(s);
  real_type value;
  stream >> value;
  return stream;
}

static bool
isReal(const XMLElement* element)
{
  if (!element)
    return false;
  return isReal(element->getData());
}

static real_type
asciiToReal(const std::string& s, const real_type& def = 0)
{
  std::stringstream stream(s);
  real_type value;
  stream >> value;
  if (!stream)
    return def;
  return value;
}

static real_type
realData(const XMLElement* element, real_type def = 0)
{
  if (!element)
    return def;
  return asciiToReal(element->getData(), def);
}

static Vector3
locationData(const XMLElement* element, const Vector3& def = Vector3(0, 0, 0))
{
  if (!element)
    return def;
  const XMLElement* xElement = element->getElement("x");
  if (!xElement)
    return def;
  const XMLElement* yElement = element->getElement("y");
  if (!yElement)
    return def;
  const XMLElement* zElement = element->getElement("z");
  if (!zElement)
    return def;
  Vector3 value = def;
  {
    std::stringstream stream(xElement->getData());
    stream >> value(0);
    //   if (!stream)
    //     return def;
  }
  {
    std::stringstream stream(yElement->getData());
    stream >> value(1);
    //   if (!stream)
    //     return def;
  }
  {
    std::stringstream stream(zElement->getData());
    stream >> value(2);
    //   if (!stream)
    //     return def;
  }
  return value;
}

static Vector3
locationData(const std::list<const XMLElement*>& locList, const char* name,
             const Vector3& def = Vector3(0, 0, 0))
{
  const XMLElement* element = 0;
  std::list<const XMLElement*>::const_iterator it;
  it = locList.begin();
  while (it != locList.end()) {
    if ((*it)->getAttribute("name") == name)
      element = *it;
    ++it;
  }

  return locationData(element, def);
}

static Vector3
orientationData(const XMLElement* element, const Vector3& def = Vector3(0, 0, 0))
{
  if (!element)
    return def;
  const XMLElement* xElement = element->getElement("roll");
  if (!xElement)
    return def;
  const XMLElement* yElement = element->getElement("pitch");
  if (!yElement)
    return def;
  const XMLElement* zElement = element->getElement("yaw");
  if (!zElement)
    return def;
  Vector3 value = def;
  {
    std::stringstream stream(xElement->getData());
    stream >> value(0);
    //   if (!stream)
    //     return def;
  }
  {
    std::stringstream stream(yElement->getData());
    stream >> value(1);
    //   if (!stream)
    //     return def;
  }
  {
    std::stringstream stream(zElement->getData());
    stream >> value(2);
    //   if (!stream)
    //     return def;
  }
  return value;
}

static InertiaMatrix
inertiaData(const XMLElement* element,
            const InertiaMatrix& def = InertiaMatrix(1, 0, 0, 1, 0, 1))
{
  InertiaMatrix I;
  I(0, 0) = realData(element->getElement("ixx"), def(0, 0));
  I(0, 1) = realData(element->getElement("ixy"), def(0, 1));
  I(0, 2) = realData(element->getElement("ixz"), def(0, 2));
  I(1, 1) = realData(element->getElement("iyy"), def(1, 1));
  I(1, 2) = realData(element->getElement("iyz"), def(1, 2));
  I(2, 2) = realData(element->getElement("izz"), def(2, 2));
  return I;
}

static std::string
stringData(const XMLElement* element)
{
  if (!element)
    return std::string();
  std::stringstream stream(element->getData());
  std::string value;
  stream >> value;
  if (!stream)
    return std::string();
  return value;
}

static std::string
indepData(const std::list<const XMLElement*>& indepList, const char* name)
{
  const XMLElement* element = 0;
  std::list<const XMLElement*>::const_iterator it;
  it = indepList.begin();
  while (it != indepList.end()) {
    if ((*it)->getAttribute("lookup") == name)
      element = *it;
    ++it;
  }
  if (!element)
    return std::string();
  std::stringstream stream(element->getData());
  std::string value;
  stream >> value;
  if (!stream)
    return std::string();
  return value;
}

static std::list<std::string>
getInputs(const XMLElement* element)
{
  std::list<std::string> ret;
  if (!element)
    return ret;
  std::list<const XMLElement*> inputs = element->getElements("input");
  std::list<const XMLElement*>::iterator it = inputs.begin();
  while (it != inputs.end()) {
    ret.push_back(stringData(*it));
    ++it;
  }
  return ret;
}

static AirSpring*
getAirSpring(const XMLElement* airSpringElem, const std::string& name)
{
  AirSpring* aoDamp = new AirSpring(name + " Air Spring Force");
  real_type pullPress
    = realData(airSpringElem->getElement("pullPressure"), 1e5);
  aoDamp->setPullPressure(pullPress);
  real_type pushPress
    = realData(airSpringElem->getElement("pushPressure"), 5e5);
  aoDamp->setPushPressure(pushPress);
  real_type area
    = realData(airSpringElem->getElement("area"), 0.01);
  aoDamp->setArea(area);
  real_type minCompr
    = realData(airSpringElem->getElement("minCompression"), 0);
  aoDamp->setMinCompression(minCompr);
  real_type maxCompr
    = realData(airSpringElem->getElement("maxCompression"), 0.2);
  aoDamp->setMaxCompression(maxCompr);
  real_type minDamp
    = realData(airSpringElem->getElement("minDamping"), 1e3);
  aoDamp->setMinDamperConstant(minDamp);
  real_type maxDamp
    = realData(airSpringElem->getElement("maxDamping"), 1e3);
  aoDamp->setMaxDamperConstant(maxDamp);
  return aoDamp;
}
  
JSBSimReader::JSBSimReader(void)
{
}

JSBSimReader::~JSBSimReader(void)
{
}

void
JSBSimReader::reset(void)
{
  mSystem = 0;
  mTopLevelGroup = 0;
  mAeroForce = 0;
  mTopLevelBody = 0;
}

bool
JSBSimReader::loadAircraft(const std::string& acFileName)
{
  // Reset the vehicle.
  resetErrorState();

  mPropertyManager.clear();
  // Allocate a new system
  mSystem = new System("JSBSim system");
  mTopLevelGroup = new Group("TopLevelGroup");
  mSystem->setNode(mTopLevelGroup);
  mAeroForce = new JSBSimAerodynamic("Aerodynamic force");
  mTopLevelGroup->addChild(mAeroForce->getGroup());
  mTopLevelBody = new RigidBody("Main Aircraft Body");
  mTopLevelGroup->addChild(mTopLevelBody);
  mTopLevelGroup->connect(mTopLevelBody->addLink("aerodynamicLink"),
                          mAeroForce->getMechanicLink());

  MobileRootJoint* mobileRootJoint = new MobileRootJoint("Root Joint");
  mTopLevelGroup->addChild(mobileRootJoint);
  mTopLevelGroup->connect(mTopLevelBody->addLink("rootJointLink"),
                          mobileRootJoint->getPort("link"));

  // Default discrete stepsize of JSBSim
  mTopLevelGroup->setSampleTime(SampleTime(1.0/120));

  // Try to find the given file on the given search path
  std::ifstream acFileStream;
  if (!openFile(mAircraftPath, acFileName, acFileStream))
    return false;

  // Get a parser FIXME
  if (!convertDocument(parseXMLStream(acFileStream)))
    return false;

  // Provide substitutes for still unresolved properties
  if (!provideSubstitutes())
    return false;

  // Finnaly connect all the collected properties ports together
  if (!connect())
    return false;

  return true;
}

bool
JSBSimReader::convertDocument(const XMLElement* topElem)
{
  // Be paranoid ...
  if (!topElem)
    return error("No toplevel xml element found");

  if (topElem->getName() != "fdm_config")
    return error("Toplevel xml element is no fdm_config");

  if (topElem->getAttribute("version").compare(0, 2, "2.") != 0)
    return error("Toplevel xml element version does not begin with \"2.\"");

  // Parse the metrics section.
  const XMLElement* metricsElem = topElem->getElement("metrics");
  if (!metricsElem)
    return error("Cannot get metrics element");
  if (!convertMetrics(metricsElem))
    return error("Cannot convert metrics data");

  // Parse the massbalance section.
  const XMLElement* massBalanceElem = topElem->getElement("mass_balance");
  if (!massBalanceElem)
    return error("Cannot get mass_balance element");
  if (!convertMassBalance(massBalanceElem))
    return error("Cannot convert mass_balance data");

  // Convert all the flight control system elements.
  const XMLElement* fcsElem = topElem->getElement("flight_control");
  if (fcsElem) {
    if (!convertFCSList(fcsElem))
      return error("Cannot convert flight_control element");
  }
  const XMLElement* autopilotElem = topElem->getElement("autopilot");
  if (autopilotElem) {
    if (!convertFCSList(autopilotElem))
      return error("Cannot convert autopilot element");
  }

  // Parse the undercarriage section
  const XMLElement* groundReactionsElem = topElem->getElement("ground_reactions");
  if (groundReactionsElem) {
    if (!convertGroundReactionsElem(groundReactionsElem))
      return error("Cannot convert ground_reactions");
  }

  // external_reactions - neat
  const XMLElement* externalReactionsElem = topElem->getElement("external_reactions");
  if (externalReactionsElem) {
    std::cout << "Ignoring external_reactions section" << std::endl;
//     if (!convertGroundReactionsElem(groundReactionsElem))
//       return error("Cannot convert ground_reactions");
  }

  // Parse the propulsion section.
  const XMLElement* propulsionElem = topElem->getElement("propulsion");
  if (propulsionElem) {
    if (!convertPropulsion(propulsionElem))
      return error("Cannot convert propulsion data");
  }

  // Include the system tags.
  std::list<const XMLElement*> systemElems = topElem->getElements("system");
  for (std::list<const XMLElement*>::iterator i = systemElems.begin();
       i != systemElems.end(); ++i) {
    if (!convertSystem(*i))
      return error("Cannot convert system data");
  }
  
  // Convert the aerodynamic force.
  const XMLElement* aeroElem = topElem->getElement("aerodynamics");
  if (aeroElem) {
    if (!convertAerodynamics(aeroElem))
      return error("Cannot convert aerodynamics elements");
  }

  return true;
}

/// converts the METRICS data
bool
JSBSimReader::convertMetrics(const XMLElement* metricsElem)
{
  // Parse the METRICS section.
  // Since all locations within the aircraft are given in the structural frame,
  // we need to know the body frame location in the veihcle first.
  // That is parse all entries and than translate them into OpenFDM parts.

  /// FIXME: unit handling??
  real_type wingarea = realData(metricsElem->getElement("wingarea"), 0);
  mAeroForce->setWingArea(Unit::squareFoot().convertFrom(wingarea));

  real_type wingspan = realData(metricsElem->getElement("wingspan"), 0);
  mAeroForce->setWingSpan(Unit::foot().convertFrom(wingspan));

  real_type chord = realData(metricsElem->getElement("chord"), 0);
  mAeroForce->setChord(Unit::foot().convertFrom(chord));

  real_type iw = realData(metricsElem->getElement("wing_incidence"), 0);
  const Port* iwPort = addMultiBodyConstModel("Wing Incidence", iw);
  registerJSBExpression("metrics/iw-rad", iwPort);

  Summer* summer = new Summer("Alpha Wing Incidence");
  summer->setNumSummands(2);
  addMultiBodyModel(summer);
  connectJSBExpression("metrics/iw-rad", summer->getInputPort(0));
  connectJSBExpression("aero/alpha-rad", summer->getInputPort(1));
  registerJSBExpression("aero/alpha-wing-rad", summer->getOutputPort());

  real_type htailarea = realData(metricsElem->getElement("htailarea"), 0);
  mAeroForce->setHTailArea(htailarea);

  real_type htailarm = realData(metricsElem->getElement("htailarm"), 0);
  mAeroForce->setHTailArm(htailarm);

  real_type vtailarea = realData(metricsElem->getElement("vtailarea"), 0);
  mAeroForce->setVTailArea(vtailarea);

  real_type vtailarm = realData(metricsElem->getElement("vtailarm"), 0);
  mAeroForce->setVTailArm(vtailarm);

  std::list<const XMLElement*> locList = metricsElem->getElements("location");
  Vector3 ap = locationData(locList, "AERORP", Vector3(0, 0, 0));
  Vector3 ep = locationData(locList, "EYEPOINT", Vector3(0, 0, 0));
  Vector3 vrp = locationData(locList, "VRP", Vector3(0, 0, 0));

  // In contrast to JSBSim, we have the possibility to simulate around
  // a point not being the center of gravity, use that here ...
  Vector3 cg = mBodyReference;
  mBodyReference = vrp;
  
  // Attach the eye point.
  ExternalInteract* epInteract = new ExternalInteract("Eyepoint Sensor");
  epInteract->setPosition(structToBody(ep));
  epInteract->setEnablePosition(true);
  epInteract->setEnableOrientation(true);
  mTopLevelGroup->addChild(epInteract);
  mTopLevelGroup->connect(mTopLevelBody->addLink("epLink"),
                          epInteract->getPort("link"));

  ExternalInteract* accelInteract = new ExternalInteract("Acceleration Sensor");
  accelInteract->setEnableLoad(true);
  accelInteract->setEnableCentrifugalAcceleration(true);
  mTopLevelGroup->addChild(accelInteract);
  mTopLevelGroup->connect(mTopLevelBody->addLink("accelLink"),
                          accelInteract->getPort("link"));

  MatrixSplit* load = new MatrixSplit("Body Load Split");
  mTopLevelGroup->addChild(load);
  mTopLevelGroup->connect(accelInteract->getPort("load"),
                          load->getPort("input"));
  load->addOutputPort("x");
  load->addOutputPort("y");
  load->addOutputPort("z");
  Delay* delay = new Delay("Normalized Load Delay", 1);
  mTopLevelGroup->addChild(delay);
  mTopLevelGroup->connect(load->getPort("z"), delay->getPort("input"));
  const Port* port = delay->getPort("output");
  registerJSBExpression("accelerations/n-pilot-z-norm", port);
  addOutputModel(port, "Normalized load value", "accelerations/nlf");

  MatrixSplit* accel = new MatrixSplit("Body Accel Split");
  mTopLevelGroup->addChild(accel);
  mTopLevelGroup->connect(accelInteract->getPort("centrifugalAcceleration"),
                          accel->getPort("input"));
  accel->addOutputPort("x");
  accel->addOutputPort("y");
  accel->addOutputPort("z");

  delay = new Delay("Acceleration Delay", 1);
  mTopLevelGroup->addChild(delay);
  mTopLevelGroup->connect(accel->getPort("z"), delay->getPort("input"));
  port = delay->getPort("output");

  registerJSBExpression("accelerations/accel-z-norm", port);

  // Set the position of the aerodynamic force frame.
  mAeroForce->setPosition(structToBody(ap));

  NodePath nodePath = mTopLevelGroup->getNodePathList().front();
  addOutputModel("aero/alpha-deg", "Alpha", "orientation/alpha-deg");
  addOutputModel("aero/beta-rad", "Beta rad", "orientation/side-slip-rad");
  addOutputModel("aero/beta-deg", "Beta", "orientation/side-slip-deg");

  return true;
}

/// converts the MASSBALANCE data
bool
JSBSimReader::convertMassBalance(const XMLElement* massBalance)
{
  // Parse the MASSBALANCE section.
  // Since all locations within the aircraft are given in the structural frame,
  // we need to know the body frame location in the veihcle first.
  // That is parse all entries and than translate them into OpenFDM parts.

  InertiaMatrix I(0, 0, 0, 0, 0, 0);
  real_type mass = 0;

  real_type ixx = realData(massBalance->getElement("ixx"), 0);
  I(0, 0) = Unit::slugSquareFoot().convertFrom(ixx);
  real_type iyy = realData(massBalance->getElement("iyy"), 0);
  I(1, 1) = Unit::slugSquareFoot().convertFrom(iyy);
  real_type izz = realData(massBalance->getElement("izz"), 0);
  I(2, 2) = Unit::slugSquareFoot().convertFrom(izz);
  real_type ixy = realData(massBalance->getElement("ixy"), 0);
  I(0, 1) = Unit::slugSquareFoot().convertFrom(ixy);
  real_type ixz = realData(massBalance->getElement("ixz"), 0);
  I(0, 2) = Unit::slugSquareFoot().convertFrom(ixz);
  real_type iyz = realData(massBalance->getElement("iyz"), 0);
  I(1, 2) = Unit::slugSquareFoot().convertFrom(iyz);
  
  mass = realData(massBalance->getElement("emptywt"), 0);
  mass = Unit::lbs().convertFrom(mass);

  std::list<const XMLElement*> locList = massBalance->getElements("location");
  Vector3 cg = locationData(locList, "CG", Vector3(0, 0, 0));

  Mass* massModel = new Mass("Emptyweight Mass", mass, I, structToBody(cg));
  mTopLevelGroup->addChild(massModel);
  mTopLevelGroup->connect(mTopLevelBody->addLink("emptyWeightMass"),
                          massModel->getPort("link"));

  std::list<const XMLElement*> pmList = massBalance->getElements("pointmass");
  std::list<const XMLElement*>::const_iterator pmit;
  pmit = pmList.begin();
  while (pmit != pmList.end()) {
    std::list<const XMLElement*> locList = (*pmit)->getElements("location");
    Vector3 loc = locationData(locList, "POINTMASS", Vector3(0, 0, 0));
    real_type weight = realData((*pmit)->getElement("weight"), 0);
    weight = Unit::lbs().convertFrom(weight);
    Mass* massModel = new Mass("Mass", weight, InertiaMatrix(0,0,0,0,0,0),
                               structToBody(loc));
    mTopLevelGroup->addChild(massModel);
    mTopLevelGroup->connect(mTopLevelBody->addLink("mass"),
                            massModel->getPort("link"));
    ++pmit;
  }

  return true;
}

bool
JSBSimReader::attachWheel(const XMLElement* wheelElem, const std::string& name,
                          const std::string& numStr, RigidBody* parent)
{
  RigidBody* wheel = new RigidBody(name + " Wheel");
  mTopLevelGroup->addChild(wheel);
  InertiaMatrix wheelInertia = inertiaData(wheelElem->getElement("inertia"),
                                           InertiaMatrix(10, 0, 0, 30, 0, 10));
  real_type wheelMass = realData(wheelElem->getElement("mass"), 30);
  Mass* mass = new Mass(name + " Wheel Inertia", wheelMass, wheelInertia);
  mTopLevelGroup->addChild(mass);
  mTopLevelGroup->connect(wheel->addLink("staticMass"),
                          mass->getPort("link"));
  
  RevoluteJoint* wj = new RevoluteJoint(name + " Wheel Joint");
  mTopLevelGroup->addChild(wj);
  mTopLevelGroup->connect(parent->addLink("wheelJoint"),
                          wj->getPort("link0"));
  mTopLevelGroup->connect(wheel->addLink("wheelJoint"),
                          wj->getPort("link1"));
  wj->setAxis(Vector3(0, 1, 0));
  Vector3 pos = structToBody(locationData(wheelElem->getElement("location")));
  wj->setPosition(pos);
  wj->setInitialPosition(0);
  wj->setInitialVelocity(0);

  std::string brake = stringData(wheelElem->getElement("brake_group"));
  // Add a brake force
  if (brake == "LEFT" || brake == "RIGHT") {
    DiscBrake* brakeF = new DiscBrake(name + " Brake Force");
    addMultiBodyModel(brakeF);
    real_type minForce = realData(wheelElem->getElement("minBrakeTorque"), 8e1);
    brakeF->setMinForce(minForce);
    real_type maxForce = realData(wheelElem->getElement("maxBrakeTorque"), 1e4);
    brakeF->setMaxForce(maxForce);
    if (brake == "LEFT") {
      if (!connectJSBExpression("gear/left-brake-cmd-norm",
                                brakeF->getPort("brakeInput")))
        return error("could not connect to brake group LEFT");
    } else if (brake == "RIGHT") {
      if (!connectJSBExpression("gear/right-brake-cmd-norm",
                                brakeF->getPort("brakeInput")))
        return error("could not connect to brake group RIGHT");
    }
    // That one reads the joint position and velocity ...
    mTopLevelGroup->connect(wj->getPort("velocity"), brakeF->getPort("velocity"));
    // ... and provides an output force
    wj->setEnableExternalForce(true);
    mTopLevelGroup->connect(brakeF->getPort("force"), wj->getPort("force"));
  } else {
    // Just some 'bearing friction'
    Gain* rollingFric = new Gain(name + " Bearing Friction Force");
    addMultiBodyModel(rollingFric);
    rollingFric->setGain(-10);
    mTopLevelGroup->connect(wj->getPort("velocity"), rollingFric->getInputPort(0));
    // ... and provides an output force
    wj->setEnableExternalForce(true);
    mTopLevelGroup->connect(rollingFric->getPort("output"), wj->getPort("force"));
  }
  
  real_type wheelDiam = realData(wheelElem->getElement("wheelDiameter"));
  WheelContact* wc = new WheelContact(name + " Wheel Contact");
  mTopLevelGroup->addChild(wc);
  mTopLevelGroup->connect(wheel->addLink("wheelContact"),
                          wc->getPort("link"));
  wc->setWheelRadius(0.5*wheelDiam);
  real_type tireSpring = realData(wheelElem->getElement("tireSpring"));
  wc->setSpringConstant(Unit::lbfPerFoot().convertFrom(tireSpring));
  real_type tireDamp = realData(wheelElem->getElement("tireDamping"));
  wc->setDamperConstant(Unit::lbfPerFoot().convertFrom(tireDamp));
  real_type fc = realData(wheelElem->getElement("frictionCoef"), 0.9);
  wc->setFrictionCoeficient(fc);
  
  const Port* port = wj->getPort("position");
  std::string nameBase = "Wheel " + numStr + " Position";
  addOutputModel(port, nameBase,
                 "gear/gear[" + numStr + "]/wheel-position-rad");
  port = addToUnit(nameBase + " converter", Unit::degree(), port);
  addOutputModel(port, nameBase + " Deg",
                 "gear/gear[" + numStr + "]/wheel-position-deg");
}

bool
JSBSimReader::convertGroundReactionsElem(const XMLElement* gr)
{
  unsigned gearNumber = 0;
  std::list<const XMLElement*> comps = gr->getElements();
  std::list<const XMLElement*>::const_iterator it;
  for (it = comps.begin(); it != comps.end(); ++it) {
    if ((*it)->getName() == "contact") {
      std::string type = (*it)->getAttribute("type");
      if (type == "BOGEY") {
        std::stringstream sstr;
        sstr << gearNumber++;
        std::string numStr = sstr.str();
        std::string name = (*it)->getAttribute("name");

        // For jsbsim use simple gears
        SimpleGear* sg = new SimpleGear(name);
        mTopLevelGroup->addChild(sg);
        mTopLevelGroup->connect(mTopLevelBody->addLink("simpleGearContact"),
                                sg->getPort("link"));
        Vector3 loc
          = locationData((*it)->getElement("location"), Vector3(0, 0, 0));
        sg->setPosition(structToBody(loc));
        
        real_type k = realData((*it)->getElement("spring_coeff"), 0);
        sg->setSpringConstant(Unit::lbfPerFoot().convertFrom(k));
        real_type d = realData((*it)->getElement("damping_coeff"), 0);
        sg->setDamperConstant(Unit::lbfPerFootPerSecond().convertFrom(d));
        real_type fs = realData((*it)->getElement("static_friction"), 0);
        sg->setFrictionCoeficient(fs);
        
        // Connect apprioriate input and output models

        // FIXME
        // missing output properties are "wow" and "tire-pressure-norm"

        // FIXME no retractable for the moment
//         std::string retract = stringData((*it)->getElement("retractable"));
//         if (retract == "RETRACT" || retract == "1") {
//           if (!connectJSBExpression("gear/gear-cmd-norm", sg->getEnablePort()))
//             return error("could not connect to retract command");
//           // Well, connect that directly to the input
//           const Port* port = lookupJSBExpression("gear/gear-pos-norm");
//           addOutputModel(port, "Gear " + numStr + " Position",
//                          "gear/gear[" + numStr + "]/position-norm");
//         }

        real_type maxSteer = realData((*it)->getElement("max_steer"), 0);
        if (maxSteer != 0) {
          UnaryFunction* scale
            = new UnaryFunction(name + " Scale", UnaryFunction::Abs);
          addFCSModel(scale);
          // FIXME: FCS might later define something for that gain ...
          // "fcs/steer-pos-deg[" + numStr + "]";
          if (!connectJSBExpression("fcs/steer-cmd-norm",
                                    scale->getPort("input")))
            return error("could not connect to steering command");

          Product* sProd = new Product(name + " SProd");
          addFCSModel(sProd);
          sProd->setNumFactors(2);
          if (!connectJSBExpression("fcs/steer-cmd-norm",
                                    sProd->getPort("input0")))
            return error("could not connect to steering command");
          mTopLevelGroup->connect(scale->getPort("output"), sProd->getPort("input1"));
          const Port* port = sProd->getPort("output");

          Gain* gain = new Gain(name + " Steer Gain");
          addFCSModel(gain);
          gain->setGain(maxSteer);
          mTopLevelGroup->connect(port, gain->getPort("input"));
          addOutputModel(port, "Gear " + numStr + " Steering Output",
                         "gear/gear[" + numStr + "]/steering-norm");

          UnitConversion* unitConv
            = new UnitConversion(name + " Degree Conversion",
                                 UnitConversion::UnitToBaseUnit,
                                 Unit::degree());
          addFCSModel(unitConv);
          sg->setEnableSteeringAngle(true);
          mTopLevelGroup->connect(gain->getPort("output"),
                                  unitConv->getPort("input"));
          mTopLevelGroup->connect(unitConv->getPort("output"),
                                  sg->getPort("steeringAngle"));
        }
        
        std::string brake = stringData((*it)->getElement("brake_group"));
        if (brake == "LEFT") {
          sg->setEnableBrakeCommand(true);
          if (!connectJSBExpression("gear/left-brake-cmd-norm",
                                    sg->getPort("brakeCommand")))
            return error("could not connect brake command for group LEFT");
        } else if (brake == "RIGHT") {
          sg->setEnableBrakeCommand(true);
          if (!connectJSBExpression("gear/right-brake-cmd-norm",
                                    sg->getPort("brakeCommand")))
            return error("could not connect brake command for group RIGHT");
        }
        
      } else if (type == "STRUCTURE" || type == "CONTACT") {
        // Very simple contact force. Penalty method.
        SimpleContact* sc = new SimpleContact((*it)->getAttribute("name"));
        mTopLevelGroup->addChild(sc);
        mTopLevelGroup->connect(mTopLevelBody->addLink("simpleContact"),
                                sc->getPort("link"));

        Vector3 loc
          = locationData((*it)->getElement("location"), Vector3(0, 0, 0));
        sc->setPosition(structToBody(loc));

        real_type k = realData((*it)->getElement("spring_coeff"), 0);
        sc->setSpringConstant(Unit::lbfPerFoot().convertFrom(k));
        real_type d = realData((*it)->getElement("damping_coeff"), 0);
        sc->setDamperConstant(Unit::lbfPerFootPerSecond().convertFrom(d));
        real_type fs = realData((*it)->getElement("static_friction"), 0);
        sc->setFrictionCoeficient(fs);
        
      } else if (type == "NOSEGEAR") {
        // Ok, a compressable gear like the F-18's nose gear.
        // Some kind of hardcoding here ...
        std::stringstream sstr;
        sstr << gearNumber++;
        std::string numStr = sstr.str();
        std::string name = (*it)->getAttribute("name");

        Vector3 compressJointPos = locationData((*it)->getElement("location"));
        // Model steering here ...
        // normally we connect the compressible part to the top level body, but
        // in case of steering this is no longer true.
        RigidBody* strutParent = mTopLevelBody;
        std::string steerable = stringData((*it)->getElement("steerable"));
        if (steerable == "true" || steerable == "1") {
          // A new part modelling the steering
          RigidBody* steer = new RigidBody(name + " Steer");
          mTopLevelGroup->addChild(steer);
          
          // connect that via a revolute joint to the toplevel body.
          // Note the 0.05m below, most steering wheels have some kind of
          // castering auto line up behavour. That is doe with this 0.05m.
          RevoluteActuator* sj = new RevoluteActuator(name + " Steer Joint");
          mTopLevelGroup->addChild(sj);
          mTopLevelGroup->connect(strutParent->addLink("steerJoint"),
                                  sj->getPort("link0"));
          mTopLevelGroup->connect(steer->addLink("steerJoint"),
                                  sj->getPort("link1"));
          sj->setAxis(Vector3(0, 0, 1));
          sj->setInitialPosition(0);
          sj->setInitialVelocity(0);
          sj->setPosition(structToBody(compressJointPos) + Vector3(0.05, 0, 0));
          
          UnaryFunction* scale
            = new UnaryFunction(name + " Scale", UnaryFunction::Abs);
          addFCSModel(scale);
          if (!connectJSBExpression("fcs/steer-cmd-norm",
                                    scale->getPort("input")))
            return error("could not connect to steering command");

          Product* sProd = new Product(name + " SProd");
          addFCSModel(sProd);
          sProd->setNumFactors(2);
          if (!connectJSBExpression("fcs/steer-cmd-norm",
                                    sProd->getPort("input0")))
            return error("could not connect to steering command");
          mTopLevelGroup->connect(scale->getPort("output"), sProd->getPort("input1"));
          mTopLevelGroup->connect(sProd->getPort("output"), sj->getPort("input"));
  
          strutParent = steer;
          
          // Prepare outputs
          const Port* port = sj->getPort("position");
          std::string nameBase = "Steering " + numStr + " Position";
          addOutputModel(port, nameBase,
                         "gear/gear[" + numStr + "]/steering-pos-rad");
          port = addToUnit(nameBase + " converter", Unit::degree(), port);
          addOutputModel(port, nameBase + " Deg",
                         "gear/gear[" + numStr + "]/steering-pos-deg");
        }
        
//         const XMLElement* launchbarElem = (*it)->getElement("launchbar");
//         if (launchbarElem) {
//           Launchbar* launchbar = new Launchbar(name + " Launchbar");
//           addMultiBodyModel(launchbar);
//           strutParent->addInteract(launchbar);
//           real_type length = realData(launchbarElem->getElement("length"), 0.5);
//           launchbar->setLength(length);
//           real_type upAngle = realData(launchbarElem->getElement("upAngle"), 30);
//           launchbar->setUpAngle(Unit::degree().convertFrom(upAngle));
//           real_type downAngle = realData(launchbarElem->getElement("downAngle"), -50);
//           launchbar->setDownAngle(Unit::degree().convertFrom(downAngle));
//           real_type force = realData(launchbarElem->getElement("launchForce"), 1000);
//           launchbar->setLaunchForce(force);
//           Vector3 loc = structToBody(locationData(launchbarElem->getElement("location")));
//           launchbar->setPosition(loc);


//           if (!connectJSBExpression("/controls/gear/launchbar",
//                                     launchbar->getInputPort("tryMount")))
//             return error("could not connect to launchbar command");
//           if (!connectJSBExpression("/controls/gear/catapult-launch-cmd",
//                                     launchbar->getInputPort("launchCommand")))
//             return error("could not connect to launch command");

//           // expose the launchbar position
//           const Port* port = launchbar->getOutputPort(0);
//           std::string nameBase = "Launchbar Position";
//           addOutputModel(port, nameBase, "gear/launchbar-pos-rad");
//           port = addToUnit(nameBase + " converter", Unit::degree(), port);
//           addOutputModel(port, nameBase + " Deg", "gear/launchbar-pos-deg");
//         }

        
        // Now the compressible part of the strut
        RigidBody* arm = new RigidBody(name + " Strut");
        mTopLevelGroup->addChild(arm);
        Mass* mass = new Mass(name + " Strut Mass", 100,
                              InertiaMatrix(0,0,0,0,0,0), Vector3(0, 0, 1));
        mTopLevelGroup->addChild(mass);
        mTopLevelGroup->connect(arm->addLink("mass"),
                                mass->getPort("link"));
        
        // This time it is a prismatic joint
        PrismaticJoint* pj = new PrismaticJoint(name + " Compress Joint");
        mTopLevelGroup->addChild(pj);
        mTopLevelGroup->connect(strutParent->addLink("strutJoint"),
                                pj->getPort("link0"));
        mTopLevelGroup->connect(arm->addLink("strutJoint"),
                                pj->getPort("link1"));
        pj->setAxis(Vector3(0, 0, -1));
        pj->setPosition(structToBody(compressJointPos));
        
        // The damper element
        const XMLElement* airSpringElem = (*it)->getElement("damper");
        AirSpring* aoDamp = getAirSpring(airSpringElem, name);
        addMultiBodyModel(aoDamp);
        pj->setEnableExternalForce(true);
        mTopLevelGroup->connect(aoDamp->getPort("force"), pj->getPort("force"));
        mTopLevelGroup->connect(pj->getPort("position"), aoDamp->getPort("position"));
        mTopLevelGroup->connect(pj->getPort("velocity"), aoDamp->getPort("velocity"));
        
        // Attach a wheel to that strut part.
        attachWheel((*it)->getElement("wheel"), name, numStr, arm);
        
        // Prepare some outputs ...
        const Port* port = pj->getPort("position");
        addOutputModel(port, "Gear " + numStr + " Compression",
                       "gear/gear[" + numStr + "]/compression-m");
        
        addOutputModel("gear/gear-pos-norm", "Gear " + numStr + " Position",
                       "gear/gear[" + numStr + "]/position-norm");
        
        
      } else if (type == "F18_MLG") {
        /// Ok, a compressable gear like the F-18's main gear is.
        /// Some kind of hardcoding here ...
        std::stringstream sstr;
        sstr << gearNumber++;
        std::string numStr = sstr.str();
        std::string name = (*it)->getAttribute("name");

        // This is the movable part of the strut, doing the compression
        RigidBody* arm = new RigidBody(name + " Arm");
        mTopLevelGroup->addChild(arm);
        Mass* mass = new Mass(name + " Strut Mass", 80,
                              InertiaMatrix(0,0,0,0,0,0), Vector3(-1, 0, 0));
        mTopLevelGroup->addChild(mass);
        mTopLevelGroup->connect(arm->addLink("mass"),
                                mass->getPort("link"));
        
        // Connect that with a revolute joint to the main body
        RevoluteJoint* rj = new RevoluteJoint(name + " Arm Joint");
        mTopLevelGroup->addChild(rj);
        mTopLevelGroup->connect(mTopLevelBody->addLink("mainLangingGear"),
                                rj->getPort("link0"));
        mTopLevelGroup->connect(arm->addLink("compressJoint"),
                                rj->getPort("link1"));
        Vector3 compressJointAxis = locationData((*it)->getElement("axis"),
                                                 Vector3(0, 1, 0));
        rj->setAxis(normalize(compressJointAxis));
        Vector3 compressJointPos = locationData((*it)->getElement("location"));
        rj->setPosition(structToBody(compressJointPos));
        rj->setInitialPosition(0);
        rj->setInitialVelocity(0);
        
        InternalInteract* lineForce = new InternalInteract(name + " Air Spring LineForce");
        lineForce->setEnableDistance(true);
        lineForce->setEnableVelocity(true);
        lineForce->setEnableForce(true);

        mTopLevelGroup->addChild(lineForce);
        mTopLevelGroup->connect(mTopLevelBody->addLink("airSpring"),
                                lineForce->getPort("link0"));
        mTopLevelGroup->connect(arm->addLink("airSpring"),
                                lineForce->getPort("link1"));
        /// FIXME that ordering in attachment is messy!
        Vector3 asMnt0 = locationData((*it)->getElement("springMount0"),
                                      compressJointPos -
                                      Unit::inch().convertTo(Vector3(0.1, 0, 0.5)));
        Vector3 asMnt1 = locationData((*it)->getElement("springMount1"),
                                      compressJointPos +
                                      Unit::inch().convertTo(Vector3(-0.5, 0, 0)));
        lineForce->setPosition0(structToBody(asMnt0));
        lineForce->setPosition1(structToBody(asMnt1));
        
        // The damper element
        const XMLElement* airSpringElem = (*it)->getElement("damper");
        AirSpring* aoDamp = getAirSpring(airSpringElem, name);
        addMultiBodyModel(aoDamp);
        // That one reads the joint position and velocity ...
        mTopLevelGroup->connect(lineForce->getPort("distance"),
                                aoDamp->getPort("position"));
        mTopLevelGroup->connect(lineForce->getPort("velocity"),
                                aoDamp->getPort("velocity"));
        // ... and provides an output force
        mTopLevelGroup->connect(aoDamp->getPort("force"),
                                lineForce->getPort("force"));
        
        // Attach a wheel to that strut part.
        attachWheel((*it)->getElement("wheel"), name, numStr, arm);
        
        const Port* port = rj->getPort("position");
        addOutputModel(port, "Gear " + numStr + " Compression",
                       "gear/gear[" + numStr + "]/compression-rad");
        
        /// FIXME add a retract joint ...
        addOutputModel("gear/gear-pos-norm", "Gear " + numStr + " Position",
                       "gear/gear[" + numStr + "]/position-norm");
        
      } else if (type == "TAILHOOK") {
//         const XMLElement* tailhookElem = (*it);
//         std::string name = (*it)->getAttribute("name");

//         Tailhook* tailhook = new Tailhook(name + " Tailhook");
//         mTopLevelGroup->addChild(tailhook);
//         real_type length = realData(tailhookElem->getElement("length"), 0.5);
//         tailhook->setLength(length);
//         real_type upAngle = realData(tailhookElem->getElement("upAngle"), 10);
//         tailhook->setUpAngle(Unit::degree().convertFrom(upAngle));
//         real_type downAngle = realData(tailhookElem->getElement("downAngle"), -75);
//         tailhook->setDownAngle(Unit::degree().convertFrom(downAngle));
//         Vector3 loc = structToBody(locationData(tailhookElem->getElement("location")));
//         tailhook->setPosition(loc);
//         addMultiBodyModel(tailhook);

//         mVehicle->getTopBody()->addInteract(tailhook);
        
//         if (!connectJSBExpression("/controls/gear/tailhook",
//                                   tailhook->getInputPort(0)))
//           return error("could not connect to tailhook command");
        
//         // expose the tailhook position
//         const Port* port = tailhook->getOutputPort(0);
//         std::string nameBase = "Tailhook Position";
//         addOutputModel(port, nameBase, "gear/tailhook/position-rad");
//         port = addToUnit(nameBase + " converter", Unit::degree(), port);
//         addOutputModel(port, nameBase + " Deg", "gear/tailhook/position-deg");

      } else {
        return error("Unknown groundreactions component of type " + type);
      }
    } else if ((*it)->getName() == "documentation") {
    } else {
      return error("Unknown groundreactions tag " + (*it)->getName());
    }
  }

  return true;
}

bool
JSBSimReader::convertPropulsion(const XMLElement* pElem)
{
  unsigned engineNumber = 0;
  unsigned tankNumber = 0;

  std::list<const XMLElement*> elems = pElem->getElements();
  std::list<const XMLElement*>::const_iterator it;
  for (it = elems.begin(); it != elems.end(); ++it) {
    if ((*it)->getName() == "engine") {
      std::stringstream sstr;
      sstr << engineNumber++;
      if (!convertEngine(*it, sstr.str()))
        return error("Cannot parse engine");
    }
    else if ((*it)->getName() == "tank") {
      std::stringstream sstr;
      sstr << tankNumber++;
      if (!convertTank(*it, sstr.str()))
        return error("Cannot parse tank");
    }
    else if ((*it)->getName() == "documentation") {
    }
    else
      return error("Unexpected PROPULSION element \"" + (*it)->getName() + "\"");
  }

  return true;
}

bool
JSBSimReader::convertTank(const XMLElement* tElem, const std::string& number)
{
//          <tank type="FUEL">    <!-- Tank number 0 -->
//             <location unit="IN">
//                 <x> 600 </x>
//                 <y> -90 </y>
//                 <z> -20 </z>
//             </location>
//             <capacity unit="LBS"> 23940 </capacity>
//             <contents unit="LBS"> 15000 </contents>
//         </tank>
  return true;
}

bool
JSBSimReader::convertThruster(const XMLElement* tElem, const std::string& number)
{
//            <thruster file="direct">
//                 <location unit="IN">
//                     <x> 670 </x>
//                     <y> 200 </y>
//                     <z> -45 </z>
//                 </location>
//                 <orient unit="DEG">
//                     <roll> 0.0 </roll>
//                     <pitch> 0.0 </pitch>
//                     <yaw> 0.0 </yaw>
//                 </orient>
//             </thruster>
  return true;
}

bool
JSBSimReader::convertEngine(const XMLElement* engine,
                            const std::string& number)
{
  Vector3 loc
    = locationData(engine->getElement("location"), Vector3(0, 0, 0));
  Vector3 orient
    = orientationData(engine->getElement("orient"), Vector3(0, 0, 0));

  loc = structToBody(loc);
  Quaternion orientation
    = Quaternion::fromYawPitchRoll(orient(2), orient(1), orient(0));


  // Engines are distinguished between that turbine model which effectively
  // moslty produces thrust by itself and the ones having a propeller and an
  // engine driving that.
  // The first one has its own single model.
  // For the second, the engine is modelled with a joint force/driver
  // the propeller with an aerodynamic table lookup system.
  std::string engineName = engine->getAttribute("file");
  std::string eFileName = engineName + ".xml";
  std::ifstream eFileStream;
  if (!openFile(mEnginePath, eFileName, eFileStream))
    return error("Cannot find engine \"" + engineName + "\"");

  SharedPtr<XMLElement> engineTopElem = parseXMLStream(eFileStream);
  if (!engineTopElem)
    return error("No toplevel xml element found for engine \""
                 + engineName + "\"");

  if (engineTopElem->getName() == "turbine_engine") {
    if (!convertTurbine(engineTopElem, number, loc, orientation, 0))
      return error("Error readinge turbine configuration");
    
  } else if (engineTopElem->getName() == "turboprop_engine") {
    if (!convertTurboProp(engineTopElem, number, loc, orientation, 0))
      return error("Error readinge turbine configuration");
    
  } else if (engineTopElem->getName() == "piston_engine") {
    if (!convertPiston(engineTopElem, number, 0))
      return error("Error readinge piston configuration");

//   } else if (engineTopElem->getName() == "rocket_engine") {
//     return error("FG_ROCKET's are not (yet?) supported!");

  } else if (engineTopElem->getName() == "electric_engine") {
    if (!convertElectric(engineTopElem, number, 0))
      return error("Error readinge electric configuration");

  } else
    return error("Unknown toplevel xml element for engine file \""
                 + eFileName + "\"");


//   // Thruster
//   const Element* thruster = engine->getElement("AC_THRUSTER");
//   std::string thrusterName = thruster->getAttribute("FILE");
//   std::string tFileName = thrusterName + ".xml";
//   std::ifstream tFileStream;
//   if (!openFile(mEnginePath, tFileName, tFileStream))
//     return error("Cannot find thruster \"" + thrusterName + "\"");

//   SharedPtr<XMLElement> thrusterTopElem = parseXMLStream(tFileStream);

//   if (!thrusterTopElem)
//     return error("No toplevel xml element found for thruster \""
//                  + thrusterName + "\"");

//   if (thrusterTopElem->getName() == "FG_DIRECT") {
//   } else if (thrusterTopElem->getName() == "FG_NOZZLE") {
//   } else if (thrusterTopElem->getName() == "FG_PROPELLER") {
//   } else
//     return error("Unknown toplevel xml element for thruster file \""
//                  + tFileName + "\"");













// //   RevoluteJoint* wj = new RevoluteJoint(name + " Wheel Joint");
// //   parent->addInteract(wj);
// //   wheel->setInLDoboardJoint(wj);
// //   wj->setAxis(Vector3(0, 1, 0));
// //   wj->setPosition(pos);
// //   wj->setInitialPosition(0);
// //   wj->setInitialVelocity(0);

// //     DiscBrake* brakeF = new DiscBrake(name + " Brake Force");
// //     brakeF->setMinForce(8e1);
// //     brakeF->setMaxForce(1e4);
// //     if (brake == "LEFT") {
// //       const Port* port = lookupJSBExpression("gear/left-brake-cmd-norm");
// //       brakeF->getInputPort(0)->connect(port);
// //     } else if (brake == "RIGHT") {
// //       const Port* port = lookupJSBExpression("gear/right-brake-cmd-norm");
// //       brakeF->getInputPort(0)->connect(port);
// //     }
// //     // That one reads the joint position and velocity ...
// //     brakeF->getInputPort(1)->connect(wj->getOutputPort(1));
// //     // ... and provides an output force
// //     wj->getInputPort(0)->connect(brakeF->getOutputPort(0));
// //     addMultiBodyModel(brakeF);






// //   std::string namestr = "Engine<" + number + ">";
// //   ConstModel* fullForce = new ConstModel(namestr + " full",
// //                                          Vector6(0, 0, 0, 4.4*1.5e4, 0, 0));
// //   addMultiBodyModel(fullForce);

// //   Product* prod = new Product(namestr + " modulation");
// //   std::string throttlename = "fcs/throttle-cmd-norm[" + number + "]";
// //   prod->getInputPort(0)->connect(lookupJSBExpression(throttlename));
// //   prod->getInputPort(1)->connect(fullForce->getOutputPort(0));
// //   addMultiBodyModel(prod);

// //   ExternalForceModel* engineForce = new ExternalForceModel(namestr);
// //   engineForce->setPosition(structToBody(loc));
// //   engineForce->setOrientation(orientation);

// //   engineForce->getInputPort(0)->connect(prod->getOutputPort(0));

// //   mVehicle->getTopBody()->addInteract(engineForce);

  return true;
}

bool
JSBSimReader::convertTurbine(const XMLElement* turbine,
                             const std::string& number,
                             const Vector3& pos,
                             const Quaternion& orientation,
                             const Port* thrusterDriver)
{
  // At the moment we have a *very* insufficient engine, just modulate the
  // thrust between 0 and max
  real_type maxThrust = realData(turbine->getElement("milthrust"), 0);

  std::string namestr = "Engine<" + number + ">";
  real_type fullThrustN = Unit::lbf().convertFrom(maxThrust);
  ConstModel* fullForce = new ConstModel(namestr + " full", fullThrustN);
  addMultiBodyModel(fullForce);

  Product* prod = new Product(namestr + " modulation");
  addMultiBodyModel(prod);
  std::string throttlename = "fcs/throttle-cmd-norm[" + number + "]";
  if (!connectJSBExpression(throttlename, prod->getPort("input0")))
    return error("could not connect to throttle command");
  mTopLevelGroup->connect(fullForce->getPort("output"),
                          prod->getPort("input1"));

  ConstModel* zeroComponent = new ConstModel(namestr + " zero", 0);
  addMultiBodyModel(zeroComponent);

  MatrixConcat* force = new MatrixConcat(namestr + " Force");
  addMultiBodyModel(force);

  mTopLevelGroup->connect(prod->getPort("output"),
                          force->addInputPort("x"));
  mTopLevelGroup->connect(zeroComponent->getPort("output"),
                          force->addInputPort("y"));
  mTopLevelGroup->connect(zeroComponent->getPort("output"),
                          force->addInputPort("z"));

  ExternalInteract* engineForce = new ExternalInteract(namestr);
  mTopLevelGroup->addChild(engineForce);
  engineForce->setEnableForce(true);
  mTopLevelGroup->connect(mTopLevelBody->addLink("engine"),
                          engineForce->getPort("link"));
  engineForce->setPosition(pos);
  engineForce->setOrientation(orientation);
  mTopLevelGroup->connect(force->getPort("output"),
                          engineForce->getPort("force"));

  return true;
}

bool
JSBSimReader::convertTurboProp(const XMLElement* turbine,
                               const std::string& number,
                               const Vector3& pos,
                               const Quaternion& orientation,
                               const Port* thrusterDriver)
{
  std::cout << "Skipping turboprop engine!" << std::endl;
  return true;
}

bool
JSBSimReader::convertElectric(const XMLElement* turbine,
                              const std::string& number,
                              const Port* thrusterDriver)
{
  std::cout << "Skipping electric engine!" << std::endl;
  return true;
}

bool
JSBSimReader::convertPiston(const XMLElement* turbine,
                            const std::string& number,
                            const Port* thrusterDriver)
{
  std::cout << "Skipping piston engine!" << std::endl;
  return true;
}

bool
JSBSimReader::convertFCSList(const XMLElement* fcsElem)
{
  std::list<const XMLElement*> comps = fcsElem->getElements();
  std::list<const XMLElement*>::const_iterator it;
  for (it = comps.begin(); it != comps.end(); ++it) {
    if ((*it)->getName() == "channel") {
      if (!convertFCSList(*it))
        return error("Cannot convert FCS channel \""
                     + (*it)->getAttribute("name") + "\"");
    } else {
      if (!convertFCSComponent(*it))
        return error("Cannot convert FCS component \""
                     + (*it)->getAttribute("name") + "\"");
    }
  }

  return true;
}

bool
JSBSimReader::convertFCSComponent(const XMLElement* fcsComponent)
{
  // The model we put into the fcs group in the end ...
  SharedPtr<Node> model;

  // The final output property.
  SharedPtr<const Port> out;

  // JSBSim FCS output values contain some implicit rules.
  // From the component name a default output property is formed.
  // If we find an OUTPUT line this is the output value too.
  // So collect them first and register them later when the final output
  // expression is known.
  std::string name = fcsComponent->getAttribute("name");
  std::string type = fcsComponent->getName();

  if (type == "SUMMER" || type == "summer") {
    SharedPtr<Summer> summer = new Summer(name);
    addFCSModel(summer);
    std::list<std::string> inputs = getInputs(fcsComponent);
    std::list<std::string>::iterator it = inputs.begin();
    unsigned n = 0;
    while (it != inputs.end()) {
      summer->setNumSummands(n+1);
      if (!connectJSBExpression(*it, summer->getInputPort(n)))
        return error("could not connect to summer input \"" + (*it) + "\"");
      ++n;
      ++it;
    }
    real_type b = realData(fcsComponent->getElement("bias"), 0);
    if (b != 0) {
      SharedPtr<Bias> bias = new Bias(name + " Bias");
      addFCSModel(bias);
      mTopLevelGroup->connect(summer->getOutputPort(), bias->getInputPort(0));
      Matrix m(1, 1);
      m(0, 0) = b;
      bias->setBias(m);
      model = bias;
    } else {
      model = summer;
    }
    // FIXME clipto is missing ...
    out = model->getPort("output");
  } else if (type == "DEADBAND" || type == "deadband") {
    SharedPtr<DeadBand> deadband = new DeadBand(name);
    addFCSModel(deadband);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, deadband->getInputPort(0)))
      return error("could not connect to deadband input \"" + token + "\"");
    deadband->setWidth(realData(fcsComponent->getElement("width"), 0));
    model = deadband;
    out = model->getPort("output");

  } else if (type == "GRADIENT" || type == "gradient") {
    SharedPtr<TimeDerivative> timeDeriv = new TimeDerivative(name);
    addFCSModel(timeDeriv);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, timeDeriv->getPort("input")))
      return error("could not connect to gradient input \"" + token + "\"");
    model = timeDeriv;
    out = model->getPort("output");

  } else if (type == "SWITCH" || type == "switch") {
    SharedPtr<JSBSimSwitch> sw = new JSBSimSwitch(name);
    addFCSModel(sw->getGroup());

    model = sw->getGroup();
    out = sw->getOutputPort();

  } else if (type == "KINEMAT" || type == "kinematic") {
    // Use that special proxy class
    SharedPtr<JSBSimKinemat> kinemat;
    kinemat = new JSBSimKinemat(name);

    real_type minVal = Limits<real_type>::max();
    real_type maxVal = -Limits<real_type>::max();
    real_type allTime = 0;
    const XMLElement* traverse = fcsComponent->getElement("traverse");
    std::list<const XMLElement*> setting = traverse->getElements("setting");
    std::list<const XMLElement*>::iterator it = setting.begin();
    while (it != setting.end()) {
      real_type val = realData((*it)->getElement("position"), 0);
      minVal = min(minVal, val);
      maxVal = max(maxVal, val);
      if (it != setting.begin())
        allTime += realData((*it)->getElement("time"), 0);
      ++it;
    }
    if (allTime != 0)
      kinemat->setRateLimit(fabs((maxVal-minVal)/allTime));
    else
      kinemat->setRateLimit(Limits<real_type>::max());
    kinemat->setMinValue(minVal);
    kinemat->setMaxValue(maxVal);
    kinemat->setNoScale(fcsComponent->getElement("noscale"));
    model = kinemat->getGroup();
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getPort("input")))
      return error("could not connect to kinemat input \"" + token + "\"");
    out = kinemat->getOutputPort();

  } else if (type == "PURE_GAIN" || type == "pure_gain") {
    SharedPtr<Gain> gain = new Gain(name);
    model = gain;
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, gain->getInputPort(0)))
      return error("could not connect to gain input \"" + token + "\"");
    gain->setGain(realData(fcsComponent->getElement("gain"), 1));
    out = model->getPort("output");

  } else if (type == "AEROSURFACE_SCALE" || type == "aerosurface_scale") {
    // An AEROSURFACE_SCALE component is done with n input saturation clipping
    // the input from -1 to 1. This is the one which is magically mapped to
    // an output property in /surface-positions/...
    // This one's output is directly connected to a lookup table
    SharedPtr<JSBSimAerosurfaceScale> asScale;
    asScale = new JSBSimAerosurfaceScale(name);
    const XMLElement* elem = fcsComponent->getElement("domain");
    if (elem) {
      asScale->setMinDomain(realData(elem->getElement("min"), -1));
      asScale->setMaxDomain(realData(elem->getElement("max"), 1));
    } else {
      asScale->setMinDomain(-1);
      asScale->setMaxDomain(1);
    }
    elem = fcsComponent->getElement("range");
    if (elem) {
      asScale->setMinValue(realData(elem->getElement("min"), -1));
      asScale->setMaxValue(realData(elem->getElement("max"), 1));
    } else {
      asScale->setMinValue(-1);
      asScale->setMaxValue(1);
    }
    std::string token = stringData(fcsComponent->getElement("zero_centered"));
    asScale->setCentered(!(token == "0" || token == "false"));
    asScale->setGain(realData(fcsComponent->getElement("gain"), 1));
    model = asScale->getGroup();
    addFCSModel(model);
    token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getPort("input")))
      return error("could not connect to gain input \"" + token + "\"");
    out = asScale->getOutputPort();

  } else if (type == "SCHEDULED_GAIN" || type == "scheduled_gain") {
    SharedPtr<JSBSimScheduledGain> sGain = new JSBSimScheduledGain(name);
    const XMLElement* tbl = fcsComponent->getElement("table");
    if (!tbl)
      return error("Table without data");
    if (1 != getNumTableDims(tbl))
      return error("Table for scheduled gain is not 1 dimensional");

    TableData<1> data;
    BreakPointVector lookup;
    if (!readTable1D(tbl, data, lookup))
      return error("Cannot read table");
    sGain->setTableData(data, lookup);

    model = sGain->getGroup();
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getPort("input")))
      return error("could not connect to gain input \"" + token + "\"");
    token = stringData(tbl->getElement("independentVar"));
    if (!connectJSBExpression(token, model->getPort("scheduleInput")))
      return error("could not connect to table input \"" + token + "\"");

    out = sGain->getOutputPort();

  } else if (type == "INTEGRATOR" || type == "integrator") {
    model = new DiscreteIntegrator(name);
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getPort("input")))
      return error("could not connect to integrator input \"" + token + "\"");
    out = model->getPort("output");
    real_type c1 = realData(fcsComponent->getElement("c1"), 1);
    if (c1 != 1) {
      SharedPtr<Gain> gain = new Gain(name + " Gain");
      addFCSModel(gain);
      mTopLevelGroup->connect(model->getPort("output"), gain->getInputPort(0));
      gain->setGain(c1);
      model = gain;
      out = model->getPort("output");
    }

  } else if (type == "LAG_FILTER" || type == "lag_filter") {
    //   C1
    // ------
    // s + C1
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    model = discreteTransfFunc;
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getPort("input")))
      return error("could not connect to transfer function input \""
                   + token + "\"");
    Vector v(1);
    v(1) = realData(fcsComponent->getElement("c1"), 1);
    discreteTransfFunc->setNumerator(v);
    v.resize(2);
    v(1) = 1;
    v(2) = realData(fcsComponent->getElement("c1"), 1);
    discreteTransfFunc->setDenominator(v);
    out = model->getPort("output");

  } else if (type == "LEAD_LAG_FILTER" || type == "lead_lag_filter") {
    // C1*s + C2
    // ---------
    // C3*s + C4
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    model = discreteTransfFunc;
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getPort("input")))
      return error("could not connect to transfer function input \""
                   + token + "\"");
    Vector v(2);
    v(1) = realData(fcsComponent->getElement("c1"), 1);
    v(2) = realData(fcsComponent->getElement("c2"), 1);
    discreteTransfFunc->setNumerator(v);
    v(1) = realData(fcsComponent->getElement("c3"), 1);
    v(2) = realData(fcsComponent->getElement("c4"), 1);
    discreteTransfFunc->setDenominator(v);

    out = model->getPort("output");

  } else if (type == "WASHOUT_FILTER" || type == "washout_filter") {
    //   s
    // ------
    // s + C1
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    model = discreteTransfFunc;
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getPort("input")))
      return error("could not connect to transfer function input \""
                   + token + "\"");
    Vector v(1);
    v(1) = 1;
    discreteTransfFunc->setNumerator(v);
    v.resize(2);
    v(1) = 1;
    v(2) = realData(fcsComponent->getElement("c1"), 1);
    discreteTransfFunc->setDenominator(v);
    out = model->getPort("output");

  } else if (type == "SECOND_ORDER_FILTER" || type == "second_order_filter") {
    // C1*s + C2*s + C3
    // ----------------
    // C4*s + C5*s + C6
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    model = discreteTransfFunc;
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getPort("input")))
      return error("could not connect to transfer function input \""
                   + token + "\"");
    Vector v(3);
    v(1) = realData(fcsComponent->getElement("c1"), 1);
    v(2) = realData(fcsComponent->getElement("c2"), 1);
    v(3) = realData(fcsComponent->getElement("c3"), 1);
    discreteTransfFunc->setNumerator(v);
    v(1) = realData(fcsComponent->getElement("c4"), 1);
    v(2) = realData(fcsComponent->getElement("c5"), 1);
    v(3) = realData(fcsComponent->getElement("c6"), 1);
    discreteTransfFunc->setDenominator(v);

    out = model->getPort("output");

  } else if (type == "FCS_FUNCTION" || type == "fcs_function") {
    SharedPtr<Summer> summer = new Summer(name);
    addFCSModel(summer);
    
    if (!convertFunction(fcsComponent->getElement("function"), summer))
      return error("could not read fcs_function \"" + name + "\"");
    model = summer;
    out = summer->getOutputPort();

  } else if (type == "PID" || type == "pid") {
    SharedPtr<JSBSimPID> pid = new JSBSimPID(name);
    addFCSModel(pid->getGroup());

    std::string data = stringData(fcsComponent->getElement("ki"));
    if (!connectJSBExpression(data, pid->getKIPort())){
      pid->setKI(realData(fcsComponent->getElement("ki"), 0));
    }

    data = stringData(fcsComponent->getElement("kp"));
    if (!connectJSBExpression(data, pid->getKPPort())){
      pid->setKP(realData(fcsComponent->getElement("kp"), 0));
    }

    data = stringData(fcsComponent->getElement("kd"));
    if (!connectJSBExpression(data, pid->getKDPort())){
      pid->setKD(realData(fcsComponent->getElement("kd"), 0));
    }
    model = pid->getGroup();
    out = pid->getOutputPort();

  } else if (type == "PROPERTY" || type == "property") {
    name = fcsComponent->getData();
    real_type value = asciiToReal(fcsComponent->getAttribute("value"), 0);
    SharedPtr<ConstModel> constModel;
    constModel = new ConstModel("Property " + name, value);
    addFCSModel(constModel);
    model = constModel;
    out = constModel->getPort("output");

  } else if (type == "SENSOR" || type == "sensor") {
    std::cout << "Ignoring SENSOR" << std::endl;

  } else if (type == "ACCELEROMETER" || type == "accelerometer") {
    std::cout << "Ignoring ACCELEROMETER" << std::endl;

  } else if (type == "ACTUATOR" || type == "actuator") {
    SharedPtr<JSBSimActuator> actuator = new JSBSimActuator(name);
    addFCSModel(actuator->getGroup());

    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, actuator->getInputPort()))
      return error("could not connect to actuator input \"" + token + "\"");

    model = actuator->getGroup();
    out = actuator->getOutputPort();

  } else if (type == "documentation") {
  } else
    return error("Unknown FCS COMPONENT type: \"" + type
                 + "\". Ignoring whole FCS component \"" + name + "\"" );

  // Register output property names.
  std::string implicitOutname = std::string("fcs/")
    + normalizeComponentName(name);
  registerJSBExpression(implicitOutname, out);
  std::string outname = stringData(fcsComponent->getElement("output"));
  if (!outname.empty() &&
      canonicalJSBProperty(outname) != canonicalJSBProperty(implicitOutname))
    registerJSBExpression(outname, out);

  return true;
}

bool
JSBSimReader::convertSystem(const XMLElement* system)
{
  std::string systemName = system->getAttribute("file");
  std::string sFileName = systemName + ".xml";
  std::ifstream sFileStream;
  if (!openFile(mSystemPath, sFileName, sFileStream))
    return error("Cannot find system \"" + systemName + "\"");

  SharedPtr<XMLElement> systemTopElem = parseXMLStream(sFileStream);

  if (!convertFCSList(systemTopElem))
    return error("Cannot convert system file \"" + sFileName + "\"");

  return true;
}

bool
JSBSimReader::convertAerodynamics(const XMLElement* aero)
{
  std::list<const XMLElement*> elems = aero->getElements();
  std::list<const XMLElement*>::const_iterator it;
  for (it = elems.begin(); it != elems.end(); ++it) {
    if ((*it)->getName() == "function") {
      if (!convertFunction(*it, 0))
        return error("Cannot convert function");

    } else if ((*it)->getName() == "axis") {
      std::string axisname = (*it)->getAttribute("name");
      
      SharedPtr<Summer> sum = new Summer(axisname + " Sum");
      addMultiBodyModel(sum);
      sum->setNumSummands(0);
      
      // Now parse the summands
      std::list<const XMLElement*> funcs = (*it)->getElements();
      std::list<const XMLElement*>::const_iterator fit;
      for (fit = funcs.begin(); fit != funcs.end(); ++fit) {
        if (!convertFunction(*fit, sum))
          return error("Cannot convert function");
      }

      if (!sum->getNumSummands())
        // FIXME connect that then with a zero port ...
        continue;
      const Port* port = sum->getPort("output");

      if (axisname == "LIFT") {
        port = addFromUnit("LIFT unit convert", Unit::lbf(), port);
        mTopLevelGroup->connect(port, mAeroForce->getLiftPort());
      } else if (axisname == "DRAG") {
        port = addFromUnit("DRAG unit convert", Unit::lbf(), port);
        mTopLevelGroup->connect(port, mAeroForce->getDragPort());
      } else if (axisname == "SIDE") {
        port = addFromUnit("SIDE unit convert", Unit::lbf(), port);
        mTopLevelGroup->connect(port, mAeroForce->getSidePort());
      } else if (axisname == "ROLL") {
        port = addFromUnit("ROLL unit convert", Unit::lbfFoot(), port);
        mTopLevelGroup->connect(port, mAeroForce->getRollPort());
      } else if (axisname == "PITCH") {
        port = addFromUnit("PITCH unit convert", Unit::lbfFoot(), port);
        mTopLevelGroup->connect(port, mAeroForce->getPitchPort());
      } else if (axisname == "YAW") {
        port = addFromUnit("YAW unit convert", Unit::lbfFoot(), port);
        mTopLevelGroup->connect(port, mAeroForce->getYawPort());
      } else
        return error("Unknown aerodynamic axis!");
    }
  }

  return true;
}

bool
JSBSimReader::convertFunction(const XMLElement* function, Summer* sum)
{
  std::string bindName = function->getAttribute("name");
  std::string name = bindName;
  std::string::size_type slachPos = bindName.rfind('/');
  if (slachPos != std::string::npos)
    name = name.substr(slachPos+1);

  SharedPtr<Group> group = new Group(name);
  mTopLevelGroup->addChild(group);
  GroupOutput* groupOutput = new GroupOutput("Output");
  group->addChild(groupOutput);
  
  std::list<const XMLElement*> elements = function->getElements();
  std::list<const XMLElement*>::iterator i;
  for (i = elements.begin(); i != elements.end(); ++i) {
    if ((*i)->getName() == "description")
      continue;
    if (!connectFunctionInput(*i, groupOutput->getPort("input"), group))
      return error("Can not connect product input!");
    // FIXME check that we get here only once
  }

  if (!bindName.empty())
    registerJSBExpression(bindName, group->getPort(groupOutput->getExternalPortIndex()));
  if (sum) {
    unsigned num = sum->getNumSummands();
    sum->setNumSummands(num+1);
    mTopLevelGroup->connect(group->getPort(groupOutput->getExternalPortIndex()), sum->getInputPort(num));
  }

  return true;
}

bool
JSBSimReader::connectUnaryFunctionInput(const std::string& name,
                                        UnaryFunction::Type type,
                                        const XMLElement* element,
                                        const Port* port, Group* parentGroup)
{
  SharedPtr<UnaryFunction> unaryFunction;
  unaryFunction = new UnaryFunction(name, type);
  parentGroup->addChild(unaryFunction);
  
  if (!parentGroup->connect(port, unaryFunction->getOutputPort()))
    return error("Can not connect UnaryFunction output to port!");
  
  std::list<const XMLElement*> elements = element->getElements();
  if (elements.size() != 1)
    return error("Input ports to Unary models must have exactly one input!");
  if (!connectFunctionInput(elements.front(), unaryFunction->getInputPort(0),
                            parentGroup))
    return error("Can not connect UnaryFunction input to port!");
  
  return true;
}


bool
JSBSimReader::connectBinaryFunctionInput(const std::string& name,
                                         BinaryFunction::Type type,
                                         const XMLElement* element,
                                         const Port* port, Group* parentGroup)
{
  SharedPtr<BinaryFunction> binaryFunction;
  binaryFunction = new BinaryFunction(name, type);
  parentGroup->addChild(binaryFunction);
  
  if (!parentGroup->connect(port, binaryFunction->getOutputPort()))
    return error("Can not connect BinaryFunction output to port!");
  
  std::list<const XMLElement*> elements = element->getElements();
  if (elements.size() != 2)
    return error("Input ports to Binary models must have exactly two inputs!");
  if (!connectFunctionInput(elements.front(), binaryFunction->getInputPort(0),
                            parentGroup))
    return error("Can not connect BinaryFunction input to port0!");
  if (!connectFunctionInput(elements.back(), binaryFunction->getInputPort(1),
                            parentGroup))
    return error("Can not connect BinaryFunction input to port1!");
  
  return true;
}

bool
JSBSimReader::connectFunctionInput(const XMLElement* element, const Port* port,
                                   Group* parentGroup)
{
  if (element->getName() == "abs") {
    if (!connectUnaryFunctionInput("abs", UnaryFunction::Abs,
                                   element, port, parentGroup))
        return error("Can not connect abs output to port!");
    return true;

  } else if (element->getName() == "acos") {
    if (!connectUnaryFunctionInput("acos", UnaryFunction::Acos,
                                   element, port, parentGroup))
        return error("Can not connect acos output to port!");
    return true;

  } else if (element->getName() == "asin") {
    if (!connectUnaryFunctionInput("asin", UnaryFunction::Asin,
                                   element, port, parentGroup))
        return error("Can not connect asin output to port!");
    return true;

  } else if (element->getName() == "atan") {
    if (!connectUnaryFunctionInput("atan", UnaryFunction::Atan,
                                   element, port, parentGroup))
        return error("Can not connect atan output to port!");
    return true;

  } else if (element->getName() == "atan2") {
    if (!connectBinaryFunctionInput("Atan2", BinaryFunction::Atan2,
                                    element, port, parentGroup))
        return error("Can not connect atan2 output to port!");
    return true;

  } else if (element->getName() == "ceil") {
    if (!connectUnaryFunctionInput("ceil", UnaryFunction::Ceil,
                                   element, port, parentGroup))
        return error("Can not connect ceil output to port!");
    return true;

  } else if (element->getName() == "cos") {
    if (!connectUnaryFunctionInput("cos", UnaryFunction::Cos,
                                   element, port, parentGroup))
        return error("Can not connect cos output to port!");
    return true;

  } else if (element->getName() == "difference") {
    SharedPtr<Summer> summer = new Summer("Difference");
    parentGroup->addChild(summer);

    SharedPtr<UnaryFunction> unaryFunction;
    unaryFunction = new UnaryFunction("Minus", UnaryFunction::Minus);
    parentGroup->addChild(unaryFunction);
    
    if (!parentGroup->connect(unaryFunction->getOutputPort(),
                              summer->getInputPort(1)))
      return error("Can not connect negative Summer input to port!");
    if (!parentGroup->connect(port, summer->getOutputPort()))
      return error("Can not connect Summer output to port!");
    
    std::list<const XMLElement*> elements = element->getElements();
    if (elements.size() != 2)
      return error("Input ports to Difference model must have exactly two inputs!");
    if (!connectFunctionInput(elements.front(), summer->getInputPort(0),
                              parentGroup))
      return error("Can not connect UnaryFunction input to port!");

    if (!connectFunctionInput(elements.back(), unaryFunction->getInputPort(0),
                              parentGroup))
      return error("Can not connect UnaryFunction input to port!");

    return true;

  } else if (element->getName() == "exp") {
    if (!connectUnaryFunctionInput("Exp", UnaryFunction::Exp,
                                   element, port, parentGroup))
        return error("Can not connect exp output to port!");
    return true;

  } else if (element->getName() == "floor") {
    if (!connectUnaryFunctionInput("Floor", UnaryFunction::Floor,
                                   element, port, parentGroup))
        return error("Can not connect floor output to port!");
    return true;

  } else if (element->getName() == "log") {
    if (!connectUnaryFunctionInput("Log", UnaryFunction::Log,
                                   element, port, parentGroup))
        return error("Can not connect log output to port!");
    return true;

  } else if (element->getName() == "log10") {
    if (!connectUnaryFunctionInput("Log10", UnaryFunction::Log10,
                                   element, port, parentGroup))
        return error("Can not connect log10 output to port!");
    return true;

  } else if (element->getName() == "log10") {
    if (!connectUnaryFunctionInput("Log10", UnaryFunction::Log10,
                                   element, port, parentGroup))
        return error("Can not connect log10 output to port!");
    return true;

  } else if (element->getName() == "pow") {
    if (!connectBinaryFunctionInput("pow", BinaryFunction::Pow,
                                    element, port, parentGroup))
        return error("Can not connect pow output to port!");
    return true;

  } else if (element->getName() == "product") {
    SharedPtr<Product> product = new Product("product");
    parentGroup->addChild(product);
  
    if (!parentGroup->connect(port, product->getOutputPort()))
      return error("Can not connect product output to port!");
  
    product->setNumFactors(0);
    std::list<const XMLElement*> elements = element->getElements();
    std::list<const XMLElement*>::iterator i;
    for (i = elements.begin(); i != elements.end(); ++i) {
      unsigned idx = product->getNumFactors();
      product->setNumFactors(idx + 1);
      if (!connectFunctionInput(*i, product->getInputPort(idx),
                                parentGroup))
        return error("Can not connect product input!");
    }
  
    return true;

  } else if (element->getName() == "property") {
    if (!connectJSBExpression(stringData(element), port))
        return error("Can not connect property to port!");
    return true;

  } else if (element->getName() == "quotient") {
    if (!connectBinaryFunctionInput("Div", BinaryFunction::Div,
                                    element, port, parentGroup))
        return error("Can not connect quotient output to port!");
    return true;

  } else if (element->getName() == "sin") {
    if (!connectUnaryFunctionInput("Sin", UnaryFunction::Sin,
                                   element, port, parentGroup))
        return error("Can not connect sin output to port!");
    return true;

  } else if (element->getName() == "sum") {
    SharedPtr<Summer> summer = new Summer("sum");
    parentGroup->addChild(summer);
  
    if (!parentGroup->connect(port, summer->getOutputPort()))
      return error("Can not connect sum output to port!");
  
    summer->setNumSummands(0);
    std::list<const XMLElement*> elements = element->getElements();
    std::list<const XMLElement*>::iterator i;
    for (i = elements.begin(); i != elements.end(); ++i) {
      unsigned idx = summer->getNumSummands();
      summer->setNumSummands(idx + 1);
      if (!connectFunctionInput(*i, summer->getInputPort(idx),
                                parentGroup))
        return error("Can not connect product input!");
    }
  
    return true;

  } else if (element->getName() == "sqr") {
    if (!connectUnaryFunctionInput("sqr", UnaryFunction::Sqr,
                                   element, port, parentGroup))
        return error("Can not connect sqr output to port!");
    return true;

  } else if (element->getName() == "sqrt") {
    if (!connectUnaryFunctionInput("sqrt", UnaryFunction::Sqrt,
                                   element, port, parentGroup))
        return error("Can not connect sqrt output to port!");
    return true;

  } else if (element->getName() == "table") {
    unsigned dim = getNumTableDims(element);
    if (dim == 1) {
      TableData<1> data;
      BreakPointVector lookup;
      if (!readTable1D(element, data, lookup))
        return error("Cannot read 1D table data.");
      std::string token = stringData(element->getElement("independentVar"));
      const Port* portP = getTablePrelookup("lookup", token, lookup);

      SharedPtr<Table1D> table = new Table1D("Table");
      parentGroup->addChild(table);
      table->setTableData(data);
      if (!JSBSimPropertyManager::connect(portP, mTopLevelGroup,
                                          table->getInputPort(0), parentGroup))
        return false;
      if (!parentGroup->connect(port, table->getPort("output")))
        return false;
      return true;

    } else if (dim == 2) {
      TableData<2> data;
      BreakPointVector lookup[2];
      if (!readTable2D(element, data, lookup))
        return error("Cannot read 2D table data.");
      
      std::list<const XMLElement*> indeps
        = element->getElements("independentVar");
      if (indeps.size() != 2)
        return error("2DTable data does not have 2 inputs!");
      std::string rowInput = indepData(indeps, "row");
      std::string colInput = indepData(indeps, "column");
      
      const Port* rPort = getTablePrelookup("Row lookup", rowInput, lookup[0]);
      const Port* cPort = getTablePrelookup("Column lookup", colInput, lookup[1]);
      
      SharedPtr<Table2D> table = new Table2D("Table");
      parentGroup->addChild(table);
      table->setTableData(data);

      if (!JSBSimPropertyManager::connect(rPort, mTopLevelGroup,
                                          table->getInputPort(0), parentGroup))
        return false;
      if (!JSBSimPropertyManager::connect(cPort, mTopLevelGroup,
                                          table->getInputPort(1), parentGroup))
        return false;
      if (!parentGroup->connect(port, table->getPort("output")))
        return false;
      return true;

    } else if (dim == 3) {
      TableData<3> data;
      BreakPointVector lookup[3];
      if (!readTable3D(element, data, lookup))
        return error("Cannot read 1D table data.");
      
      std::list<const XMLElement*> indeps
        = element->getElements("independentVar");
      if (indeps.size() != 3)
        return error("3DTable data does not have 3 inputs!");
      std::string rowInput = indepData(indeps, "row");
      std::string colInput = indepData(indeps, "column");
      std::string pageInput = indepData(indeps, "table");
      
      const Port* rPort = getTablePrelookup("Row lookup", rowInput, lookup[0]);
      const Port* cPort = getTablePrelookup("Column lookup", colInput, lookup[1]);
      const Port* pPort = getTablePrelookup("Page lookup", pageInput, lookup[2]);
      
      SharedPtr<Table3D> table = new Table3D("Table");
      parentGroup->addChild(table);
      table->setTableData(data);

      if (!JSBSimPropertyManager::connect(rPort, mTopLevelGroup,
                                          table->getInputPort(0), parentGroup))
        return false;
      if (!JSBSimPropertyManager::connect(cPort, mTopLevelGroup,
                                          table->getInputPort(1), parentGroup))
        return false;
      if (!JSBSimPropertyManager::connect(pPort, mTopLevelGroup,
                                          table->getInputPort(2), parentGroup))
        return false;
      if (!parentGroup->connect(port, table->getPort("output")))
        return false;

      return true;

    }

  } else if (element->getName() == "tan") {
    if (!connectUnaryFunctionInput("tan", UnaryFunction::Tan,
                                   element, port, parentGroup))
        return error("Can not connect tan output to port!");
    return true;

  } else if (element->getName() == "value") {
    real_type value = realData(element, 0);
    SharedPtr<ConstModel> constModel = new ConstModel("Value", value);
    parentGroup->addChild(constModel);
    if (!parentGroup->connect(port, constModel->getPort("output")))
      return error("Can not connect ConstModel output to port!");
    return true;

  }
  return false;
}

unsigned
JSBSimReader::getNumTableDims(const XMLElement* tableElem)
{
  if (!tableElem)
    return 0;
  std::list<const XMLElement*> indeps
    = tableElem->getElements("independentVar");
  return indeps.size();
}

bool
JSBSimReader::readTable1D(const XMLElement* tableElem,
                          TableData<1>& data, BreakPointVector& lookup)
{
  std::string input = stringData(tableElem->getElement("independentVar"));

  const XMLElement* td = tableElem->getElement("tableData");
  if (!td)
    return error("Table without tableData!");

  std::vector<real_type> values;
  std::stringstream stream(td->getData());
  while (stream) {
    real_type val;
    stream >> val;
    if (!stream)
      break;
    values.push_back(val);
  }
  
  if (values.size() % 2)
    return error("Inconsistent 1D table data!");

  unsigned sz = values.size()/2;
  TableData<1>::SizeVector sv;
  sv(0) = sz;
  data = TableData<1>(sv);
  for (unsigned idx = 0; idx < sz; ++idx) {
    lookup.insert(values[idx*2]);
    TableData<1>::Index iv;
    iv(0) = idx;
    data(iv) = values[idx*2+1];
  }
  return true;
}

bool
JSBSimReader::readTable2D(const XMLElement* tableElem,
                          TableData<2>& data, BreakPointVector lookup[2])
{
  std::list<const XMLElement*> indeps
    = tableElem->getElements("independentVar");
  if (indeps.size() != 2)
    return error("2DTable data does not have 2 inputs!");

  const XMLElement* td = tableElem->getElement("tableData");
  if (!td)
    return error("Table without tableData!");

  std::stringstream stream(td->getData());

  unsigned cols = 0;
  while (stream && cols == 0) {
    std::string line;
    std::getline(stream, line, '\n');
    std::stringstream lstream(line);
    for (; lstream; ++cols) {
      real_type in = 0;
      lstream >> in;
      if (!lstream)
        break;
      lookup[1].insert(in);
    }
  }
  std::vector<real_type> values;
  unsigned rows = 0;
  while(stream) {
    real_type val;
    stream >> val;
    if (!stream)
      break;
    lookup[0].insert(val);
    ++rows;

    for (unsigned i = 0; i < cols; ++i) {
      stream >> val;
      values.push_back(val);
    }
  }

  if (values.size() != rows*cols)
    return error("Invalid table size!");

  TableData<2>::SizeVector sv;
  sv(0) = rows;
  sv(1) = cols;
  data = TableData<2>(sv);
  for (unsigned i = 0; i < rows; ++i) {
    for (unsigned j = 0; j < cols; ++j) {
      TableData<2>::Index iv;
      iv(0) = i;
      iv(1) = j;
      data(iv) = values[i*cols+j];
    }
  }

  return lookup[0].size() == data.size(0) && lookup[1].size() == data.size(1);
}

bool
JSBSimReader::readTable3D(const XMLElement* tableElem,
                          TableData<3>& data, BreakPointVector lookup[3])
{
  return false;
//   std::list<const XMLElement*> indeps
//     = tableElem->getElements("independentVar");
//   std::string rowInput;
//   std::string colInput;
//   std::string pageInput;
//   if (indeps.size() == 1) {
//     rowInput = stringData(indeps.front());
//   } else {
//     rowInput = indepData(indeps, "row");
//     colInput = indepData(indeps, "column");
//     pageInput = indepData(indeps, "table");
//   }
}

} // namespace OpenFDM
