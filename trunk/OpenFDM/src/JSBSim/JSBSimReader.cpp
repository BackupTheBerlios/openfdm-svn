/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>
#include <stack>

#include <OpenFDM/Vector.h>
#include <OpenFDM/Matrix.h>
#include <OpenFDM/Quaternion.h>

#include <OpenFDM/AeroForce.h>
#include <OpenFDM/Bias.h>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/DeadBand.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/ExternalForceModel.h>
#include <OpenFDM/TransferFunction.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Input.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/Launchbar.h>
#include <OpenFDM/LinearSpringDamper.h>
#include <OpenFDM/MaxModel.h>
#include <OpenFDM/AirSpring.h>
#include <OpenFDM/PrismaticJoint.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/RevoluteActuator.h>
#include <OpenFDM/RevoluteJoint.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/Sensor.h>
#include <OpenFDM/SimpleContact.h>
#include <OpenFDM/SimpleGear.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/Table.h>
#include <OpenFDM/Tailhook.h>
#include <OpenFDM/TimeDerivative.h>
#include <OpenFDM/UnaryFunctionModel.h>
#include <OpenFDM/Units.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/WheelContact.h>
#include <OpenFDM/DiscBrake.h>

#include <OpenFDM/ReaderWriter.h>

#include "JSBSimAerosurfaceScale.h"
#include "JSBSimKinemat.h"
#include "JSBSimScheduledGain.h"

#include "JSBSimReader.h"

namespace OpenFDM {

static real_type
realData(const XMLElement* element, real_type def = 0)
{
  if (!element)
    return def;
  std::stringstream stream(element->getData());
  real_type value;
  stream >> value;
  if (!stream)
    return def;
  return value;
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
    stream >> value(1);
    //   if (!stream)
    //     return def;
  }
  {
    std::stringstream stream(yElement->getData());
    stream >> value(2);
    //   if (!stream)
    //     return def;
  }
  {
    std::stringstream stream(zElement->getData());
    stream >> value(3);
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
    stream >> value(1);
    //   if (!stream)
    //     return def;
  }
  {
    std::stringstream stream(yElement->getData());
    stream >> value(2);
    //   if (!stream)
    //     return def;
  }
  {
    std::stringstream stream(zElement->getData());
    stream >> value(3);
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
  I(1, 1) = realData(element->getElement("ixx"), def(1, 1));
  I(1, 2) = realData(element->getElement("ixy"), def(1, 2));
  I(1, 3) = realData(element->getElement("ixz"), def(1, 3));
  I(2, 2) = realData(element->getElement("iyy"), def(2, 2));
  I(2, 3) = realData(element->getElement("iyz"), def(2, 3));
  I(3, 3) = realData(element->getElement("izz"), def(3, 3));
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
  // Throw away any possibly loaded vehicle
  mVehicle = 0;
}

bool
JSBSimReader::loadAircraft(const std::string& acFileName)
{
  // Reset the vehicle.
  resetErrorState();

  mExpressionTable.clear();
  // Allocate a new vehicle
  mVehicle = new Vehicle;
  mAeroForce = new AeroForce("Aerodynamic force");
  mVehicle->getMultiBodySystem()->addModel(mAeroForce);
  mVehicle->getTopBody()->addInteract(mAeroForce);

  // Default discrete stepsize of JSBSim
  mVehicle->getModelGroup()->addSampleTime(SampleTime(1.0/120));

  // Try to find the given file on the given search path
  std::ifstream acFileStream;
  if (!openFile(mAircraftPath, acFileName, acFileStream))
    return false;

  // Get a parser FIXME
  return convertDocument(parseXMLStream(acFileStream));
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

  // Parse the propulsion section.
  const XMLElement* propulsionElem = topElem->getElement("propulsion");
  if (propulsionElem) {
    if (!convertPropulsion(propulsionElem))
      return error("Cannot convert PROPULSION data");
  }
  
  // Convert the aerodynamic force.
  const XMLElement* aeroElem = topElem->getElement("aerodynamics");
  if (aeroElem) {
    if (!convertAerodynamics(aeroElem))
      return error("Cannot convert AERODYNAMICS elements");
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
  mAeroForce->setWingArea(convertFrom(uFoot2, wingarea));

  real_type wingspan = realData(metricsElem->getElement("wingspan"), 0);
  mAeroForce->setWingSpan(convertFrom(uFoot, wingspan));

  real_type chord = realData(metricsElem->getElement("chord"), 0);
  /// FIXME wrong written
  mAeroForce->setCoord(convertFrom(uFoot, chord));

  const XMLElement* iwElem = metricsElem->getElement("wing_incidence");
  if (iwElem) {
    real_type iw = realData(iwElem, 0);
    PortProvider* port = addMultiBodyConstModel("Wing Incidence Constant", iw);
    registerJSBExpression("metrics/iw-deg", port);
  }

  const XMLElement* htareaElem = metricsElem->getElement("htailarea");
  if (htareaElem) {
    real_type htailarea = realData(htareaElem, 0);
    PortProvider* port = addMultiBodyConstModel("HTail Area Constant", htailarea);
    registerJSBExpression("metrics/Sh-sqft", port);
  }

  const XMLElement* htarmElem = metricsElem->getElement("htailarm");
  if (htarmElem) {
    real_type htailarm = realData(htarmElem, 0);
    PortProvider* port = addMultiBodyConstModel("HTail Arm Constant", htailarm);
    registerJSBExpression("metrics/lh-ft", port);
  }

  const XMLElement* vtareaElem = metricsElem->getElement("vtailarea");
  if (vtareaElem) {
    real_type vtailarea = realData(vtareaElem, 0);
    PortProvider* port = addMultiBodyConstModel("VTail Area Constant", vtailarea);
    registerJSBExpression("metrics/Sv-sqft", port);
  }

  const XMLElement* vtarmElem = metricsElem->getElement("vtailarm");
  if (vtarmElem) {
    real_type vtailarm = realData(vtarmElem, 0);
    PortProvider* port = addMultiBodyConstModel("VTail Arm Constant", vtailarm);
    registerJSBExpression("metrics/lv-ft", port);
  }


  std::list<const XMLElement*> locList = metricsElem->getElements("location");
  Vector3 ap = locationData(locList, "AERORP", Vector3(0, 0, 0));
  Vector3 ep = locationData(locList, "EYEPOINT", Vector3(0, 0, 0));
  Vector3 vrp = locationData(locList, "VRP", Vector3(0, 0, 0));

  // In contrast to JSBSim, we have the possibility to simulate around
  // a point not being the center of gravity, use that here ...
  Vector3 cg = mBodyReference;
  mBodyReference = vrp;
  
  // Attach the eye point.
  FreeFrame* epFrame = new FreeFrame("Eyepoint Frame");
  epFrame->setPosition(structToBody(ep));
  epFrame->setRelVel(Vector6::zeros());
  epFrame->setRelVelDot(Vector6::zeros());
  mVehicle->getTopBody()->getFrame()->addChildFrame(epFrame);
  Sensor* accelSensor = new Sensor("Acceleration Sensor");
  mVehicle->getMultiBodySystem()->addModel(accelSensor);
  mVehicle->getTopBody()->addInteract(accelSensor);
  accelSensor->addSampleTime(SampleTime(1.0/120));
  PortProvider* port = accelSensor->getOutputPort("nlfz");
  registerJSBExpression("accelerations/n-pilot-z-norm", port);
  addOutputModel(port, "Normalized load value", "accelerations/nlf");
  port = accelSensor->getOutputPort("az");
  registerJSBExpression("accelerations/accel-z-norm", port);

  // Set the position of the aerodynamic force frame.
  mAeroForce->setPosition(structToBody(ap));
  port = lookupJSBExpression("aero/alpha-deg", mAeroForce->getPath());
  addOutputModel(port, "Alpha", "orientation/alpha-deg");
  port = lookupJSBExpression("aero/beta-rad", mAeroForce->getPath());
  addOutputModel(port, "Beta rad", "orientation/side-slip-rad");
  port = lookupJSBExpression("aero/beta-deg", mAeroForce->getPath());
  addOutputModel(port, "Beta", "orientation/side-slip-deg");

  return true;
}

/// converts the METRICS data
bool
JSBSimReader::convertMassBalance(const XMLElement* massBalance)
{
  // Parse the METRICS section.
  // Since all locations within the aircraft are given in the structural frame,
  // we need to know the body frame location in the veihcle first.
  // That is parse all entries and than translate them into OpenFDM parts.

  InertiaMatrix I(0, 0, 0, 0, 0, 0);
  real_type mass = 0;
  typedef std::pair<Vector3,real_type> masspoint;
  typedef std::list<masspoint> masslist;
  masslist masses;

  real_type ixx = realData(massBalance->getElement("ixx"), 0);
  I(1, 1) = convertFrom(uSlugFt2, ixx);
  real_type iyy = realData(massBalance->getElement("iyy"), 0);
  I(2, 2) = convertFrom(uSlugFt2, iyy);
  real_type izz = realData(massBalance->getElement("izz"), 0);
  I(3, 3) = convertFrom(uSlugFt2, izz);
  real_type ixy = realData(massBalance->getElement("ixy"), 0);
  I(1, 2) = convertFrom(uSlugFt2, ixy);
  real_type ixz = realData(massBalance->getElement("ixz"), 0);
  I(1, 3) = convertFrom(uSlugFt2, ixz);
  real_type iyz = realData(massBalance->getElement("iyz"), 0);
  I(2, 3) = convertFrom(uSlugFt2, iyz);
  
  mass = realData(massBalance->getElement("emptywt"), 0);
  mass = convertFrom(uPoundSealevel, mass);

  std::list<const XMLElement*> locList = massBalance->getElements("location");
  Vector3 cg = locationData(locList, "CG", Vector3(0, 0, 0));

  std::list<const XMLElement*> pmList = massBalance->getElements("pointmass");
  std::list<const XMLElement*>::const_iterator pmit;
  pmit = pmList.begin();
  while (pmit != pmList.end()) {
    std::list<const XMLElement*> locList = (*pmit)->getElements("location");
    Vector3 loc = locationData(locList, "POINTMASS", Vector3(0, 0, 0));
    real_type weight = realData((*pmit)->getElement("weight"), 0);
    weight = convertFrom(uPoundSealevel, weight);
    masses.push_back(masspoint(loc, weight));
    ++pmit;
  }

  // Now collect all static inertia values starting with the emptyweight
  // and empty inertia together in spi.
  SpatialInertia spi(I, mass);
  spi = inertiaFrom(structToBody(cg), spi);
  masslist::iterator it = masses.begin();
  while (it != masses.end()) {
    SpatialInertia inertia(convertFrom(uPoundSealevel, it->second));
    spi += inertiaFrom(structToBody(it->first), inertia);
    ++it;
  }
  Mass* massModel = new Mass("Emptyweight Mass", spi);
  mVehicle->getMultiBodySystem()->addModel(massModel);
  mVehicle->getTopBody()->addInteract(massModel);

  return true;
}

bool
JSBSimReader::attachWheel(const XMLElement* wheelElem, const std::string& name,
                          const std::string& numStr, RigidBody* parent,
                          const Vector3& parentDesignPos)
{
  RigidBody* wheel = new RigidBody(name + " Wheel");
  mVehicle->getMultiBodySystem()->addModel(wheel);
  InertiaMatrix wheelInertia = inertiaData(wheelElem->getElement("inertia"),
                                           InertiaMatrix(10, 0, 0, 30, 0, 10));
  real_type wheelMass = realData(wheelElem->getElement("mass"), 30);
  Mass* mass = new Mass(name + " Wheel Inertia",
                        SpatialInertia(wheelInertia, wheelMass));
  mVehicle->getMultiBodySystem()->addModel(mass);
  wheel->addInteract(mass);
  
  RevoluteJoint* wj = new RevoluteJoint(name + " Wheel Joint");
  mVehicle->getMultiBodySystem()->addModel(wj);
  parent->addInteract(wj);
  wheel->setInboardJoint(wj);
  wj->setJointAxis(Vector3(0, 1, 0));
  Vector3 pos = structToBody(locationData(wheelElem->getElement("location")));
  wj->setPosition(pos - parentDesignPos);
  wj->setOrientation(Quaternion::unit());
  wj->setJointPos(0);
  wj->setJointVel(0);

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
      if (!connectJSBExpression("gear/left-brake-pos-norm",
                                brakeF->getInputPort(0)))
        return error("could not connect to brake group LEFT");
    } else if (brake == "RIGHT") {
      if (!connectJSBExpression("gear/right-brake-pos-norm",
                                brakeF->getInputPort(0)))
        return error("could not connect to brake group RIGHT");
    }
    // That one reads the joint position and velocity ...
    Connection::connect(wj->getOutputPort(1), brakeF->getInputPort(1));
    // ... and provides an output force
    Connection::connect(brakeF->getOutputPort(0), wj->getInputPort(0));
  } else {
    // Just some 'bearing friction'
    Gain* rollingFric = new Gain(name + " Bearing Friction Force");
    addMultiBodyModel(rollingFric);
    rollingFric->setGain(-10);
    Connection::connect(wj->getOutputPort(1), rollingFric->getInputPort(0));
    // ... and provides an output force
    Connection::connect(rollingFric->getOutputPort(0), wj->getInputPort(0));
  }
  
  real_type wheelDiam = realData(wheelElem->getElement("wheelDiameter"));
  WheelContact* wc = new WheelContact(name + " Wheel Contact");
  mVehicle->getMultiBodySystem()->addModel(wc);
  wheel->addInteract(wc);
  wc->setWheelRadius(0.5*wheelDiam);
  real_type tireSpring = realData(wheelElem->getElement("tireSpring"));
  wc->setSpringConstant(convertFrom(uPoundForcePFt, tireSpring));
  real_type tireDamp = realData(wheelElem->getElement("tireDamping"));
  wc->setSpringDamping(convertFrom(uPoundForcePFt, tireDamp));
  real_type fc = realData(wheelElem->getElement("frictionCoef"), 0.9);
  wc->setFrictionCoeficient(fc);
  
  PortProvider* port = wj->getOutputPort(0);
  std::string nameBase = "Wheel " + numStr + " Position";
  addOutputModel(port, nameBase,
                 "gear/gear[" + numStr + "]/wheel-position-rad");
  port = addToUnit(nameBase + " converter", uDegree, port);
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
        mVehicle->getMultiBodySystem()->addModel(sg);
        mVehicle->getTopBody()->addInteract(sg);
        Vector3 loc
          = locationData((*it)->getElement("location"), Vector3(0, 0, 0));
        sg->setPosition(structToBody(loc));
        
        real_type k = realData((*it)->getElement("spring_coeff"), 0);
        sg->setSpringConstant(convertFrom(uPoundForcePFt, k));
        // FIXME: conversion factor:
        // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
        // Note that friction coefficients are different from that but
        // viscosous friction is just used in that way ...
        real_type d = realData((*it)->getElement("damping_coeff"), 0);
        sg->setSpringDamping(convertFrom(uPoundForcePFt, d));
        real_type fs = realData((*it)->getElement("static_friction"), 0);
        sg->setFrictionCoeficient(fs);
        
        // Connect apprioriate input and output models

        // FIXME
        // missing output properties are "wow" and "tire-pressure-norm"

        std::string retract = stringData((*it)->getElement("retractable"));
        if (retract == "RETRACT" || retract == "1") {
          if (!connectJSBExpression("gear/gear-pos-norm", sg->getEnablePort()))
            return error("could not connect to retract command");
          // Well, connect that directly to the input
          PortProvider* port = lookupJSBExpression("gear/gear-pos-norm");
          addOutputModel(port, "Gear " + numStr + " Position",
                         "gear/gear[" + numStr + "]/position-norm");
        }

        real_type maxSteer = realData((*it)->getElement("max_steer"), 0);
        if (maxSteer != 0) {
          UnaryFunctionModel* scale
            = new UnaryFunctionModel(name + " Scale", UnaryFunctionModel::Abs);
          addFCSModel(scale);
          // FIXME: FCS might later define something for that gain ...
          // "fcs/steer-pos-deg[" + numStr + "]";
          if (!connectJSBExpression("fcs/steer-cmd-norm",
                                    scale->getInputPort(0)))
            return error("could not connect to steering command");

          Product* sProd = new Product(name + " SProd");
          addFCSModel(sProd);
          sProd->setNumFactors(2);
          if (!connectJSBExpression("fcs/steer-cmd-norm",
                                    sProd->getInputPort(0)))
            return error("could not connect to steering command");
          Connection::connect(scale->getOutputPort(0), sProd->getInputPort(1));
          PortProvider* port = sProd->getOutputPort(0);

          Gain* gain = new Gain(name + " Steer Gain");
          addFCSModel(gain);
          gain->setGain(maxSteer);
          Connection::connect(port, gain->getInputPort(0));
          addOutputModel(port, "Gear " + numStr + " Steering Output",
                         "gear/gear[" + numStr + "]/steering-norm");

          UnitConversionModel* unitConv
            = new UnitConversionModel(name + " Degree Conversion",
                                      UnitConversionModel::UnitToSi,
                                      uDegree);
          addFCSModel(unitConv);
          Connection::connect(gain->getOutputPort(0),
                              unitConv->getInputPort(0));
          Connection::connect(unitConv->getOutputPort(0),
                              sg->getInputPort("steeringAngle"));
        }
        
        std::string brake = stringData((*it)->getElement("brake_group"));
        if (brake == "LEFT") {
          if (!connectJSBExpression("gear/left-brake-pos-norm",
                                    sg->getInputPort("brakeCommand")))
            return error("could not connect brake command for group LEFT");
        } else if (brake == "RIGHT") {
          if (!connectJSBExpression("gear/right-brake-pos-norm",
                                    sg->getInputPort("brakeCommand")))
            return error("could not connect brake command for group RIGHT");
        }
        
      } else if (type == "STRUCTURE" || type == "CONTACT") {
        // Very simple contact force. Penalty method.
        SimpleContact* sc = new SimpleContact((*it)->getAttribute("name"));
        mVehicle->getMultiBodySystem()->addModel(sc);
        mVehicle->getTopBody()->addInteract(sc);

        Vector3 loc
          = locationData((*it)->getElement("location"), Vector3(0, 0, 0));
        sc->setPosition(structToBody(loc));

        real_type k = realData((*it)->getElement("spring_coeff"), 0);
        sc->setSpringConstant(convertFrom(uPoundForcePFt, k));
        // FIXME: conversion factor:
        // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
        // Note that friction coefficients are different from that but
        // viscosous friction is just used in that way ...
        real_type d = realData((*it)->getElement("damping_coeff"), 0);
        sc->setSpringDamping(convertFrom(uPoundForcePFt, d));
        real_type fs = realData((*it)->getElement("static_friction"), 0);
        sc->setFrictionCoeficient(fs);
        
      } else if (type == "NOSEGEAR") {
        // Ok, a compressable gear like the F-18's main gear is.
        // Some kind of hardcoding here ...
        std::stringstream sstr;
        sstr << gearNumber++;
        std::string numStr = sstr.str();
        std::string name = (*it)->getAttribute("name");

        Vector3 parentPos = structToBody(Vector3::zeros());
        Vector3 compressJointPos = locationData((*it)->getElement("location"));
        // Model steering here ...
        // normally we connect the compressible part to the top level body, but
        // in case of steering this is no longer true.
        RigidBody* strutParent = mVehicle->getTopBody();
        std::string steerable = stringData((*it)->getElement("steerable"));
        if (steerable == "true" || steerable == "1") {
          // A new part modelling the steering
          RigidBody* steer = new RigidBody(name + " Steer");
          mVehicle->getMultiBodySystem()->addModel(steer);
          
          // connect that via a revolute joint to the toplevel body.
          // Note the 0.05m below, most steering wheels have some kind of
          // castering auto line up behavour. That is doe with this 0.05m.
          RevoluteActuator* sj = new RevoluteActuator(name + " Steer Joint");
          mVehicle->getMultiBodySystem()->addModel(sj);
          strutParent->addInteract(sj);
          steer->setInboardJoint(sj);
          sj->setJointAxis(Vector3(0, 0, 1));
          sj->setJointPos(0);
          sj->setJointVel(0);
          parentPos = structToBody(compressJointPos) + Vector3(0.05, 0, 0);
          sj->setPosition(parentPos);
          sj->setOrientation(Quaternion::unit());
          
          UnaryFunctionModel* scale
            = new UnaryFunctionModel(name + " Scale", UnaryFunctionModel::Abs);
          addFCSModel(scale);
          if (!connectJSBExpression("fcs/steer-cmd-norm",
                                    scale->getInputPort(0)))
            return error("could not connect to steering command");

          Product* sProd = new Product(name + " SProd");
          addFCSModel(sProd);
          sProd->setNumFactors(2);
          if (!connectJSBExpression("fcs/steer-cmd-norm",
                                    sProd->getInputPort(0)))
            return error("could not connect to steering command");
          Connection::connect(scale->getOutputPort(0), sProd->getInputPort(1));
          Connection::connectRoute(sProd->getOutputPort(0), sj->getInputPort(0));
  
          strutParent = steer;
          
          // Prepare outputs
          PortProvider* port = sj->getOutputPort(0);
          std::string nameBase = "Steering " + numStr + " Position";
          addOutputModel(port, nameBase,
                         "gear/gear[" + numStr + "]/steering-pos-rad");
          port = addToUnit(nameBase + " converter", uDegree, port);
          addOutputModel(port, nameBase + " Deg",
                         "gear/gear[" + numStr + "]/steering-pos-deg");
        }
        
        const XMLElement* launchbarElem = (*it)->getElement("launchbar");
        if (launchbarElem) {
          Launchbar* launchbar = new Launchbar(name + " Launchbar");
          addMultiBodyModel(launchbar);
          strutParent->addInteract(launchbar);
          real_type length = realData(launchbarElem->getElement("length"), 0.5);
          launchbar->setLength(length);
          real_type upAngle = realData(launchbarElem->getElement("upAngle"), 30);
          launchbar->setUpAngle(convertFrom(uDegree, upAngle));
          real_type downAngle = realData(launchbarElem->getElement("downAngle"), -50);
          launchbar->setDownAngle(convertFrom(uDegree, downAngle));
          real_type force = realData(launchbarElem->getElement("launchForce"), 1000);
          launchbar->setLaunchForce(force);
          Vector3 loc = structToBody(locationData(launchbarElem->getElement("location")));
          launchbar->setPosition(loc - parentPos);


          if (!connectJSBExpression("/controls/gear/launchbar",
                                    launchbar->getInputPort("tryMount")))
            return error("could not connect to launchbar command");
          if (!connectJSBExpression("/controls/gear/catapult-launch-cmd",
                                    launchbar->getInputPort("launchCommand")))
            return error("could not connect to launch command");

          // expose the launchbar position
          PortProvider* port = launchbar->getOutputPort(0);
          std::string nameBase = "Launchbar Position";
          addOutputModel(port, nameBase, "gear/launchbar-pos-rad");
          port = addToUnit(nameBase + " converter", uDegree, port);
          addOutputModel(port, nameBase + " Deg", "gear/launchbar-pos-deg");
        }

        
        // Now the compressible part of the strut
        RigidBody* arm = new RigidBody(name + " Strut");
        mVehicle->getMultiBodySystem()->addModel(arm);
        Mass* mass = new Mass(name + " Strut Mass", inertiaFrom(Vector3(0, 0, 1), SpatialInertia(100)));
        mVehicle->getMultiBodySystem()->addModel(mass);
        arm->addInteract(mass);
        
        // This time it is a prismatic joint
        PrismaticJoint* pj = new PrismaticJoint(name + " Compress Joint");
        mVehicle->getMultiBodySystem()->addModel(pj);
        strutParent->addInteract(pj);
        arm->setInboardJoint(pj);
        pj->setJointAxis(Vector3(0, 0, -1));
        pj->setPosition(structToBody(compressJointPos) - parentPos);
        parentPos = structToBody(compressJointPos);
        
        // The damper element
        const XMLElement* airSpringElem = (*it)->getElement("damper");
        AirSpring* aoDamp = getAirSpring(airSpringElem, name);
        addMultiBodyModel(aoDamp);
        Connection::connect(aoDamp->getOutputPort(0), pj->getInputPort(0));
        Connection::connect(pj->getOutputPort(0), aoDamp->getInputPort(0));
        Connection::connect(pj->getOutputPort(1), aoDamp->getInputPort(1));
        
        // Attach a wheel to that strut part.
        attachWheel((*it)->getElement("wheel"), name, numStr, arm, parentPos);
        
        // Prepare some outputs ...
        PortProvider* port = pj->getOutputPort(0);
        addOutputModel(port, "Gear " + numStr + " Compression",
                       "gear/gear[" + numStr + "]/compression-m");
        
        port = lookupJSBExpression("gear/gear-pos-norm");
        addOutputModel(port, "Gear " + numStr + " Position",
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
        mVehicle->getMultiBodySystem()->addModel(arm);
        Mass* mass = new Mass(name + " Strut Mass", inertiaFrom(Vector3(-1, 0, 0), SpatialInertia(80)));
        mVehicle->getMultiBodySystem()->addModel(mass);
        arm->addInteract(mass);
        
        // Connect that with a revolute joint to the main body
        RevoluteJoint* rj = new RevoluteJoint(name + " Arm Joint");
        mVehicle->getMultiBodySystem()->addModel(rj);
        mVehicle->getTopBody()->addInteract(rj);
        arm->setInboardJoint(rj);
        Vector3 compressJointAxis = locationData((*it)->getElement("axis"),
                                                 Vector3(0, 1, 0));
        rj->setJointAxis(normalize(compressJointAxis));
        rj->setJointPos(0);
        rj->setJointVel(0);
        Vector3 compressJointPos = locationData((*it)->getElement("location"));
        rj->setPosition(structToBody(compressJointPos));
        rj->setOrientation(Quaternion::unit());
        
        LineForce* lineForce = new LineForce(name + " Air Spring LineForce");
        mVehicle->getMultiBodySystem()->addModel(lineForce);
        mVehicle->getTopBody()->addInteract(lineForce);
        arm->addInteract(lineForce);
        /// FIXME that ordering in attachment is messy!
        Vector3 asMnt0 = locationData((*it)->getElement("springMount0"),
                                      compressJointPos -
                                      convertTo(uInch, Vector3(0.1, 0, 0.5)));
        Vector3 asMnt1 = locationData((*it)->getElement("springMount1"),
                                      compressJointPos +
                                      convertTo(uInch, Vector3(-0.5, 0, 0)));
        lineForce->setPosition0(structToBody(asMnt0));
        lineForce->setPosition1(structToBody(asMnt1)
                                - structToBody(compressJointPos));
        
        // The damper element
        const XMLElement* airSpringElem = (*it)->getElement("damper");
        AirSpring* aoDamp = getAirSpring(airSpringElem, name);
        addMultiBodyModel(aoDamp);
        // That one reads the joint position and velocity ...
        Connection::connect(lineForce->getOutputPort(0),
                            aoDamp->getInputPort(0));
        Connection::connect(lineForce->getOutputPort(1),
                            aoDamp->getInputPort(1));
        // ... and provides an output force
        Connection::connect(aoDamp->getOutputPort(0),
                            lineForce->getInputPort(0));
        
        // Attach a wheel to that strut part.
        attachWheel((*it)->getElement("wheel"), name, numStr, arm,
                    structToBody(compressJointPos));
        
        PortProvider* port = rj->getOutputPort(0);
        addOutputModel(port, "Gear " + numStr + " Compression",
                       "gear/gear[" + numStr + "]/compression-rad");
        
        /// FIXME add a retract joint ...
        port = lookupJSBExpression("gear/gear-pos-norm");
        addOutputModel(port, "Gear " + numStr + " Position",
                       "gear/gear[" + numStr + "]/position-norm");
        
      } else if (type == "TAILHOOK") {
        const XMLElement* tailhookElem = (*it);
        std::string name = (*it)->getAttribute("name");

        Tailhook* tailhook = new Tailhook(name + " Tailhook");
        mVehicle->getMultiBodySystem()->addModel(tailhook);
        real_type length = realData(tailhookElem->getElement("length"), 0.5);
        tailhook->setLength(length);
        real_type upAngle = realData(tailhookElem->getElement("upAngle"), 10);
        tailhook->setUpAngle(convertFrom(uDegree, upAngle));
        real_type downAngle = realData(tailhookElem->getElement("downAngle"), -75);
        tailhook->setDownAngle(convertFrom(uDegree, downAngle));
        Vector3 loc = structToBody(locationData(tailhookElem->getElement("location")));
        tailhook->setPosition(loc);
        addMultiBodyModel(tailhook);
        mVehicle->getTopBody()->addInteract(tailhook);
        
        if (!connectJSBExpression("/controls/gear/tailhook",
                                  tailhook->getInputPort(0)))
          return error("could not connect to tailhook command");
        
        // expose the tailhook position
        PortProvider* port = tailhook->getOutputPort(0);
        std::string nameBase = "Tailhook Position";
        addOutputModel(port, nameBase, "gear/tailhook/position-rad");
        port = addToUnit(nameBase + " converter", uDegree, port);
        addOutputModel(port, nameBase + " Deg", "gear/tailhook/position-deg");

      } else {
        return error("Unknown groundreactions component of type " + type);
      }
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
    = Quaternion::fromYawPitchRoll(orient(3), orient(2), orient(1));


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
    
//   } else if (engineTopElem->getName() == "piston_engine") {
//     if (!convertPiston(engineTopElem, number, 0))
//       return error("Error readinge piston configuration");

//   } else if (engineTopElem->getName() == "rocket_engine") {
//     return error("FG_ROCKET's are not (yet?) supported!");

//   } else if (engineTopElem->getName() == "electric_engine") {
//     if (!convertElectric(engineTopElem, number, 0))
//       return error("Error readinge electric configuration");

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
// //   wj->setJointAxis(Vector3(0, 1, 0));
// //   wj->setPosition(pos);
// //   wj->setOrientation(Quaternion::unit());
// //   wj->setJointPos(0);
// //   wj->setJointVel(0);

// //     DiscBrake* brakeF = new DiscBrake(name + " Brake Force");
// //     brakeF->setMinForce(8e1);
// //     brakeF->setMaxForce(1e4);
// //     if (brake == "LEFT") {
// //       Port* port = lookupJSBExpression("gear/left-brake-pos-norm");
// //       brakeF->getInputPort(0)->connect(port);
// //     } else if (brake == "RIGHT") {
// //       Port* port = lookupJSBExpression("gear/right-brake-pos-norm");
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
                             PortProvider* thrusterDriver)
{
  // At the moment we have a *very* insufficient engine, just modulate the
  // thrust between 0 and max
  real_type maxThrust = realData(turbine->getElement("milthrust"), 0);

  std::string namestr = "Engine<" + number + ">";
  ConstModel* fullForce = new ConstModel(namestr + " full");
  addMultiBodyModel(fullForce);
  fullForce->setValue(Vector6(0, 0, 0, convertFrom(uPoundForce, maxThrust), 0, 0));

  Product* prod = new Product(namestr + " modulation");
  addMultiBodyModel(prod);
  std::string throttlename = "fcs/throttle-cmd-norm[" + number + "]";
  if (!connectJSBExpression(throttlename, prod->getInputPort(0)))
    return error("could not connect to throttle command");
  Connection::connect(fullForce->getOutputPort(0), prod->getInputPort(1));

  ExternalForceModel* engineForce = new ExternalForceModel(namestr);
  mVehicle->getMultiBodySystem()->addModel(engineForce);
  mVehicle->getTopBody()->addInteract(engineForce);
  engineForce->setPosition(pos);
  engineForce->setOrientation(orientation);
  Connection::connect(prod->getOutputPort(0), engineForce->getInputPort(0));

  return true;
}

// bool
// JSBSimReader::convertElectric(const XMLElement* turbine,
//                               const std::string& number,
//                               Port* thrusterDriver)
// {
//   return true;
// }

// bool
// JSBSimReader::convertPiston(const XMLElement* turbine,
//                             const std::string& number,
//                             Port* thrusterDriver)
// {
//   return true;
// }

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
  SharedPtr<Model> model;

  // The final output property.
  SharedPtr<PortProvider> out;

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
      Connection::connect(summer->getOutputPort(0), bias->getInputPort(0));
      Matrix m(1, 1);
      m(1, 1) = b;
      bias->setBias(m);
      model = bias;
    } else {
      model = summer;
    }
    out = model->getOutputPort(0);
  } else if (type == "DEADBAND" || type == "deadband") {
    SharedPtr<DeadBand> deadband = new DeadBand(name);
    addFCSModel(deadband);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, deadband->getInputPort(0)))
      return error("could not connect to deadband input \"" + token + "\"");
    deadband->setWidth(realData(fcsComponent->getElement("width"), 0));
    model = deadband;
    out = model->getOutputPort(0);

  } else if (type == "GRADIENT" || type == "gradient") {
    SharedPtr<TimeDerivative> timeDeriv = new TimeDerivative(name);
    addFCSModel(timeDeriv);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, timeDeriv->getInputPort(0)))
      return error("could not connect to gradient input \"" + token + "\"");
    model = timeDeriv;
    out = model->getOutputPort(0);

  } else if (type == "SWITCH" || type == "switch") {
    std::cout << "Ignoring SWITCH" << std::endl;

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
    model = kinemat->getModelGroup();
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getInputPort(0)))
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
    out = model->getOutputPort(0);

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
    model = asScale->getModelGroup();
    addFCSModel(model);
    token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getInputPort(0)))
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
    TableLookup lookup;
    if (!readTable1D(tbl, data, lookup))
      return error("Cannot read table");
    sGain->setTableData(data, lookup);

    model = sGain->getModelGroup();
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getInputPort(0)))
      return error("could not connect to gain input \"" + token + "\"");
    token = stringData(tbl->getElement("independentVar"));
    if (!connectJSBExpression(token, model->getInputPort(1)))
      return error("could not connect to table input \"" + token + "\"");

    out = sGain->getOutputPort();

  } else if (type == "INTEGRATOR" || type == "integrator") {
    model = new DiscreteIntegrator(name);
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getInputPort(0)))
      return error("could not connect to integrator input \"" + token + "\"");
    out = model->getOutputPort(0);
    real_type c1 = realData(fcsComponent->getElement("c1"), 1);
    if (c1 != 1) {
      SharedPtr<Gain> gain = new Gain(name + " Gain");
      addFCSModel(gain);
      Connection::connect(model->getOutputPort(0), gain->getInputPort(0));
      gain->setGain(c1);
      model = gain;
      out = model->getOutputPort(0);
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
    if (!connectJSBExpression(token, model->getInputPort(0)))
      return error("could not connect to transfer function input \""
                   + token + "\"");
    Vector v(1);
    v(1) = realData(fcsComponent->getElement("c1"), 1);
    discreteTransfFunc->setNumerator(v);
    v.resize(2);
    v(1) = 1;
    v(2) = realData(fcsComponent->getElement("c1"), 1);
    discreteTransfFunc->setDenominator(v);
    out = model->getOutputPort(0);

  } else if (type == "LEAD_LAG_FILTER" || type == "lead_lag_filter") {
    // C1*s + C2
    // ---------
    // C3*s + C4
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    model = discreteTransfFunc;
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getInputPort(0)))
      return error("could not connect to transfer function input \""
                   + token + "\"");
    Vector v(2);
    v(1) = realData(fcsComponent->getElement("c1"), 1);
    v(2) = realData(fcsComponent->getElement("c2"), 1);
    discreteTransfFunc->setNumerator(v);
    v(1) = realData(fcsComponent->getElement("c3"), 1);
    v(2) = realData(fcsComponent->getElement("c4"), 1);
    discreteTransfFunc->setDenominator(v);

    out = model->getOutputPort(0);

  } else if (type == "WASHOUT_FILTER" || type == "washout_filter") {
    //   s
    // ------
    // s + C1
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    model = discreteTransfFunc;
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getInputPort(0)))
      return error("could not connect to transfer function input \""
                   + token + "\"");
    Vector v(1);
    v(1) = 1;
    discreteTransfFunc->setNumerator(v);
    v.resize(2);
    v(1) = 1;
    v(2) = realData(fcsComponent->getElement("c1"), 1);
    discreteTransfFunc->setDenominator(v);
    out = model->getOutputPort(0);

  } else if (type == "SECOND_ORDER_FILTER" || type == "second_order_filter") {
    // C1*s + C2*s + C3
    // ----------------
    // C4*s + C5*s + C6
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    model = discreteTransfFunc;
    addFCSModel(model);
    std::string token = stringData(fcsComponent->getElement("input"));
    if (!connectJSBExpression(token, model->getInputPort(0)))
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

    out = model->getOutputPort(0);

  } else if (type == "SENSOR" || type == "sensor") {
    std::cout << "Ignoring SENSOR" << std::endl;

  } else
    return error("Unknown FCS COMPONENT type: \"" + type
                 + "\". Ignoring whole FCS component \"" + name + "\"" );

  OpenFDMAssert(out->isConnected());

  // Register all output property names.
  std::string implicitOutname = normalizeComponentName(name);
  registerJSBExpression(std::string("fcs/") + implicitOutname, out);
  if (fcsComponent->getElement("output")) {
    std::string outname = stringData(fcsComponent->getElement("output"));
    if (outname != implicitOutname)
      registerJSBExpression(outname, out);
  }

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
        continue;
      PortProvider* port = sum->getOutputPort(0);

      if (axisname == "LIFT") {
        port = addFromUnit("LIFT unit convert", uPoundForce, port);
        Connection::connect(port, mAeroForce->getInputPort("lift"));
      } else if (axisname == "DRAG") {
        port = addFromUnit("DRAG unit convert", uPoundForce, port);
        Connection::connect(port, mAeroForce->getInputPort("drag"));
      } else if (axisname == "SIDE") {
        port = addFromUnit("SIDE unit convert", uPoundForce, port);
        Connection::connect(port, mAeroForce->getInputPort("side"));
      } else if (axisname == "ROLL") {
        port = addFromUnit("ROLL unit convert", uPoundForceFt, port);
        Connection::connect(port, mAeroForce->getInputPort("roll"));
      } else if (axisname == "PITCH") {
        port = addFromUnit("PITCH unit convert", uPoundForceFt, port);
        Connection::connect(port, mAeroForce->getInputPort("pitch"));
      } else if (axisname == "YAW") {
        port = addFromUnit("YAW unit convert", uPoundForceFt, port);
        Connection::connect(port, mAeroForce->getInputPort("yaw"));
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
  PortProvider* port = 0;
  std::list<const XMLElement*> elems = function->getElements();
  std::list<const XMLElement*>::const_iterator it;
  for (it = elems.begin(); it != elems.end(); ++it) {
    if ((*it)->getName() == "description") {
      // Just ignore
    } else if ((*it)->getName() == "product") {
      SharedPtr<Product> prod = new Product(name + " product");
      addMultiBodyModel(prod);
      std::list<PortProvider*> inputs = readFunctionInputs(*it, name);
      if (inputs.empty())
        return error("Cannot read product inputs!");
      unsigned i = 0;
      std::list<PortProvider*>::iterator iit = inputs.begin();
      while (iit != inputs.end()) {
        prod->setNumFactors(i+1);
        Connection::connect(*iit++, prod->getInputPort(i++));
      }
      port = prod->getOutputPort(0);
    } else {
      return error("Unknown tag in function");
    }
  }

  if (!port)
    return error("function without output!");

  registerJSBExpression(bindName, port);
  if (sum) {
    unsigned num = sum->getNumSummands();
    sum->setNumSummands(num+1);
    Connection::connect(port, sum->getInputPort(num));
  }

  return true;
}

std::list<PortProvider*>
JSBSimReader::readFunctionInputs(const XMLElement* operationTag,
                                 const std::string& name)
{
  Model::Path path = mVehicle->getMultiBodySystem()->getGroupPath();
  std::list<PortProvider*> inputs;
  std::list<const XMLElement*> args = operationTag->getElements();
  std::list<const XMLElement*>::const_iterator ait;
  for (ait = args.begin(); ait != args.end(); ++ait) {
    if ((*ait)->getName() == "value") {
      std::stringstream stream((*ait)->getData());
      real_type value;
      stream >> value;
      inputs.push_back(addMultiBodyConstModel(name + " Constant", value));
    } else if ((*ait)->getName() == "property") {
      inputs.push_back(lookupJSBExpression(stringData(*ait), path));
    } else if ((*ait)->getName() == "table") {
      unsigned dim = getNumTableDims(*ait);
      if (dim == 1) {
        TableData<1> data;
        TableLookup lookup;
        if (!readTable1D(*ait, data, lookup)) {
          error("Cannot read 1D table data.");
          return std::list<PortProvider*>();
        }
        std::string token = stringData((*ait)->getElement("independentVar"));
        PortProvider* port = getTablePrelookup(name + " lookup",
                                lookupJSBExpression(token, path), lookup);

        SharedPtr<Table1D> table = new Table1D(name + " Table");
        addMultiBodyModel(table);
        Connection::connect(port, table->getInputPort(0));
        table->setTableData(data);
        inputs.push_back(table->getOutputPort(0));

      } else if (dim == 2) {
        TableData<2> data;
        TableLookup lookup[2];
        if (!readTable2D(*ait, data, lookup)) {
          error("Cannot read 2D table data.");
          return std::list<PortProvider*>();
        }

        std::list<const XMLElement*> indeps
          = (*ait)->getElements("independentVar");
        if (indeps.size() != 2) {
          error("2DTable data does not have 2 inputs!");
          return std::list<PortProvider*>();
        }
        std::string rowInput = indepData(indeps, "row");
        std::string colInput = indepData(indeps, "column");

        PortProvider* rPort = getTablePrelookup(name + " row lookup",
                            lookupJSBExpression(rowInput, path), lookup[0]);
        PortProvider* cPort = getTablePrelookup(name + " column lookup",
                            lookupJSBExpression(colInput, path), lookup[1]);

        SharedPtr<Table2D> table = new Table2D(name  + " Table");
        addMultiBodyModel(table);
        Connection::connect(rPort, table->getInputPort(0));
        Connection::connect(cPort, table->getInputPort(1));
        table->setTableData(data);
        inputs.push_back(table->getOutputPort(0));

      } else if (dim == 3) {
        TableData<3> data;
        TableLookup lookup[3];
        if (!readTable3D(*ait, data, lookup)) {
          error("Cannot read 1D table data.");
          return std::list<PortProvider*>();
        }

        std::list<const XMLElement*> indeps
          = (*ait)->getElements("independentVar");
        if (indeps.size() != 3) {
          error("3DTable data does not have 3 inputs!");
          return std::list<PortProvider*>();
        }
        std::string rowInput = indepData(indeps, "row");
        std::string colInput = indepData(indeps, "column");
        std::string pageInput = indepData(indeps, "table");

        PortProvider* rPort = getTablePrelookup(name + " row lookup",
                           lookupJSBExpression(rowInput, path), lookup[0]);
        PortProvider* cPort = getTablePrelookup(name + " column lookup",
                           lookupJSBExpression(colInput, path), lookup[1]);
        PortProvider* pPort = getTablePrelookup(name + " page lookup",
                           lookupJSBExpression(pageInput, path), lookup[2]);

        SharedPtr<Table3D> table = new Table3D(name  + " Table");
        addMultiBodyModel(table);
        Connection::connect(rPort, table->getInputPort(0));
        Connection::connect(cPort, table->getInputPort(1));
        Connection::connect(pPort, table->getInputPort(2));
        table->setTableData(data);
        inputs.push_back(table->getOutputPort(0));

      } else {
        error("Unknown tabel dimension.");
        return std::list<PortProvider*>();
      }
    } else {
      error("Unknown function input");
      return std::list<PortProvider*>();
    }
  }
  return inputs;
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
                          TableData<1>& data, TableLookup& lookup)
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
  sv(1) = sz;
  data = TableData<1>(sv);
  for (unsigned idx = 0; idx < sz; ++idx) {
    lookup.setAtIndex(idx+1, values[idx*2]);
    TableData<1>::Index iv;
    iv(1) = idx + 1;
    data(iv) = values[idx*2+1];
  }
  return true;
}

bool
JSBSimReader::readTable2D(const XMLElement* tableElem,
                          TableData<2>& data, TableLookup lookup[2])
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
      lookup[1].setAtIndex(cols+1, in);
    }
  }
  std::vector<real_type> values;
  unsigned rows = 0;
  while(stream) {
    real_type val;
    stream >> val;
    if (!stream)
      break;
    lookup[0].setAtIndex(++rows, val);

    for (unsigned i = 0; i < cols; ++i) {
      stream >> val;
      values.push_back(val);
    }
  }

  if (values.size() != rows*cols)
    return error("Invalid table size!");

  TableData<2>::SizeVector sv;
  sv(1) = rows;
  sv(2) = cols;
  data = TableData<2>(sv);
  for (unsigned i = 0; i < rows; ++i) {
    for (unsigned j = 0; j < cols; ++j) {
      TableData<2>::Index iv;
      iv(1) = i+1;
      iv(2) = j+1;
      data(iv) = values[i*cols+j];
    }
  }

  return lookup[0].isValid() && lookup[1].isValid() &&
    lookup[0].size() == data.size(1) && lookup[1].size() == data.size(2);
}

bool
JSBSimReader::readTable3D(const XMLElement* tableElem,
                          TableData<3>& data, TableLookup lookup[3])
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
