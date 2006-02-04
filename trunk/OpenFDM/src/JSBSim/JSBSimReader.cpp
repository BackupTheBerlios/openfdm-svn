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
realData(const XMLElement* element, real_type def)
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
locationData(const XMLElement* element, const Vector3& def)
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
             const Vector3& def)
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
orientationData(const XMLElement* element, const Vector3& def)
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
    Port* port = addConstModel("Wing Incidence Constant", iw);
    registerJSBExpression("metrics/iw-deg", port);
  }

  const XMLElement* htareaElem = metricsElem->getElement("htailarea");
  if (htareaElem) {
    real_type htailarea = realData(htareaElem, 0);
    Port* port = addConstModel("HTail Area Constant", htailarea);
    registerJSBExpression("metrics/Sh-sqft", port);
  }

  const XMLElement* htarmElem = metricsElem->getElement("htailarm");
  if (htarmElem) {
    real_type htailarm = realData(htarmElem, 0);
    Port* port = addConstModel("HTail Arm Constant", htailarm);
    registerJSBExpression("metrics/lh-ft", port);
  }

  const XMLElement* vtareaElem = metricsElem->getElement("vtailarea");
  if (vtareaElem) {
    real_type vtailarea = realData(vtareaElem, 0);
    Port* port = addConstModel("VTail Area Constant", vtailarea);
    registerJSBExpression("metrics/Sv-sqft", port);
  }

  const XMLElement* vtarmElem = metricsElem->getElement("vtailarm");
  if (vtarmElem) {
    real_type vtailarm = realData(vtarmElem, 0);
    Port* port = addConstModel("VTail Arm Constant", vtailarm);
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
  Sensor* accelSensor = new Sensor("Acceleration Sensor");
  accelSensor->addSampleTime(SampleTime(1.0/120));
  Port* port = accelSensor->getOutputPort("nz");
  registerJSBExpression("accelerations/n-pilot-z-norm", port);
//   epFrame->addInteract(accelSensor);
  mVehicle->getTopBody()->addInteract(accelSensor);
  mVehicle->getTopBody()->getFrame()->addChildFrame(epFrame);
  addOutputModel(port, "Normalized load value", "accelerations/nlf");

  // Set the position of the aerodynamic force frame.
  mAeroForce->setPosition(structToBody(ap));

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
  mVehicle->getTopBody()->addInteract(new Mass("Emptyweight Mass", spi));

  return true;
}

// void
// JSBSimReader::attachWheel(const std::string& name, const Vector3& pos,
//                                 const std::string& brake,
//                                 const std::string& numStr, real_type wheelDiam,
//                                 real_type tireSpring, real_type tireDamp,
//                                 RigidBody* parent)
// {
//   RigidBody* wheel = new RigidBody(name + " Wheel");
//   InertiaMatrix wheelInertia(10, 0, 0, 30, 0, 10);
//   wheel->addInteract(new Mass(name + " Wheel Inertia",
//                               SpatialInertia(wheelInertia, 30)));
//   mVehicle->getMultiBodySystem()->addRigidBody(wheel);
  
//   RevoluteJoint* wj = new RevoluteJoint(name + " Wheel Joint");
//   parent->addInteract(wj);
//   wheel->setInboardJoint(wj);
//   wj->setJointAxis(Vector3(0, 1, 0));
//   wj->setPosition(pos);
//   wj->setOrientation(Quaternion::unit());
//   wj->setJointPos(0);
//   wj->setJointVel(0);

//   // Add a brake force
//   if (brake == "LEFT" || brake == "RIGHT") {
//     DiscBrake* brakeF = new DiscBrake(name + " Brake Force");
//     brakeF->setMinForce(8e1);
//     brakeF->setMaxForce(1e4);
//     if (brake == "LEFT") {
//       Port* port = lookupJSBExpression("gear/left-brake-pos-norm");
//       brakeF->getInputPort(0)->connect(port);
//     } else if (brake == "RIGHT") {
//       Port* port = lookupJSBExpression("gear/right-brake-pos-norm");
//       brakeF->getInputPort(0)->connect(port);
//     }
//     // That one reads the joint position and velocity ...
//     brakeF->getInputPort(1)->connect(wj->getOutputPort(1));
//     // ... and provides an output force
//     wj->getInputPort(0)->connect(brakeF->getOutputPort(0));
//     addMultiBodyModel(brakeF);
//   } else {
//     // Just some 'rolloing friction' FIXME: does this belong here?
//     Gain* rollingFric = new Gain(name + " Rolling Friction Force");
//     rollingFric->setGain(-10);
//     rollingFric->getInputPort(0)->connect(wj->getOutputPort(1));
//     // ... and provides an output force
//     wj->getInputPort(0)->connect(rollingFric->getOutputPort(0));
//     addMultiBodyModel(rollingFric);
//   }
  
//   WheelContact* wc = new WheelContact(name + " Wheel Contact");
//   wc->setWheelRadius(0.5*wheelDiam);
//   wc->setSpringConstant(convertFrom(uPoundForcePFt, tireSpring));
//   wc->setSpringDamping(convertFrom(uPoundForcePFt, tireDamp));
//   wc->setFrictionCoeficient(0.9);
//   wheel->addInteract(wc);
  
//   Port* port = wj->getOutputPort(0);
//   std::string nameBase = "Wheel " + numStr + " Position";
//   addOutputModel(port, nameBase,
//                  "gear/gear[" + numStr + "]/wheel-position-rad");
//   UnitConversionModel* unitModel
//     = new UnitConversionModel(nameBase + " converter",
//                               UnitConversionModel::SiToUnit, uDegree);
//   unitModel->getInputPort(0)->connect(port);
//   addFCSModel(unitModel);
//   addOutputModel(unitModel->getOutputPort(0), nameBase + " Deg",
//                  "gear/gear[" + numStr + "]/wheel-position-deg");
// }

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
          Port* port = lookupJSBExpression("gear/gear-pos-norm");
          sg->getInputPort("enabled")->connect(port);
          // Well, connect that directly to the input
          addOutputModel(port, "Gear " + numStr + " Position",
                         "gear/gear[" + numStr + "]/position-norm");
        }

        real_type maxSteer = realData((*it)->getElement("max_steer"), 0);
        if (maxSteer != 0) {
          // FIXME: FCS might later define something for that gain ...
//           prop = lookupJSBExpression("fcs/steer-pos-deg[" + numStr + "]");
          Port* port = lookupJSBExpression("fcs/steer-cmd-norm");
          Gain* gain = new Gain(name + " Steer Gain");
          gain->setGain(maxSteer);
          gain->getInputPort(0)->connect(port);
          addFCSModel(gain);
          addOutputModel(port, "Gear " + numStr + " Steering Output",
                         "gear/gear[" + numStr + "]/steering-norm");

          UnitConversionModel* unitConv
            = new UnitConversionModel(name + " Degree Conversion",
                                      UnitConversionModel::UnitToSi,
                                      uDegree);
          unitConv->getInputPort(0)->connect(gain->getOutputPort(0));
          addFCSModel(unitConv);

          sg->getInputPort("steeringAngle")->connect(unitConv->getOutputPort(0));
        }
        
        std::string brake = stringData((*it)->getElement("brake_group"));
        if (brake == "LEFT") {
          Port* port = lookupJSBExpression("gear/left-brake-pos-norm");
          sg->getInputPort("brakeCommand")->connect(port);
        } else if (brake == "RIGHT") {
          Port* port = lookupJSBExpression("gear/right-brake-pos-norm");
          sg->getInputPort("brakeCommand")->connect(port);
        }
        
        mVehicle->getTopBody()->addInteract(sg);


      } else if (type == "STRUCTURE" || type == "CONTACT") {
        // Very simple contact force. Penalty method.
        SimpleContact* sc = new SimpleContact((*it)->getAttribute("name"));

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
        
        mVehicle->getTopBody()->addInteract(sc);

      } else if (type == "TAILHOOK") /*FIXME*/ {
      } else if (type == "LAUNCHBAR") {
      } else {
        return error("Unknown groundreactions component of type " + type);
      }
    } else {
      return error("Unknown groundreactions tag " + (*it)->getName());
    }
  }
  


//   // Undercarriage parsing.
//   std::stringstream datastr(data);
//   unsigned gearNumber = 0;

//   while (datastr) {
//     std::string uctype;
//     datastr >> uctype;
    
//     std::stringstream sstr;
//     sstr << gearNumber;
//     std::string numStr = sstr.str();
//     // Increment the gear number
//     ++gearNumber;

//     if (uctype == "AC_GEAR") {
//       std::string name, type, brake, retract;
//       real_type x, y, z, k, d, fs, fd, rr, sa;
//       datastr >> name >> x >> y >> z >> k >> d >> fs >> fd >> rr
//               >> type >> brake >> sa >> retract;

//       if (type == "CASTERING") {
//         // Modelling castering gars as simple contacs without a special
//         // direction
//         SimpleContact* sc = new SimpleContact(name);
//         sc->setPosition(structToBody(Vector3(x, y, z)));

//         sc->setSpringConstant(convertFrom(uPoundForcePFt, k));
//         // FIXME: conversion factor:
//         // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
//         // Note that friction coefficients are different from that but
//         // viscosous friction is just used in that way ...
//         sc->setSpringDamping(convertFrom(uPoundForcePFt, d));
//         sc->setFrictionCoeficient(0.1*fs);
        
//         mVehicle->getTopBody()->addInteract(sc);

//       } else {
//         // For jsbsim use simple gears
//         SimpleGear* sg = new SimpleGear(name);
//         sg->setPosition(structToBody(Vector3(x, y, z)));
        
//         sg->setSpringConstant(convertFrom(uPoundForcePFt, k));
//         // FIXME: conversion factor:
//         // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
//         // Note that friction coefficients are different from that but
//         // viscosous friction is just used in that way ...
//         sg->setSpringDamping(convertFrom(uPoundForcePFt, d));
//         sg->setFrictionCoeficient(fs);
        
//         // Connect apprioriate input and output models

//         // FIXME
//         // missing output properties are "wow" and "tire-pressure-norm"

//         if (retract == "RETRACT") {
//           Port* port = lookupJSBExpression("gear/gear-pos-norm");
//           sg->getInputPort("enabled")->connect(port);
//           // Well, connect that directly to the input
//           addOutputModel(port, "Gear " + numStr + " Position",
//                          "gear/gear[" + numStr + "]/position-norm");
//         }

//         if (type == "STEERABLE") {
//           // FIXME: FCS might later define something for that gain ...
// //           prop = lookupJSBExpression("fcs/steer-pos-deg[" + numStr + "]");
//           Port* port = lookupJSBExpression("fcs/steer-cmd-norm");
//           Gain* gain = new Gain(name + " Steer Gain");
//           gain->setGain(sa);
//           gain->getInputPort(0)->connect(port);
//           addFCSModel(gain);
//           addOutputModel(port, "Gear " + numStr + " Steering Output",
//                          "gear/gear[" + numStr + "]/steering-norm");


//           UnitConversionModel* unitConv
//             = new UnitConversionModel(name + " Degree Conversion",
//                                       UnitConversionModel::UnitToSi,
//                                       uDegree);
//           unitConv->getInputPort(0)->connect(gain->getOutputPort(0));
//           addFCSModel(unitConv);

//           sg->getInputPort("steeringAngle")->connect(unitConv->getOutputPort(0));
//         }
        
//         if (brake == "LEFT") {
//           Port* port = lookupJSBExpression("gear/left-brake-pos-norm");
//           sg->getInputPort("brakeCommand")->connect(port);
//         } else if (brake == "RIGHT") {
//           Port* port = lookupJSBExpression("gear/right-brake-pos-norm");
//           sg->getInputPort("brakeCommand")->connect(port);
//         }
        
//         mVehicle->getTopBody()->addInteract(sg);
//       }
      
//     } else if (uctype == "AC_LAUNCHBAR") {
//       std::string d;
//       datastr >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d;

//     } else if (uctype == "AC_HOOK") {
//       std::string d;
//       datastr >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d;

//     } else if (uctype == "AC_F18MLG") { 
//       /// Well, that here is exactly how it should not be,
//       /// but for initial testing of a new unfinished fdm ...
//       std::string name, brake;
//       Vector3 compressJointPos;
//       real_type pullPress;
//       real_type pushPress;
//       real_type area;
//       real_type minCompr;
//       real_type maxCompr;
//       real_type minDamp;
//       real_type maxDamp;
//       real_type armLength;
//       real_type wheelDiam;
//       real_type tireSpring, tireDamp;


//       datastr >> name >> brake
//               >> compressJointPos(1)
//               >> compressJointPos(2)
//               >> compressJointPos(3)
//               >> pullPress >> pushPress
//               >> area
//               >> minCompr
//               >> maxCompr
//               >> minDamp
//               >> maxDamp
//               >> armLength
//               >> wheelDiam
//               >> tireSpring >> tireDamp;

//       // Well this is come hardcoding, but as a demo built from within the
//       // legacy JSBSim format this is ok :)

//       // This is the movable part of the strut, doing the compression
//       RigidBody* arm = new RigidBody(name + " Arm");
//       mVehicle->getMultiBodySystem()->addRigidBody(arm);
//       arm->addInteract(new Mass(name + " Strut Mass", inertiaFrom(Vector3(-1, 0, 0), SpatialInertia(80))));

//       // Connect that with a revolute joint to the main body
//       RevoluteJoint* rj = new RevoluteJoint(name + " Arm Joint");
//       mVehicle->getTopBody()->addInteract(rj);
//       arm->setInboardJoint(rj);
//       rj->setJointAxis(Vector3(0, 1, 0));
//       rj->setJointPos(0);
//       rj->setJointVel(0);
//       rj->setPosition(structToBody(compressJointPos));
//       rj->setOrientation(Quaternion::unit());

// #if 0
//       // Well, we use an air spring for that. It is directly in the
//       // revolute joint. That is wrong, but at the moment aprioriate.
//       AirSpring* aoDamp = new AirSpring(name + " Air Spring Force");
//       aoDamp->setPullPressure(pullPress);
//       aoDamp->setPushPressure(pushPress);
//       aoDamp->setArea(area);
//       aoDamp->setMinCompression(minCompr);
//       aoDamp->setMaxCompression(maxCompr);
//       aoDamp->setMinDamperConstant(minDamp);
//       aoDamp->setMaxDamperConstant(maxDamp);
//       // That one reads the joint position and velocity ...
//       aoDamp->getInputPort(0)->connect(rj->getOutputPort(0));
//       aoDamp->getInputPort(1)->connect(rj->getOutputPort(1));
//       // ... and provides an output force
//       rj->getInputPort(0)->connect(aoDamp->getOutputPort(0));
//       addMultiBodyModel(aoDamp);
// #else
//       LineForce* lineForce = new LineForce(name + " Air Spring LineForce");
//       /// FIXME that ordering in attachment is messy!
//       lineForce->setPosition0(structToBody(compressJointPos) - Vector3(0.1, 0, 0.5));
//       lineForce->setPosition1(Vector3(-0.5, 0, 0));
//       mVehicle->getTopBody()->addInteract(lineForce);
//       arm->addInteract(lineForce);

//       AirSpring* aoDamp = new AirSpring(name + " Air Spring Force");
//       aoDamp->setPullPressure(pullPress);
//       aoDamp->setPushPressure(pushPress);
//       aoDamp->setArea(area);
//       aoDamp->setMinCompression(minCompr);
//       aoDamp->setMaxCompression(maxCompr);
//       aoDamp->setMinDamperConstant(minDamp);
//       aoDamp->setMaxDamperConstant(maxDamp);
//       addMultiBodyModel(aoDamp);

//       // That one reads the joint position and velocity ...
//       aoDamp->getInputPort(0)->connect(lineForce->getOutputPort(0));
//       aoDamp->getInputPort(1)->connect(lineForce->getOutputPort(1));
//       // ... and provides an output force
//       lineForce->getInputPort(0)->connect(aoDamp->getOutputPort(0));
// #endif

//       // Attach a wheel to that strut part.
//       attachWheel(name, Vector3(-armLength, 0, 0), brake, numStr, wheelDiam,
//                   tireSpring, tireDamp, arm);

//       Port* port = rj->getOutputPort(0);
//       addOutputModel(port, "Gear " + numStr + " Compression",
//                      "gear/gear[" + numStr + "]/compression-rad");

//       /// FIXME add a retract joint ...
//       port = lookupJSBExpression("gear/gear-pos-norm");
//       addOutputModel(port, "Gear " + numStr + " Position",
//                      "gear/gear[" + numStr + "]/position-norm");

//     } else if (uctype == "AC_CLG") {
//       std::string name, brake, steer;
//       Vector3 compressJointPos;
//       real_type pullPress;
//       real_type pushPress;
//       real_type area;
//       real_type minCompr;
//       real_type maxCompr;
//       real_type minDamp;
//       real_type maxDamp;
//       real_type wheelDiam;
//       real_type tireSpring, tireDamp;

//       datastr >> name >> brake
//               >> compressJointPos(1)
//               >> compressJointPos(2)
//               >> compressJointPos(3)
//               >> pullPress >> pushPress
//               >> area
//               >> minCompr
//               >> maxCompr
//               >> minDamp
//               >> maxDamp
//               >> wheelDiam
//               >> tireSpring >> tireDamp
//               >> steer;

//       // Well this is come hardcoding, but as a demo built from within the
//       // legacy JSBSim format this is ok :)

//       // Model steering here ...
//       // normally we connect the compressible part to the top level body, but
//       // in case of steering this is no longer true.
//       RigidBody* strutParent = mVehicle->getTopBody();
//       if (steer == "STEERABLE") {
//         // A new part modelling the steering
//         RigidBody* steer = new RigidBody(name + " Steer");
//         mVehicle->getMultiBodySystem()->addRigidBody(steer);

//         // connect that via a revolute joint to the toplevel body.
//         // Note the 0.05m below, most steering wheels have some kind of
//         // castering auto line up behavour. That is doe with this 0.05m.
//         RevoluteActuator* sj = new RevoluteActuator(name + " Steer Joint");
//         strutParent->addInteract(sj);
//         steer->setInboardJoint(sj);
//         sj->setJointAxis(Vector3(0, 0, 1));
//         sj->setJointPos(0);
//         sj->setJointVel(0);
//         sj->setPosition(structToBody(compressJointPos)
//                         + Vector3(0.05, 0, 0));
//         sj->setOrientation(Quaternion::unit());

//         Port* port = lookupJSBExpression("fcs/steer-cmd-norm");
//         sj->getInputPort(0)->connect(port);
        
//         strutParent = steer;
        
//         // Prepare outputs
//         port = sj->getOutputPort(0);
//         std::string nameBase = "Steering " + numStr + " Position";
//         addOutputModel(port, nameBase,
//                        "gear/gear[" + numStr + "]/steering-pos-rad");
//         UnitConversionModel* unitModel
//           = new UnitConversionModel(nameBase + " converter",
//                                     UnitConversionModel::SiToUnit, uDegree);
//         unitModel->getInputPort(0)->connect(port);
//         addFCSModel(unitModel);
//         addOutputModel(unitModel->getOutputPort(0), nameBase + " Deg",
//                        "gear/gear[" + numStr + "]/steering-pos-deg");
//       }


//       // Now the compressible part of the strut
//       RigidBody* arm = new RigidBody(name + " Strut");
//       mVehicle->getMultiBodySystem()->addRigidBody(arm);
//       arm->addInteract(new Mass(name + " Strut Mass", inertiaFrom(Vector3(0, 0, 1), SpatialInertia(100))));

//       // This time it is a prismatic joint
//       PrismaticJoint* pj = new PrismaticJoint(name + " Compress Joint");
//       strutParent->addInteract(pj);
//       arm->setInboardJoint(pj);
//       pj->setJointAxis(Vector3(0, 0, -1));
//       if (strutParent == mVehicle->getTopBody())
//         pj->setPosition(structToBody(compressJointPos));
//       else
//         pj->setPosition(Vector3(-0.05, 0, 0));

//       // With an air spring
//       AirSpring* aoDamp = new AirSpring(name + " Air Spring Force");
//       aoDamp->setPullPressure(pullPress);
//       aoDamp->setPushPressure(pushPress);
//       aoDamp->setArea(area);
//       aoDamp->setMinCompression(minCompr);
//       aoDamp->setMaxCompression(maxCompr);
//       aoDamp->setMinDamperConstant(minDamp);
//       aoDamp->setMaxDamperConstant(maxDamp);
//       pj->getInputPort(0)->connect(aoDamp->getOutputPort(0));
//       aoDamp->getInputPort(0)->connect(pj->getOutputPort(0));
//       aoDamp->getInputPort(1)->connect(pj->getOutputPort(1));
//       addMultiBodyModel(aoDamp);

//       // Attach a wheel to that strut part.
//       attachWheel(name, Vector3::zeros(), brake, numStr, wheelDiam,
//                   tireSpring, tireDamp, arm);

//       // Prepare some outputs ...
//       Port* port = pj->getOutputPort(0);
//       addOutputModel(port, "Gear " + numStr + " Compression",
//                      "gear/gear[" + numStr + "]/compression-m");

//       port = lookupJSBExpression("gear/gear-pos-norm");
//       addOutputModel(port, "Gear " + numStr + " Position",
//                      "gear/gear[" + numStr + "]/position-norm");

//     } else if (uctype == "AC_CONTACT") {
//       std::string name, type, brake, retract;
//       real_type x, y, z, k, d, fs, fd, rr, sa;
//       datastr >> name >> x >> y >> z >> k >> d >> fs >> fd >> rr
//               >> type >> brake >> sa >> retract;

//       // Very simple contact force. Penalty method.
//       SimpleContact* sc = new SimpleContact(name);
//       sc->setPosition(structToBody(Vector3(x, y, z)));

//       sc->setSpringConstant(convertFrom(uPoundForcePFt, k));
//       // FIXME: conversion factor:
//       // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
//       // Note that friction coefficients are different from that but
//       // viscosous friction is just used in that way ...
//       sc->setSpringDamping(convertFrom(uPoundForcePFt, d));
//       sc->setFrictionCoeficient(fs);

//       mVehicle->getTopBody()->addInteract(sc);
//     }
//   }

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
    
//   } else if (engineTopElem->getName() == "FG_PISTON") {
//     if (!convertPiston(engineTopElem, number, 0))
//       return error("Error readinge piston configuration");

//   } else if (engineTopElem->getName() == "FG_ROCKET") {
//     return error("FG_ROCKET's are not (yet?) supported!");

//   } else if (engineTopElem->getName() == "FG_ELECTRIC") {
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
                             Port* thrusterDriver)
{
  // At the moment we have a *very* insufficient engine, just modulate the
  // thrust between 0 and max
  real_type maxThrust = realData(turbine->getElement("milthrust"), 0);

  std::string namestr = "Engine<" + number + ">";
  ConstModel* fullForce = new ConstModel(namestr + " full");
  fullForce->setValue(Vector6(0, 0, 0, convertFrom(uPoundForce, maxThrust), 0, 0));
  addMultiBodyModel(fullForce);

  Product* prod = new Product(namestr + " modulation");
  std::string throttlename = "fcs/throttle-cmd-norm[" + number + "]";
  prod->getInputPort(0)->connect(lookupJSBExpression(throttlename));
  prod->getInputPort(1)->connect(fullForce->getOutputPort(0));
  addMultiBodyModel(prod);

  ExternalForceModel* engineForce = new ExternalForceModel(namestr);
  engineForce->setPosition(pos);
  engineForce->setOrientation(orientation);

  engineForce->getInputPort(0)->connect(prod->getOutputPort(0));

  mVehicle->getTopBody()->addInteract(engineForce);

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
  SharedPtr<Port> out;
  SharedPtr<Port> normOut;

  // JSBSim FCS output values contain some implicit rules.
  // From the component name a default output property is formed.
  // If we find an OUTPUT line this is the output value too.
  // So collect them first and register them later when the final output
  // expression is known.
  std::string name = fcsComponent->getAttribute("name");
  std::string type = fcsComponent->getName();

  if (type == "SUMMER" || type == "summer") {
    SharedPtr<Summer> summer = new Summer(name);
    std::list<std::string> inputs = getInputs(fcsComponent);
    std::list<std::string>::iterator it = inputs.begin();
    unsigned n = 0;
    while (it != inputs.end()) {
      summer->setNumSummands(n+1);
      summer->getInputPort(n)->connect(lookupJSBExpression(*it));
      ++n;
      ++it;
    }
    addFCSModel(summer);
    real_type b = realData(fcsComponent->getElement("bias"), 0);
    if (b != 0) {
      SharedPtr<Bias> bias = new Bias(name + " Bias");
      bias->getInputPort(0)->connect(summer->getOutputPort(0));
      addFCSModel(bias);
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
    std::string token = stringData(fcsComponent->getElement("input"));
    deadband->getInputPort(0)->connect(lookupJSBExpression(token));
    deadband->setWidth(realData(fcsComponent->getElement("width"), 0));
    model = deadband;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "GRADIENT" || type == "gradient") {
    SharedPtr<TimeDerivative> timeDeriv = new TimeDerivative(name);
    std::string token = stringData(fcsComponent->getElement("input"));
    timeDeriv->getInputPort(0)->connect(lookupJSBExpression(token));
    model = timeDeriv;
    addFCSModel(model);
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
    std::string token = stringData(fcsComponent->getElement("input"));
    model->getInputPort(0)->connect(lookupJSBExpression(token));
    addFCSModel(model);
    out = kinemat->getOutputPort();
    normOut = kinemat->getOutputNormPort();

  } else if (type == "PURE_GAIN" || type == "pure_gain") {
    SharedPtr<Gain> gain = new Gain(name);
    std::string token = stringData(fcsComponent->getElement("input"));
    gain->getInputPort(0)->connect(lookupJSBExpression(token));
    gain->setGain(realData(fcsComponent->getElement("gain"), 1));
    model = gain;
    addFCSModel(model);
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
    model = asScale->getModelGroup();
    token = stringData(fcsComponent->getElement("input"));
    model->getInputPort(0)->connect(lookupJSBExpression(token));
    addFCSModel(model);
    out = asScale->getOutputPort();
    normOut = asScale->getOutputNormPort();

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
    std::string token = stringData(fcsComponent->getElement("input"));
    model->getInputPort(0)->connect(lookupJSBExpression(token));
    token = stringData(tbl->getElement("independentVar"));
    model->getInputPort(1)->connect(lookupJSBExpression(token));
    addFCSModel(model);

    out = sGain->getOutputPort();

  } else if (type == "INTEGRATOR" || type == "integrator") {
    model = new DiscreteIntegrator(name);
    std::string token = stringData(fcsComponent->getElement("input"));
    model->getInputPort(0)->connect(lookupJSBExpression(token));
    out = model->getOutputPort(0);
    addFCSModel(model);
    real_type c1 = realData(fcsComponent->getElement("c1"), 1);
    if (c1 != 1) {
      SharedPtr<Gain> gain = new Gain(name + " Gain");
      gain->getInputPort(0)->connect(model->getOutputPort(0));
      gain->setGain(c1);
      model = gain;
      addFCSModel(model);
      out = model->getOutputPort(0);
    }

  } else if (type == "LAG_FILTER" || type == "lag_filter") {
    //   C1
    // ------
    // s + C1
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    std::string token = stringData(fcsComponent->getElement("input"));
    discreteTransfFunc->getInputPort(0)->connect(lookupJSBExpression(token));

    Vector v(1);
    v(1) = realData(fcsComponent->getElement("c1"), 1);
    discreteTransfFunc->setNumerator(v);
    v.resize(2);
    v(1) = 1;
    v(2) = realData(fcsComponent->getElement("c1"), 1);
    discreteTransfFunc->setDenominator(v);
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "LEAD_LAG_FILTER" || type == "lead_lag_filter") {
    // C1*s + C2
    // ---------
    // C3*s + C4
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    std::string token = stringData(fcsComponent->getElement("input"));
    discreteTransfFunc->getInputPort(0)->connect(lookupJSBExpression(token));

    Vector v(2);
    v(1) = realData(fcsComponent->getElement("c1"), 1);
    v(2) = realData(fcsComponent->getElement("c2"), 1);
    discreteTransfFunc->setNumerator(v);
    v(1) = realData(fcsComponent->getElement("c3"), 1);
    v(2) = realData(fcsComponent->getElement("c4"), 1);
    discreteTransfFunc->setDenominator(v);

    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "WASHOUT_FILTER" || type == "washout_filter") {
    //   s
    // ------
    // s + C1
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    std::string token = stringData(fcsComponent->getElement("input"));
    discreteTransfFunc->getInputPort(0)->connect(lookupJSBExpression(token));

    Vector v(1);
    v(1) = 1;
    discreteTransfFunc->setNumerator(v);
    v.resize(2);
    v(1) = 1;
    v(2) = realData(fcsComponent->getElement("c1"), 1);
    discreteTransfFunc->setDenominator(v);
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "SECOND_ORDER_FILTER" || type == "second_order_filter") {
    // C1*s + C2*s + C3
    // ----------------
    // C4*s + C5*s + C6
    SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    std::string token = stringData(fcsComponent->getElement("input"));
    discreteTransfFunc->getInputPort(0)->connect(lookupJSBExpression(token));

    Vector v(3);
    v(1) = realData(fcsComponent->getElement("c1"), 1);
    v(2) = realData(fcsComponent->getElement("c2"), 1);
    v(3) = realData(fcsComponent->getElement("c3"), 1);
    discreteTransfFunc->setNumerator(v);
    v(1) = realData(fcsComponent->getElement("c4"), 1);
    v(2) = realData(fcsComponent->getElement("c5"), 1);
    v(3) = realData(fcsComponent->getElement("c6"), 1);
    discreteTransfFunc->setDenominator(v);

    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "SENSOR" || type == "sensor") {
    std::cout << "Ignoring SENSOR" << std::endl;

  } else
    return error("Unknown FCS COMPONENT type: \"" + type
                 + "\". Ignoring whole FCS component \"" + name + "\"" );

  OpenFDMAssert(out->isConnected());
  if (!normOut || !normOut->isConnected())
    normOut = out;

  // Register all output property names.
  std::list<std::string> outlist;
  outlist.push_back(std::string("fcs/") + normalizeComponentName(name));
  if (fcsComponent->getElement("output"))
    outlist.push_back(stringData(fcsComponent->getElement("output")));
  std::list<std::string>::iterator it;
  for (it = outlist.begin(); it != outlist.end(); ++it) {
    std::string propName = *it;
    registerJSBExpression(propName, out);

    // Well, just an other kind of black magic ...
    if (propName == "fcs/elevator-pos-rad") {
      registerJSBExpression("fcs/elevator-pos-norm", normOut);
    } else if (propName == "fcs/left-aileron-pos-rad" ||
               propName == "fcs/aileron-pos-rad") {
      registerJSBExpression("fcs/left-aileron-pos-norm", normOut);
    } else if (propName == "fcs/right-aileron-pos-rad") {
      registerJSBExpression("fcs/right-aileron-pos-norm", normOut);
    } else if (propName == "fcs/rudder-pos-rad") {
      registerJSBExpression("fcs/rudder-pos-norm", normOut);
    } else if (propName == "fcs/speedbrake-pos-rad") {
      registerJSBExpression("fcs/speedbrake-pos-norm", normOut);
    } else if (propName == "fcs/spoiler-pos-rad") {
      registerJSBExpression("fcs/spoiler-pos-norm", normOut);
    } else if (propName == "fcs/flap-pos-deg") {
      registerJSBExpression("fcs/flap-pos-norm", normOut);
    }
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
      addMultiBodyModel(sum);
      Port* port = sum->getOutputPort(0);

      if (axisname == "LIFT") {
        port = addMultiBodyFromUnit("LIFT unit convert", uPoundForce, port);
        mAeroForce->getInputPort("lift")->connect(port);
      } else if (axisname == "DRAG") {
        port = addMultiBodyFromUnit("DRAG unit convert", uPoundForce, port);
        mAeroForce->getInputPort("drag")->connect(port);
      } else if (axisname == "SIDE") {
        port = addMultiBodyFromUnit("SIDE unit convert", uPoundForce, port);
        mAeroForce->getInputPort("side")->connect(port);
      } else if (axisname == "ROLL") {
        port = addMultiBodyFromUnit("ROLL unit convert", uPoundForceFt, port);
        mAeroForce->getInputPort("roll")->connect(port);
      } else if (axisname == "PITCH") {
        port = addMultiBodyFromUnit("PITCH unit convert", uPoundForceFt, port);
        mAeroForce->getInputPort("pitch")->connect(port);
      } else if (axisname == "YAW") {
        port = addMultiBodyFromUnit("YAW unit convert", uPoundForceFt, port);
        mAeroForce->getInputPort("yaw")->connect(port);
      } else
        return error("Unknown aerodynamic axis!");
    }
  }

  return true;
}

bool
JSBSimReader::convertFunction(const XMLElement* function,
                              Summer* sum)
{
  std::string bindName = function->getAttribute("name");
  std::string name = bindName;
  std::string::size_type slachPos = bindName.rfind('/');
  if (slachPos != std::string::npos)
    name = name.substr(slachPos+1);
  Port* port = 0;
  std::list<const XMLElement*> elems = function->getElements();
  std::list<const XMLElement*>::const_iterator it;
  for (it = elems.begin(); it != elems.end(); ++it) {
    if ((*it)->getName() == "description") {
      // Just ignore
    } else if ((*it)->getName() == "product") {
      SharedPtr<Product> prod = new Product(name + " product");
      addMultiBodyModel(prod);
      std::list<Port*> inputs = readFunctionInputs(*it, name);
      if (inputs.empty())
        return error("Cannot read product inputs!");
      unsigned i = 0;
      std::list<Port*>::iterator iit = inputs.begin();
      while (iit != inputs.end()) {
        prod->setNumFactors(i+1);
        prod->getInputPort(i++)->connect(*iit++);
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
    sum->getInputPort(num)->connect(port);
  }

  return true;
}

std::list<Port*>
JSBSimReader::readFunctionInputs(const XMLElement* operationTag,
                                 const std::string& name)
{
  std::list<Port*> inputs;
  std::list<const XMLElement*> args = operationTag->getElements();
  std::list<const XMLElement*>::const_iterator ait;
  for (ait = args.begin(); ait != args.end(); ++ait) {
    if ((*ait)->getName() == "value") {
      SharedPtr<ConstModel> constModel = new ConstModel(name + " Constant");
      addMultiBodyModel(constModel);
      std::stringstream stream((*ait)->getData());
      Matrix v(1, 1);
      stream >> v(1, 1);
      constModel->setValue(v);
      inputs.push_back(constModel->getOutputPort(0));
    } else if ((*ait)->getName() == "property") {
      inputs.push_back(lookupJSBExpression(stringData(*ait)));
    } else if ((*ait)->getName() == "table") {
      unsigned dim = getNumTableDims(*ait);
      if (dim == 1) {
        TableData<1> data;
        TableLookup lookup;
        if (!readTable1D(*ait, data, lookup)) {
          error("Cannot read 1D table data.");
          return std::list<Port*>();
        }
        std::string token = stringData((*ait)->getElement("independentVar"));
        Port* port = getTablePrelookup(name + " lookup",
                                       lookupJSBExpression(token),
                                       lookup);

        SharedPtr<Table1D> table = new Table1D(name + " Table");
        table->getInputPort(0)->connect(port);
        table->setTableData(data);
        addMultiBodyModel(table);
        inputs.push_back(table->getOutputPort(0));

      } else if (dim == 2) {
        TableData<2> data;
        TableLookup lookup[2];
        if (!readTable2D(*ait, data, lookup)) {
          error("Cannot read 2D table data.");
          return std::list<Port*>();
        }

        std::list<const XMLElement*> indeps
          = (*ait)->getElements("independentVar");
        if (indeps.size() != 2) {
          error("2DTable data does not have 2 inputs!");
          return std::list<Port*>();
        }
        std::string rowInput = indepData(indeps, "row");
        std::string colInput = indepData(indeps, "column");

        Port* rPort = getTablePrelookup(name + " row lookup",
                                        lookupJSBExpression(rowInput),
                                        lookup[0]);
        Port* cPort = getTablePrelookup(name + " column lookup",
                                        lookupJSBExpression(colInput),
                                        lookup[1]);

        SharedPtr<Table2D> table = new Table2D(name  + " Table");
        table->getInputPort(0)->connect(rPort);
        table->getInputPort(1)->connect(cPort);
        table->setTableData(data);
        addMultiBodyModel(table);
        inputs.push_back(table->getOutputPort(0));

      } else if (dim == 3) {
        TableData<3> data;
        TableLookup lookup[3];
        if (!readTable3D(*ait, data, lookup)) {
          error("Cannot read 1D table data.");
          return std::list<Port*>();
        }

        std::list<const XMLElement*> indeps
          = (*ait)->getElements("independentVar");
        if (indeps.size() != 3) {
          error("3DTable data does not have 3 inputs!");
          return std::list<Port*>();
        }
        std::string rowInput = indepData(indeps, "row");
        std::string colInput = indepData(indeps, "column");
        std::string pageInput = indepData(indeps, "table");

        error("FIXME 3DTable not yet implemented!");
        return std::list<Port*>();

      } else {
        error("Unknown tabel dimension.");
        return std::list<Port*>();
      }
    } else {
      error("Unknown function input");
      return std::list<Port*>();
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
