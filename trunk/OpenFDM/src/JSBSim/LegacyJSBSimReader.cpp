/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>

#include <OpenFDM/Vector.h>
#include <OpenFDM/Matrix.h>
#include <OpenFDM/Quaternion.h>

#include <OpenFDM/AeroForce.h>
#include <OpenFDM/Bias.h>
#include <OpenFDM/DeadBand.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Expression.h>
#include <OpenFDM/TransferFunction.h>
#include <OpenFDM/DirectForce.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Input.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/LinearSpring.h>
#include <OpenFDM/AirSpring.h>
#include <OpenFDM/PrismaticJoint.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/RevoluteJoint.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/Sensor.h>
#include <OpenFDM/SimpleContact.h>
#include <OpenFDM/SimpleGear.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/TimeDerivative.h>
#include <OpenFDM/UnaryFunctionModel.h>
#include <OpenFDM/Units.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/WheelContact.h>
#include <OpenFDM/LineActuator.h>
#include <OpenFDM/DiscBrake.h>

#include <OpenFDM/ReaderWriter.h>
#include <OpenFDM/XML/Tablereader.h>
#include <OpenFDM/XML/XMLReader.h>

#include "LegacyJSBSimReader.h"

namespace OpenFDM {

LegacyJSBSimReader::LegacyJSBSimReader(void)
{
}

LegacyJSBSimReader::~LegacyJSBSimReader(void)
{
}

void
LegacyJSBSimReader::reset(void)
{
  // Throw away any possibly loaded vehicle
  mVehicle = 0;
}

void
LegacyJSBSimReader::addAircraftPath(const std::string& path)
{
  mAircraftPath.push_back(path);
}

void
LegacyJSBSimReader::addEnginePath(const std::string& path)
{
  mEnginePath.push_back(path);
}

bool
LegacyJSBSimReader::loadAircraft(const std::string& acFileName)
{
  // Reset the vehicle.
  resetErrorState();

  mExpressionTable.clear();
  // Allocate a new vehicle
  mVehicle = new Vehicle;
  mAeroForce = new AeroForce(mVehicle->getEnvironment(), "Aerodynamic force");
  mVehicle->getTopBody()->addMultiBodyModel(mAeroForce);
  // Default discrete stepsize of JSBSim
  mVehicle->getModelGroup()->addSampleTime(SampleTime(1.0/120));

  // Create the aerodynamic properties of JSBSim.
  makeAeroprops();

  // Try to find the given file on the given search path
  std::ifstream acFileStream;
  if (!openFile(mAircraftPath, acFileName, acFileStream))
    return false;

  // Need to fix that one due to the braindead errors in the "xml" format
  std::stringstream fixedAcStream;
  acFileStream >> fixedAcStream.rdbuf();
  acFileStream.close();
  std::string s = fixedAcStream.str();
  fixupTEST(s);
  fixedAcStream.str(s);

  // Parse the file and generate a dom like representation of it
  XMLDomParser parser;
  if (!parser.parseXML(fixedAcStream))
    return false;

  return convertDocument(parser.getDocument());
}

bool
LegacyJSBSimReader::openFile(const std::list<std::string>& paths,
                             const std::string& file, std::ifstream& fs)
{
  // Search the given path for such files.
  std::list<std::string>::const_iterator it;
  for (it = paths.begin(); it != paths.end(); ++it) {
    // We do no longer need to care for different slaches ... Yippie!!!!!
    std::string filename = (*it) + '/' + file;
    fs.open(filename.c_str());
    if (fs.is_open())
      return true;
  }

  return false;
}

void
LegacyJSBSimReader::fixupTEST(std::string& s)
{
  std::string::size_type pos = 0;
  while ((pos = s.find("<TEST", pos)) != std::string::npos) {
    pos = s.find(">", pos);
    if (pos == std::string::npos)
      break;
    ++pos;
    std::string::size_type testend = s.find("</TEST>", pos);

    std::string::size_type reppos;
    while ((reppos = s.find(">", pos)) < testend)
      s.replace(reppos, 1, "&gt;"), testend += 3;
    while ((reppos = s.find(">=", pos)) < testend)
      s.replace(reppos, 2, "&gt;="), testend += 3;
    while ((reppos = s.find("<", pos)) < testend)
      s.replace(reppos, 1, "&lt;"), testend += 3;
    while ((reppos = s.find("<=", pos)) < testend)
      s.replace(reppos, 2, "&lt;="), testend += 3;

    pos = testend;
  }
}

std::string
LegacyJSBSimReader::propNameFromJSBSim(const std::string& jsbSymbol)
{
  /// Convert a JSBSim, property name like it can appear in config files
  /// into a flightgear property name. That is it strips the optional - sign
  /// and prepends the property with fdm/jsbsim/

  if (jsbSymbol.empty())
    return jsbSymbol;

  std::string propName = jsbSymbol;

  // Strip the minus sign
  if (propName[0] == '-')
    propName.erase(0, 1);
  if (propName.empty())
    return propName;

  if (propName[0] == '/')
    // In case of an 'absolute' property, strip that leading /
    propName.erase(0, 1);
  else
    // In case of a 'relative' property, add the jsb prefix
    propName = "fdm/jsbsim/" + propName;

  return propName;
}

bool
LegacyJSBSimReader::propMinusFromJSBSim(const std::string& jsbSymbol)
{
  /// Returns true in case the JSBSim property contains a minus
  return 0 < jsbSymbol.size() && jsbSymbol[0] == '-';
}

std::string
LegacyJSBSimReader::normalizeComponentName(const std::string& name)
{
  // Convert a fcs name to the output property name where its output will be
  // available automagically. Well that kind of black magic is really horrible
  // for users not familiar with the source
  std::string ret;
  std::string::const_iterator it = name.begin();
  while (it != name.end()) {
    if (isspace(*it) || *it == '_')
      ret += '-';
    else
      ret += tolower(*it);
    ++it;
  }
  return ret;
}

Port*
LegacyJSBSimReader::lookupJSBExpression(const std::string& name)
{
  // Convert to something being able to look up
  std::string propName = propNameFromJSBSim(name);
  
  Port* port;
  if (mExpressionTable.count(propName) <= 0) {
    // Not yet available, so look and see if it is an input
    port = createAndScheduleInput(propName);

    // Ok, still not available, create a constant zero thing and bail out ...
    if (!port || !port->isConnected()) {
      std::cerr << "Creating expression \"" << propName << "\"" << std::endl;
      port = new Port;
      port->setProperty(Property(new ConstExpressionPropertyImpl<real_type>(0)));
      return port;
    }
    
  } else {
    // Just get that one, if we already have it
    port = mExpressionTable[propName];
  }

  // If we need the negative input, just multiply with a negative gain
  if (propMinusFromJSBSim(name))
    return addInverterModel(name.substr(name.rfind('/')+1), port);
  else
    return port;
}

void
LegacyJSBSimReader::registerExpression(const std::string& name, Port* port)
{
  if (name.size() <= 0)
    return;
  if (!port || !port->isConnected())
    return;
  if (0 < mExpressionTable.count(name)) {
    std::cerr << "Already have an expression for " << name << std::endl;
    return;
  }

  mExpressionTable[name] = port;
}

void
LegacyJSBSimReader::registerJSBExpression(const std::string& name,
                                          Port* port)
{
  if (name.size() <= 0)
    return;
  if (!port || !port->isConnected())
    return;
  if (name[0] == '/')
    registerExpression(name.substr(1), port);
  else
    registerExpression("fdm/jsbsim/" + name, port);

  // Well, just an other kind of black magic ...
  if (0 < name.size() && name[0] == '/') {
    addOutputModel(port, name.substr(name.rfind('/')+1), name.substr(1));
  } else if (name == "fcs/elevator-pos-norm") {
    addOutputModel(port, name.substr(4),
                   "surface-positions/elevator-pos-norm");
  } else if (name == "fcs/left-aileron-pos-norm" ||
             name == "fcs/aileron-pos-norm") {
    addOutputModel(port, name.substr(4),
                   "surface-positions/left-aileron-pos-norm");
  } else if (name == "fcs/right-aileron-pos-norm") {
    addOutputModel(port, name.substr(4),
                   "surface-positions/right-aileron-pos-norm");
  } else if (name == "fcs/rudder-pos-norm") {
    addOutputModel(port, name.substr(4),
                   "surface-positions/rudder-pos-norm");
  } else if (name == "fcs/speedbrake-pos-norm") {
    addOutputModel(port, name.substr(4),
                   "surface-positions/speedbrake-pos-norm");
  } else if (name == "fcs/spoiler-pos-norm") {
    addOutputModel(port, name.substr(4),
                   "surface-positions/spoiler-pos-norm");
  } else if (name == "fcs/flap-pos-norm") {
    addOutputModel(port, name.substr(4),
                   "surface-positions/flap-pos-norm");
  }
}

Port*
LegacyJSBSimReader::createAndScheduleInput(const std::string& propName)
{
  // This routine checks if the given propName is a special JSBSim
  // input property. If so, it schedules and registers a discrete input model.
  // If the propName points directly into the controls directory,
  // schedule an input
  if (propName.substr(0, 9) == "controls/") {
//     std::string inputName = propName.substr(propName.rfind('/'));
    std::string inputName = propName;
    return addInputModel("Control Input " + inputName, propName);
  } else {
    Port* port = 0;
    if (propName == "fdm/jsbsim/fcs/aileron-cmd-norm") {
      port = addInputModel("Aileron Input",
                           "controls/flight/aileron");

    } else if (propName == "fdm/jsbsim/fcs/roll-trim-cmd-norm") {
      port = addInputModel("Aileron Trim Input",
                           "controls/flight/aileron-trim");

    } else if (propName == "fdm/jsbsim/fcs/elevator-cmd-norm") {
      port = addInputModel("Elevator Input",
                           "controls/flight/elevator");

    } else if (propName == "fdm/jsbsim/fcs/pitch-trim-cmd-norm") {
      port = addInputModel("Elevator Trim Input",
                           "controls/flight/elevator-trim");

    } else if (propName == "fdm/jsbsim/fcs/rudder-cmd-norm") {
      // FIXME is inverted in JSBSim ...
      port = addInputModel("Rudder Input",
                           "controls/flight/rudder");

    } else if (propName == "fdm/jsbsim/fcs/yaw-trim-cmd-norm") {
      // FIXME also with a minus
      port = addInputModel("Yaw Trim Input",
                           "controls/flight/rudder-trim");

    } else if (propName == "fdm/jsbsim/fcs/steer-cmd-norm") {
      // FIXME is seperate in flightgear ???
      // port = addInputModel("Steering Input", "controls/gear/steering");
      port = addInputModel("Steering Input",
                           "controls/flight/rudder");

    } else if (propName.substr(0, 28) == "fdm/jsbsim/fcs/steer-pos-deg") {
      return lookupJSBExpression("fcs/steer-cmd-norm");

    } else if (propName == "fdm/jsbsim/fcs/flap-cmd-norm") {
      port = addInputModel("Flaps Input",
                           "controls/flight/flaps");

    } else if (propName == "fdm/jsbsim/fcs/speedbrake-cmd-norm") {
      port = addInputModel("Speedbrake Input",
                           "controls/flight/speedbrake");

    } else if (propName == "fdm/jsbsim/fcs/spoiler-cmd-norm") {
      port = addInputModel("Spoiler Input",
                           "controls/flight/spoiler");


    } else if (propName.substr(0, 32) == "fdm/jsbsim/fcs/throttle-cmd-norm") {
      std::string control = "controls/engines/engine" +
        propName.substr(32) + "/throttle";
      port = addInputModel("Throttle Input " + propName.substr(33, 1),
                           control);

    } else if (propName.substr(0, 32) == "fdm/jsbsim/fcs/throttle-pos-norm") {
      std::string cmd = "fcs/throttle-cmd-norm" + propName.substr(32);
      return lookupJSBExpression(cmd);


    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/mixture-cmd-norm") {
      std::string control = "controls/engines/engine" + 
        propName.substr(31) + "/mixture";
      port = addInputModel("Mixture Input " + propName.substr(32, 1), control);

    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/mixture-pos-norm") {
      std::string cmd = "fcs/mixture-cmd-norm" + propName.substr(31);
      return lookupJSBExpression(cmd);


    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/advance-cmd-norm") {
      std::string control = "controls/engines/engine" + 
        propName.substr(31) + "/propeller-pitch";
      port = addInputModel("Propeller Pitch Input " + propName.substr(32, 1),
                           control);

    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/advance-pos-norm") {
      std::string cmd = "fcs/advance-cmd-norm" + propName.substr(31);
      return lookupJSBExpression(cmd);

    } else if (propName == "fdm/jsbsim/gear/gear-cmd-norm") {
      port = addInputModel("Gear Retract Input",
                           "controls/gear/gear-down");

    } else if (propName == "fdm/jsbsim/gear/gear-pos-norm") {
      return lookupJSBExpression("gear/gear-cmd-norm");

    } else if (propName == "controls/gear/brake-parking") {
      port = addInputModel("Parking Brake Input",
                           "controls/gear/brake-parking");

    } else if (propName == "fdm/jsbsim/gear/right-brake-pos-norm") {
      Port* pilotBr = addInputModel("Right Brake Input",
                                    "controls/gear/brake-right");
      Port* copilotBr = addInputModel("Right Copilot Brake Input",
                                      "controls/gear/copilot-brake-right");

      Port* parkBr = lookupJSBExpression("/controls/gear/brake-parking");

      // FIXME: we don't have a max model ...
      MaxExpressionImpl* mex = new MaxExpressionImpl;
      mex->addInputProperty(pilotBr->getProperty());
      mex->addInputProperty(copilotBr->getProperty());
      mex->addInputProperty(parkBr->getProperty());
      port = new Port;
      port->setProperty(Property(mex));

    } else if (propName == "fdm/jsbsim/gear/left-brake-pos-norm") {
      Port* pilotBr = addInputModel("Left Brake Input",
                                    "controls/gear/brake-left");
      Port* copilotBr = addInputModel("Left Copilot Brake Input",
                                      "controls/gear/copilot-brake-left");
      
      Port* parkBr = lookupJSBExpression("/controls/gear/brake-parking");

      // FIXME: we don't have a max model ...
      MaxExpressionImpl* mex = new MaxExpressionImpl;
      mex->addInputProperty(pilotBr->getProperty());
      mex->addInputProperty(copilotBr->getProperty());
      mex->addInputProperty(parkBr->getProperty());
      port = new Port;
      port->setProperty(Property(mex));

    } else if (propName.substr(0, 19) == "fdm/jsbsim/fcs/mag-") {
      // Special absolute modules for fcs/mag-*
      // remove the 'mag-' substring here and use that as input for the
      // Abs block
      std::string name = "fcs/" + propName.substr(19);
      Port* in = lookupJSBExpression(name);
      port = addAbsModel(propName.substr(15), in);

    }

    if (port && port->isConnected())
      registerExpression(propName, port);

    return port;
  }

  return 0;
}

Port*
LegacyJSBSimReader::addInputModel(const std::string& name,
                                  const std::string& propName, real_type gain)
{
  Input* input = new Input(name);
  input->setInputName(propName);
  input->setInputGain(gain);
  addFCSModel(input);
  Port* port = input->getOutputPort(0);
  registerExpression(propName, port);
  return port;
}

void
LegacyJSBSimReader::addOutputModel(Port* out,
                                   const std::string& name,
                                   const std::string& propName, real_type gain)
{
  Output* output = new Output(std::string(name) + " output");
  output->getInputPort(0)->connect(out);
  output->setOutputName(propName);
  output->setOutputGain(gain);
  addFCSModel(output);
}

Port*
LegacyJSBSimReader::addInverterModel(const std::string& name, Port* in)
{
  Gain *gain = new Gain(name + " Inverter");
  gain->getInputPort(0)->connect(in);
  gain->setGain(-1);
  addFCSModel(gain);
  return gain->getOutputPort(0);
}

Port*
LegacyJSBSimReader::addAbsModel(const std::string& name, Port* in)
{
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Abs", new AbsExpressionImpl);
  unary->getInputPort(0)->connect(in);
  addFCSModel(unary);
  return unary->getOutputPort(0);
}

void
LegacyJSBSimReader::addFCSModel(Model* model)
{
  // FIXME
  while (!mVehicle->getModelGroup()->addModel(model)) {
    model->setName(model->getName() + "x");
  }
}

bool
LegacyJSBSimReader::convertDocument(const XMLDocument* jsbDoc)
{
  // Be paranoid ...
  if (!jsbDoc)
    return false;

  const XMLElement* topElem = jsbDoc->getElement();
  if (!topElem)
    return error("No toplevel xml element found");

  if (topElem->getName() != "FDM_CONFIG")
    return error("Toplevel xml element is no FDM_CONFIG");

  if (topElem->getAttribute("VERSION").compare(0, 2, "1.") != 0)
    return error("Toplevel xml element VERSION does not begin with \"1.\"");

  // Parse the metrics section.
  const XMLElement* metricsElem = topElem->getElement("METRICS");
  if (!metricsElem)
    return error("Cannot get METRICS element");
  if (!convertMetrics(metricsElem->getData()))
    return error("Cannot convert METRICS data");

  // Convert all the flight control system elements.
  const XMLElement* fcsElem = topElem->getElement("FLIGHT_CONTROL");
  if (fcsElem) {
    if (!convertFCSList(fcsElem))
      return error("Cannot convert FLIGHT_CONTROL data");
  }
  const XMLElement* autopilotElem = topElem->getElement("AUTOPILOT");
  if (autopilotElem) {
    if (!convertFCSList(autopilotElem))
      return error("Cannot convert AUTOPILOT data");
  }

  // Parse the undercarriage section
  const XMLElement* undercarriageElem = topElem->getElement("UNDERCARRIAGE");
  if (undercarriageElem) {
    if (!convertUndercarriage(undercarriageElem->getData()))
      return error("Cannot convert UNDERCARRIAGE data");
  }

  // Parse the propulsion section.
  const XMLElement* propulsionElem = topElem->getElement("PROPULSION");
  if (propulsionElem) {
    if (!convertPropulsion(propulsionElem))
      return error("Cannot convert PROPULSION data");
  }
  
  // Convert the aerodynamic force.
  const XMLElement* aeroElem = topElem->getElement("AERODYNAMICS");
  if (aeroElem) {
    if (!convertAerodynamics(aeroElem))
      return error("Cannot convert AERODYNAMICS elements");
  }

  return true;
}

/// converts the METRICS data
bool
LegacyJSBSimReader::convertMetrics(const std::string& data)
{
  // Parse the METRICS section.
  // Since all locations within the aircraft are given in the structural frame,
  // we need to know the body frame location in the veihcle first.
  // That is parse all entries and than translate them into OpenFDM parts.

  std::stringstream datastr(data);

  InertiaMatrix I(0, 0, 0, 0, 0, 0);
  real_type mass = 0;
  Vector3 vrp = Vector3::zeros();
  bool haveVrp = false;
  Vector3 ap = Vector3::zeros();
  Vector3 ep = Vector3::zeros();
  bool haveEp = false;
  typedef std::pair<Vector3,real_type> masspoint;
  typedef std::list<masspoint> masslist;
  masslist masses;

  while (datastr) {
    std::string name;
    datastr >> name;
    
    if (name == "AC_WINGAREA") {
      real_type value;
      datastr >> value;
      mAeroForce->setWingArea(convertFrom(uFoot2, value));
    } else if (name == "AC_WINGSPAN") {
      real_type value;
      datastr >> value;
      mAeroForce->setWingSpan(convertFrom(uFoot, value));
    } else if (name == "AC_WINGINCIDENCE") {
      real_type value;
      datastr >> value;
      Port* port = new Port;
      port->setProperty(Property(new ConstExpressionPropertyImpl<real_type>(value)));
      registerJSBExpression("metrics/iw-deg", port);
    } else if (name == "AC_CHORD") {
      real_type value;
      datastr >> value;
      mAeroForce->setCoord(convertFrom(uFoot, value));
    } else if (name == "AC_HTAILAREA") {
      real_type value;
      datastr >> value;
      Port* port = new Port;
      port->setProperty(Property(new ConstExpressionPropertyImpl<real_type>(value)));
      registerJSBExpression("metrics/Sh-sqft", port);
    } else if (name == "AC_HTAILARM") {
      real_type value;
      datastr >> value;
      Port* port = new Port;
      port->setProperty(Property(new ConstExpressionPropertyImpl<real_type>(value)));
      registerJSBExpression("metrics/lh-ft", port);
    } else if (name == "AC_VTAILAREA") {
      real_type value;
      datastr >> value;
      Port* port = new Port;
      port->setProperty(Property(new ConstExpressionPropertyImpl<real_type>(value)));
      registerJSBExpression("metrics/Sv-sqft", port);
    } else if (name == "AC_VTAILARM") {
      real_type value;
      datastr >> value;
      Port* port = new Port;
      port->setProperty(Property(new ConstExpressionPropertyImpl<real_type>(value)));
      registerJSBExpression("metrics/lv-ft", port);
    } else if (name == "AC_IXX") {
      real_type value;
      datastr >> value;
      I(1, 1) = convertFrom(uSlugFt2, value);
    } else if (name == "AC_IYY") {
      real_type value;
      datastr >> value;
      I(2, 2) = convertFrom(uSlugFt2, value);
    } else if (name == "AC_IZZ") {
      real_type value;
      datastr >> value;
      I(3, 3) = convertFrom(uSlugFt2, value);
    } else if (name == "AC_IXY") {
      real_type value;
      datastr >> value;
      I(1, 2) = -convertFrom(uSlugFt2, value);;
    } else if (name == "AC_IXZ") {
      real_type value;
      datastr >> value;
      I(1, 3) = -convertFrom(uSlugFt2, value);
    } else if (name == "AC_IYZ") {
      real_type value;
      datastr >> value;
      I(2, 3) = -convertFrom(uSlugFt2, value);
    } else if (name == "AC_EMPTYWT") {
      datastr >> mass;
      mass = convertFrom(uPoundSealevel, mass);
    } else if (name == "AC_CGLOC") {
      datastr >> mBodyReference(1) >> mBodyReference(2) >> mBodyReference(3);
    } else if (name == "AC_EYEPTLOC") {
      datastr >> ep(1) >> ep(2) >> ep(3);
      haveEp = true;
    } else if (name == "AC_AERORP") {
      datastr >> ap(1) >> ap(2) >> ap(3);
    } else if (name == "AC_VRP") {
      datastr >> vrp(1) >> vrp(2) >> vrp(3);
      haveVrp = true;
    } else if (name == "AC_POINTMASS") {
      Vector3 loc;
      real_type mpmass;
      datastr >> mpmass >> loc(1) >> loc(2) >> loc(3);
      masses.push_back(masspoint(loc, mpmass));
    }
  }

  // In contrast to JSBSim, we have the possibility to simulate around a point
  // not being the center of gravity, use that here ...
  Vector3 cg = mBodyReference;
  if (haveVrp)
    mBodyReference = vrp;
  if (!haveEp)
    ep = cg;

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
  mVehicle->getTopBody()->addMultiBodyModel(new Mass(spi));

  // Attach the eye point.
  FreeFrame* epFrame = new FreeFrame("Eyepoint Frame");
  epFrame->setPosition(structToBody(ep));
  epFrame->setRelVel(Vector6::zeros());
  epFrame->setRelAccel(Vector6::zeros());
  Sensor* accelSensor = new Sensor("Acceleration Sensor");
  accelSensor->addSampleTime(SampleTime(1.0/120));
  Port* port = accelSensor->getOutputPort("nz");
  registerJSBExpression("accelerations/n-pilot-z-norm", port);
//   epFrame->addMultiBodyModel(accelSensor);
  mVehicle->getTopBody()->addMultiBodyModel(accelSensor);
  mVehicle->getTopBody()->addChildFrame(epFrame);
  addOutputModel(port, "Normalized load value", "/accelerations/nlf");

  // Set the position of the aerodynamic force frame.
  mAeroForce->setPosition(structToBody(ap));

  return true;
}

void
LegacyJSBSimReader::attachWheel(const std::string& name, const Vector3& pos,
                                const std::string& brake,
                                const std::string& numStr, real_type wheelDiam,
                                real_type tireSpring, real_type tireDamp,
                                RigidBody* parent)
{
  RigidBody* wheel = new RigidBody(name + " Wheel");
  InertiaMatrix wheelInertia(10, 0, 0, 100, 0, 10);
  wheel->addMultiBodyModel(new Mass(SpatialInertia(wheelInertia, 50)));
  parent->addChildFrame(wheel);
  
  RevoluteJoint* wj = new RevoluteJoint(name + " Wheel Joint");
  parent->addMultiBodyModel(wj, 0);
  wheel->addMultiBodyModel(wj, 1);
  wj->setJointAxis(Vector3(0, 1, 0));
  wj->setPosition(pos);
  wj->setOrientation(Quaternion::unit());
  wj->setJointPos(0);
  wj->setJointVel(0);

  // Add an brake force
  DiscBrake* brakeF = new DiscBrake(name + " Brake Force");
  brakeF->setFrictionConstant(-1e4);
  if (brake == "LEFT") {
    Port* port = lookupJSBExpression("gear/left-brake-pos-norm");
    brakeF->getInputPort(0)->connect(port);
  } else if (brake == "RIGHT") {
    Port* port = lookupJSBExpression("gear/right-brake-pos-norm");
    brakeF->getInputPort(0)->connect(port);
  }
  wj->setLineForce(brakeF);
  
  WheelContact* wc = new WheelContact(name + " Wheel Contact",
                                      mVehicle->getEnvironment());
  wc->setWheelRadius(0.5*wheelDiam);
  wc->setSpringConstant(convertFrom(uPoundForcePFt, tireSpring));
  wc->setSpringDamping(convertFrom(uPoundForcePFt, tireDamp));
  wc->setFrictionCoeficient(0.9);
  wheel->addMultiBodyModel(wc);
  
  Port* port = wj->getOutputPort(0);
  addOutputModel(port, "Wheel " + numStr + " Position",
                 "gear/gear[" + numStr + "]/wheel-position-rad");
  SiToUnitExpressionImpl* c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(port->getProperty());
  port = new Port;
  port->setProperty(Property(c));
  addOutputModel(port, "Wheel " + numStr + " Position Deg",
                 "gear/gear[" + numStr + "]/wheel-position-deg");
}

bool
LegacyJSBSimReader::convertUndercarriage(const std::string& data)
{
  // Undercarriage parsing.
  std::stringstream datastr(data);
  unsigned gearNumber = 0;

  while (datastr) {
    std::string uctype;
    datastr >> uctype;
    
    std::stringstream sstr;
    sstr << gearNumber;
    std::string numStr = sstr.str();
    // Increment the gear number
    ++gearNumber;

    if (uctype == "AC_GEAR") {
      std::string name, type, brake, retract;
      real_type x, y, z, k, d, fs, fd, rr, sa;
      datastr >> name >> x >> y >> z >> k >> d >> fs >> fd >> rr
              >> type >> brake >> sa >> retract;

      if (type == "CASTERING") {
        // Modelling castering gars as simple contacs without a special
        // direction
        SimpleContact* sc = new SimpleContact(name,
                                              mVehicle->getEnvironment());
        sc->setPosition(structToBody(Vector3(x, y, z)));

        sc->setSpringConstant(convertFrom(uPoundForcePFt, k));
        // FIXME: conversion factor:
        // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
        // Note that friction coefficients are different from that but
        // viscosous friction is just used in that way ...
        sc->setSpringDamping(convertFrom(uPoundForcePFt, d));
        sc->setFrictionCoeficient(0.1*fs);
        
        mVehicle->getTopBody()->addMultiBodyModel(sc);

      } else {
        // For jsbsim use simple gears
        SimpleGear* sg = new SimpleGear(name, mVehicle->getEnvironment());
        sg->setPosition(structToBody(Vector3(x, y, z)));
        
        sg->setSpringConstant(convertFrom(uPoundForcePFt, k));
        // FIXME: conversion factor:
        // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
        // Note that friction coefficients are different from that but
        // viscosous friction is just used in that way ...
        sg->setSpringDamping(convertFrom(uPoundForcePFt, d));
        sg->setFrictionCoeficient(fs);
        
        // Connect apprioriate input and output models

        // FIXME
        // missing output properties are "wow" and "tire-pressure-norm"

        if (retract == "RETRACT") {
          Port* port = lookupJSBExpression("gear/gear-pos-norm");
          sg->getInputPort("enabled")->connect(port);
          // Well, connect that directly to the input
          addOutputModel(port, "Gear " + numStr + " Position",
                         "/gear/gear[" + numStr + "]/position-norm");
        }

        if (type == "STEERABLE") {
          // FIXME: FCS might later define something for that gain ...
//           prop = lookupJSBExpression("fcs/steer-pos-deg[" + numStr + "]");
          Port* port = lookupJSBExpression("fcs/steer-cmd-norm");
          Gain* gain = new Gain(name + " Steer Gain");
          gain->setGain(sa);
          gain->getInputPort(0)->connect(port);
          addFCSModel(gain);
          addOutputModel(port, "Gear " + numStr + " Steering Output",
                         "/gear/gear[" + numStr + "]/steering-norm");

          UnaryFunctionModel *unary
            = new UnaryFunctionModel(name + " Degree Conversion",
                                     new UnitToSiExpressionImpl(uDegree));
          unary->getInputPort(0)->connect(gain->getOutputPort(0));
          addFCSModel(unary);

          sg->getInputPort("steeringAngle")->connect(unary->getOutputPort(0));
        }
        
        if (brake == "LEFT") {
          Port* port = lookupJSBExpression("gear/left-brake-pos-norm");
          sg->getInputPort("brakeCommand")->connect(port);
        } else if (brake == "RIGHT") {
          Port* port = lookupJSBExpression("gear/right-brake-pos-norm");
          sg->getInputPort("brakeCommand")->connect(port);
        }
        
        mVehicle->getTopBody()->addMultiBodyModel(sg);
      }
      
    } else if (uctype == "AC_LAUNCHBAR") {
      std::string d;
      datastr >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d;

    } else if (uctype == "AC_HOOK") {
      std::string d;
      datastr >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d;

    } else if (uctype == "AC_F18MLG") { 
      /// Well, that here is exactly how it should not be,
      /// but for initial testing of a new unfinished fdm ...
      std::string name, brake;
      Vector3 compressJointPos;
      real_type pullPress;
      real_type pushPress;
      real_type area;
      real_type minCompr;
      real_type maxCompr;
      real_type minDamp;
      real_type maxDamp;
      real_type armLength;
      real_type wheelDiam;
      real_type tireSpring, tireDamp;


      datastr >> name >> brake
              >> compressJointPos(1)
              >> compressJointPos(2)
              >> compressJointPos(3)
              >> pullPress >> pushPress
              >> area
              >> minCompr
              >> maxCompr
              >> minDamp
              >> maxDamp
              >> armLength
              >> wheelDiam
              >> tireSpring >> tireDamp;

      // Well this is come hardcoding, but as a demo built from within the
      // legacy JSBSim format this is ok :)

      // This is the movable part of the strut, doing the compression
      RigidBody* arm = new RigidBody(name + " Arm");
      mVehicle->getTopBody()->addChildFrame(arm);
      arm->addMultiBodyModel(new Mass(inertiaFrom(Vector3(-1, 0, 0), SpatialInertia(200))));

      // Connect that with a revolute joint to the main body
      RevoluteJoint* rj = new RevoluteJoint(name + " Arm Joint");
      mVehicle->getTopBody()->addMultiBodyModel(rj, 0);
      arm->addMultiBodyModel(rj, 1);
      rj->setJointAxis(Vector3(0, 1, 0));
      rj->setJointPos(0);
      rj->setJointVel(0);
      rj->setPosition(structToBody(compressJointPos));
      rj->setOrientation(Quaternion::unit());

      // Well, we use an air spring for that. It is directly in the
      // revolute joint. That is wring, but at the moment aprioriate.
      AirSpring* aoDamp = new AirSpring(name + " Air Spring Force");
      aoDamp->setPullPressure(pullPress);
      aoDamp->setPushPressure(pushPress);
      aoDamp->setArea(area);
      aoDamp->setMinCompression(minCompr);
      aoDamp->setMaxCompression(maxCompr);
      aoDamp->setMinDamperConstant(minDamp);
      aoDamp->setMaxDamperConstant(maxDamp);
      rj->setLineForce(aoDamp);

      // Attach a wheel to that strut part.
      attachWheel(name, Vector3(-armLength, 0, 0), brake, numStr, wheelDiam,
                  tireSpring, tireDamp, arm);

      Port* port = rj->getOutputPort(0);
      addOutputModel(port, "Gear " + numStr + " Compression",
                     "/gear/gear[" + numStr + "]/compression-rad");

      port = lookupJSBExpression("gear/gear-pos-norm");
      addOutputModel(port, "Gear " + numStr + " Position",
                     "/gear/gear[" + numStr + "]/position-norm");

    } else if (uctype == "AC_CLG") {
      std::string name, brake, steer;
      Vector3 compressJointPos;
      real_type pullPress;
      real_type pushPress;
      real_type area;
      real_type minCompr;
      real_type maxCompr;
      real_type minDamp;
      real_type maxDamp;
      real_type wheelDiam;
      real_type tireSpring, tireDamp;

      datastr >> name >> brake
              >> compressJointPos(1)
              >> compressJointPos(2)
              >> compressJointPos(3)
              >> pullPress >> pushPress
              >> area
              >> minCompr
              >> maxCompr
              >> minDamp
              >> maxDamp
              >> wheelDiam
              >> tireSpring >> tireDamp
              >> steer;

      // Well this is come hardcoding, but as a demo built from within the
      // legacy JSBSim format this is ok :)

      // Model steering here ...
      // normally we connect the compressible part to the top level body, but
      // in case of steering this is no longer true.
      RigidBody* strutParent = mVehicle->getTopBody();
      if (steer == "STEERABLE") {
        // A new part modelling the steering
        RigidBody* steer = new RigidBody(name + " Steer");
        strutParent->addChildFrame(steer);

        // connect that via a revolute joint to the toplevel body.
        // Note the 0.05m below, most steering wheels have some kind of
        // castering auto line up behavour. That is doe with this 0.05m.
        RevoluteJoint* sj = new RevoluteJoint(name + " Steer Joint");
        strutParent->addMultiBodyModel(sj, 0);
        steer->addMultiBodyModel(sj, 1);
        sj->setJointAxis(Vector3(0, 0, 1));
        sj->setJointPos(0);
        sj->setJointVel(0);
        sj->setPosition(structToBody(compressJointPos)
                        + Vector3(0.05, 0, 0));
        sj->setOrientation(Quaternion::unit());

        // Add an actuator trying to interpret the steering command
        LineActuator* steerAct = new LineActuator(name + " Steering Actuator");
        Port* port = lookupJSBExpression("fcs/steer-cmd-norm");
        steerAct->getInputPort(0)->connect(port);
        steerAct->setProportionalGain(-1e6);
        steerAct->setDerivativeGain(-1e3);
        sj->setLineForce(steerAct);
        
        strutParent = steer;
        
        // Prepare outputs
        port = sj->getOutputPort(0);
        addOutputModel(port, "Steering " + numStr + " Position",
                       "/gear/gear[" + numStr + "]/steering-pos-rad");
        SiToUnitExpressionImpl* c = new SiToUnitExpressionImpl(uDegree);
        c->setInputProperty(port->getProperty());
        port = new Port(); // FIXME add unit convert model
        port->setProperty(Property(c));
        addOutputModel(port, "Steering " + numStr + " Position Deg",
                       "/gear/gear[" + numStr + "]/steering-pos-deg");
      }


      // Now the compressible part of the strut
      RigidBody* arm = new RigidBody(name + " Strut");
      strutParent->addChildFrame(arm);
      arm->addMultiBodyModel(new Mass(inertiaFrom(Vector3(0, 0, 1), SpatialInertia(200))));

      // This time it is a prismatic joint
      PrismaticJoint* pj = new PrismaticJoint(name + " Compress Joint");
      strutParent->addMultiBodyModel(pj, 0);
      arm->addMultiBodyModel(pj, 1);
      pj->setJointAxis(Vector3(0, 0, -1));
      if (strutParent == mVehicle->getTopBody())
        pj->setPosition(structToBody(compressJointPos));
      else
        pj->setPosition(Vector3(-0.05, 0, 0));

      // With an air spring
      AirSpring* aoDamp = new AirSpring(name + " Air Spring Force");
      aoDamp->setPullPressure(pullPress);
      aoDamp->setPushPressure(pushPress);
      aoDamp->setArea(area);
      aoDamp->setMinCompression(minCompr);
      aoDamp->setMaxCompression(maxCompr);
      aoDamp->setMinDamperConstant(minDamp);
      aoDamp->setMaxDamperConstant(maxDamp);
      pj->setLineForce(aoDamp);

      // Attach a wheel to that strut part.
      attachWheel(name, Vector3::zeros(), brake, numStr, wheelDiam,
                  tireSpring, tireDamp, arm);

      // Prepare some outputs ...
      Port* port = pj->getOutputPort(0);
      addOutputModel(port, "Gear " + numStr + " Compression",
                     "/gear/gear[" + numStr + "]/compression-m");

      port = lookupJSBExpression("gear/gear-pos-norm");
      addOutputModel(port, "Gear " + numStr + " Position",
                     "/gear/gear[" + numStr + "]/position-norm");

    } else if (uctype == "AC_CONTACT") {
      std::string name, type, brake, retract;
      real_type x, y, z, k, d, fs, fd, rr, sa;
      datastr >> name >> x >> y >> z >> k >> d >> fs >> fd >> rr
              >> type >> brake >> sa >> retract;

      // Very simple contact force. Penalty method.
      SimpleContact* sc = new SimpleContact(name, mVehicle->getEnvironment());
      sc->setPosition(structToBody(Vector3(x, y, z)));

      sc->setSpringConstant(convertFrom(uPoundForcePFt, k));
      // FIXME: conversion factor:
      // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
      // Note that friction coefficients are different from that but
      // viscosous friction is just used in that way ...
      sc->setSpringDamping(convertFrom(uPoundForcePFt, d));
      sc->setFrictionCoeficient(fs);

      mVehicle->getTopBody()->addMultiBodyModel(sc);
    }
  }

  return true;
}

bool
LegacyJSBSimReader::convertFCSList(const XMLElement* fcsElem)
{
  // JSBSim has a very strange way to define default input values for FCS
  // components. The output of the prevous component is the default input
  // value. This one is stored here.
  mPrevousFCSOutput = 0;
  
  std::list<shared_ptr<const XMLElement> > comps
    = fcsElem->getElements("COMPONENT");
  std::list<shared_ptr<const XMLElement> >::const_iterator it;
  for (it = comps.begin(); it != comps.end(); ++it) {
    if (!convertFCSComponent((*it)->getAttribute("TYPE"),
                             (*it)->getAttribute("NAME"),
                             (*it)->getData()))
      error("Cannot convert FCS component \""
            + (*it)->getAttribute("NAME") + "\"");
  }

  // Make shure it is not set when we exit here.
  mPrevousFCSOutput = 0;

  return true;
}

bool
LegacyJSBSimReader::convertFCSComponent(const std::string& type,
                                        const std::string& name,
                                        const std::string& data)
{
  std::stringstream datastr(data);

  // The model we put into the fcs group in the end ...
  shared_ptr<Model> model;
  shared_ptr<DeadBand> deadband;
  shared_ptr<Summer> summer;
  shared_ptr<Gain> gain;
  shared_ptr<DiscreteTransferFunction> discreteTransfFunc;
  shared_ptr<Table1D> table1D;
  shared_ptr<Saturation> kinematRateLimit;
  shared_ptr<Saturation> inputSaturation;

  // The final output property.
  shared_ptr<Port> out;
  shared_ptr<Port> normOut;

  // JSBSim FCS output values contain some implicit rules.
  // From the component name a default output property is formed.
  // If we find an OUTPUT line this is the output value too.
  // So collect them first and register them later when the final output
  // expression is known.
  std::list<std::string> outlist;
  outlist.push_back(string("fcs/") + normalizeComponentName(name));

  if (type == "SUMMER") {
    summer = new Summer(name);
    summer->setNumSummands(0);
    model = summer;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "DEADBAND") {
    deadband = new DeadBand(name);
    model = deadband;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "GRADIENT") {
    model = new TimeDerivative(name);
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "SWITCH") {
    std::cout << "Ignoring SWITCH" << std::endl;

  } else if (type == "KINEMAT") {
    // A KINEMAT is done as a first order ODE packed into a discrete system
    // The derivative is limited to match the avarage movement speed of the
    // KINEMAT. This is not exactly like JSBSim does that, but it is
    // sufficient for now.
    gain = new Gain(name + " Input Gain");
    addFCSModel(gain);
    gain->setGain(1);
    model = gain;

    inputSaturation = new Saturation(name + " Input Saturation");
    addFCSModel(inputSaturation);
    inputSaturation->getInputPort(0)->connect(gain->getOutputPort(0));

    Summer* inputError = new Summer(name + " Input Sum");
    addFCSModel(inputError);
    inputError->getInputPort(0)->connect(inputSaturation->getOutputPort(0));
    inputError->setNumSummands(2);

    Gain* errorGain = new Gain(name + " Error Gain");
    addFCSModel(errorGain);
    errorGain->setGain(100);
    errorGain->getInputPort(0)->connect(inputError->getOutputPort(0));

    kinematRateLimit = new Saturation(name + " Rate Limit");
    addFCSModel(kinematRateLimit);
    kinematRateLimit->getInputPort(0)->connect(errorGain->getOutputPort(0));

    DiscreteIntegrator* integrator
      = new DiscreteIntegrator(name + " Integrator");
    addFCSModel(integrator);
    integrator->getInputPort(0)->connect(kinematRateLimit->getOutputPort(0));
    Matrix tmp(1, 1);
    tmp(1, 1) = 1;
//     tmp.clear();
    integrator->setInitialValue(tmp);
    out = integrator->getOutputPort(0);

    Gain* feedbackGain = new Gain(name + " Feedback Gain");
    addFCSModel(feedbackGain);
    feedbackGain->setGain(-1);
    feedbackGain->getInputPort(0)->connect(integrator->getOutputPort(0));
    inputError->getInputPort(1)->connect(feedbackGain->getOutputPort(0));

  } else if (type == "PURE_GAIN") {
    gain = new Gain(name);
    gain->setGain(1);
    model = gain;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "AEROSURFACE_SCALE") {
    // An AEROSURFACE_SCALE component is done with n input saturation clipping
    // the input from -1 to 1. This is the one which is magically mapped to
    // an output property in /surface-positions/...
    // This one's output is directly connected to a lookup table

    inputSaturation = new Saturation(name + "Input Saturation");
    model = inputSaturation;
    addFCSModel(inputSaturation);
    normOut = inputSaturation->getOutputPort(0);
    Matrix tmp(1, 1);
    tmp(1, 1) = -1;
    inputSaturation->setMinSaturation(tmp);
    tmp(1, 1) = 1;
    inputSaturation->setMaxSaturation(tmp);

    table1D = new Table1D(name);
    TableLookup tl;
    tl.setAtIndex(1, -1);
    tl.setAtIndex(2, 0);
    tl.setAtIndex(3, 1);
    table1D->setTableLookup(tl);
    TableData<1>::SizeVector sv;
    sv(1) = 3;
    TableData<1> tableData(sv);
    TableData<1>::Index iv;
    iv(1) = 1;
    tableData(iv) = -1;
    iv(1) = 2;
    tableData(iv) = 0;
    iv(1) = 3;
    tableData(iv) = 1;
    table1D->setTableData(tableData);
    table1D->getInputPort(0)->connect(inputSaturation->getOutputPort(0));

    addFCSModel(table1D);
    out = table1D->getOutputPort(0);

  } else if (type == "SCHEDULED_GAIN") {
    Product* prod = new Product(name);
    prod->setNumFactors(2);
    table1D = new Table1D(std::string("Lookup table for ") + name);
    addFCSModel(table1D);
    prod->getInputPort(1)->connect(table1D->getOutputPort(0));
    model = prod;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "INTEGRATOR") {
    model = new DiscreteIntegrator(name);
    out = model->getOutputPort(0);
    addFCSModel(model);

  } else if (type == "LAG_FILTER") {
    //   C1
    // ------
    // s + C1
    discreteTransfFunc = new DiscreteTransferFunction(name);
    Vector v(1);
    discreteTransfFunc->setNumerator(v);
    v.resize(2);
    v(1) = 1;
    discreteTransfFunc->setDenominator(v);
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "LEAD_LAG_FILTER") {
    // C1*s + C2
    // ---------
    // C3*s + C4
    discreteTransfFunc = new DiscreteTransferFunction(name);
    discreteTransfFunc->setNumerator(Vector(2));
    discreteTransfFunc->setDenominator(Vector(2));
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "WASHOUT_FILTER") {
    //   s
    // ------
    // s + C1
    discreteTransfFunc = new DiscreteTransferFunction(name);
    Vector v(2);
    v(1) = 1;
    v(2) = 0;
    discreteTransfFunc->setNumerator(v);
    discreteTransfFunc->setDenominator(v);
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (type == "SECOND_ORDER_FILTER") {
    // C1*s + C2*s + C3
    // ----------------
    // C4*s + C5*s + C6
    discreteTransfFunc = new DiscreteTransferFunction(name);
    discreteTransfFunc->setNumerator(Vector(3));
    discreteTransfFunc->setDenominator(Vector(3));
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else
    return error("Unknown FCS COMPONENT type: \"" + type
                 + "\". Ignoring whole FCS component \"" + name + "\"" );

  OpenFDMAssert(out->isConnected());

  // The output expression from the prevous FCS block.
  // This is the default input value for this FCS block.
  std::list<shared_ptr<Port> > inputs;

  // Collect any expressions for the output chain here.
  shared_ptr<Saturation> saturation;
  shared_ptr<Bias> outbias;
  shared_ptr<Gain> outgain;
  bool outInvert = false;
  bool noScale = false;

  for (;;) {
    std::string token;
    datastr >> token;
    if (!datastr)
      break;

    if (token == "BIAS") {
      real_type value;
      datastr >> value;

      std::string modelName = std::string("Output Bias for ") + name;
      outbias = new Bias(modelName);
      addFCSModel(outbias);

      Matrix tmp(1, 1);
      tmp(1, 1) = value;
      outbias->setBias(tmp);
      
    } else if (token == "CLIPTO") {
      real_type clipmin, clipmax;
      datastr >> clipmin >> clipmax;
      
      if (!saturation) {
        std::string modelName = std::string(name) + " OSat";
        saturation = new Saturation(modelName);
        addFCSModel(saturation);
      }

      Matrix tmp(1, 1);
      tmp(1, 1) = clipmin;
      saturation->setMinSaturation(tmp);

      tmp(1, 1) = clipmax;
      saturation->setMaxSaturation(tmp);
      
    } else if (token == "C1") {
      real_type value;
      datastr >> value;

      if (type == "INTEGRATOR") {
        std::string modelName = std::string("Output Gain for ") + name;
        outgain = new Gain(modelName);
        addFCSModel(outgain);
        outgain->setGain(value);
      } else if (type == "LAG_FILTER") {
        //   C1
        // ------
        // s + C1
        Vector v = discreteTransfFunc->getNumerator();
        v(2) = value;
        discreteTransfFunc->setNumerator(v);
        v = discreteTransfFunc->getDenominator();
        v(2) = value;
        discreteTransfFunc->setDenominator(v);
        
      } else if (type == "LEAD_LAG_FILTER") {
        // C1*s + C2
        // ---------
        // C3*s + C4
        Vector v = discreteTransfFunc->getNumerator();
        v(1) = value;
        discreteTransfFunc->setNumerator(v);
        
      } else if (type == "WASHOUT_FILTER") {
        //   s
        // ------
        // s + C1
        Vector v = discreteTransfFunc->getDenominator();
        v(2) = value;
        discreteTransfFunc->setDenominator(v);
        
      } else if (type == "SECOND_ORDER_FILTER") {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getNumerator();
        v(1) = value;
        discreteTransfFunc->setNumerator(v);
      } else
        return error("No C1 parameter allowed for \"" + type + "\"");
      
    } else if (token == "C2") {
      real_type value;
      datastr >> value;

      if (type == "LEAD_LAG_FILTER") {
        // C1*s + C2
        // ---------
        // C3*s + C4
        Vector v = discreteTransfFunc->getNumerator();
        v(2) = value;
        discreteTransfFunc->setNumerator(v);
        
      } else if (type == "SECOND_ORDER_FILTER") {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getNumerator();
        v(2) = value;
        discreteTransfFunc->setNumerator(v);
      } else
        return error("No C2 parameter allowed for \"" + type + "\"");
      
    } else if (token == "C3") {
      real_type value;
      datastr >> value;

      if (type == "LEAD_LAG_FILTER") {
        // C1*s + C2
        // ---------
        // C3*s + C4
        Vector v = discreteTransfFunc->getDenominator();
        v(1) = value;
        discreteTransfFunc->setDenominator(v);
        
      } else if (type == "SECOND_ORDER_FILTER") {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getNumerator();
        v(3) = value;
        discreteTransfFunc->setNumerator(v);

      } else
        return error("No C3 parameter allowed for \"" + type + "\"");
      
    } else if (token == "C4") {
      real_type value;
      datastr >> value;

      if (type == "LEAD_LAG_FILTER") {
        // C1*s + C2
        // ---------
        // C3*s + C4
        Vector v = discreteTransfFunc->getDenominator();
        v(2) = value;
        discreteTransfFunc->setDenominator(v);
        
      } else if (type == "SECOND_ORDER_FILTER") {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getDenominator();
        v(1) = value;
        discreteTransfFunc->setDenominator(v);

      } else
        return error("No C4 parameter allowed for \"" + type + "\"");
      
    } else if (token == "C5") {
      real_type value;
      datastr >> value;

      if (type == "SECOND_ORDER_FILTER") {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getDenominator();
        v(2) = value;
        discreteTransfFunc->setDenominator(v);

      } else
        return error("No C5 parameter allowed for \"" + type + "\"");
      
    } else if (token == "C6") {
      real_type value;
      datastr >> value;

      if (type == "SECOND_ORDER_FILTER") {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getDenominator();
        v(3) = value;
        discreteTransfFunc->setDenominator(v);

      } else
        return error("No C6 parameter allowed for \"" + type + "\"");
      
    } else if (token == "DETENTS") {
      int detents;
      datastr >> detents;

      if (type == "KINEMAT") {
        real_type minVal = Limits<real_type>::max();
        real_type maxVal = -Limits<real_type>::max();
        real_type allTime = 0;
        for (unsigned i = 0; datastr && i < detents; ++i) {
          real_type val, time;
          datastr >> val >> time;
          // Ignore the first time entry ...
          if (i)
            allTime += time;
          minVal = min(minVal, val);
          maxVal = max(maxVal, val);
        }
        Matrix tmp(1, 1);
        if (allTime != 0) {
          real_type avgTransRate = abs((maxVal-minVal)/allTime);
          tmp(1, 1) = -avgTransRate;
          kinematRateLimit->setMinSaturation(tmp);
          tmp(1, 1) = avgTransRate;
          kinematRateLimit->setMaxSaturation(tmp);
        }
        tmp(1, 1) = minVal;
        inputSaturation->setMinSaturation(tmp);
        tmp(1, 1) = maxVal;
        inputSaturation->setMaxSaturation(tmp);

        gain->setGain(maxVal);
      } else
        return error("No DETENTS parameter allowed for \"" + type + "\"");

    } else if (token == "GAIN") {
      real_type value;
      datastr >> value;

      if (gain) {
        gain->setGain(value);
      } else {
        std::string modelName = std::string(name) + " OGain";
        outgain = new Gain(modelName);
        addFCSModel(outgain);
        outgain->setGain(value);
      }
      
    } else if (token == "INPUT") {
      datastr >> token;
      inputs.push_back(lookupJSBExpression(token));
      
    } else if (token == "INVERT") {
      // Append a minus expression to the output chain.
      outInvert = true;
      
    } else if (token == "MAX") {
      real_type clipmax;
      datastr >> clipmax;
      
      if (type == "AEROSURFACE_SCALE") {
        TableData<1> tableData = table1D->getTableData();
        TableData<1>::Index iv;
        iv(1) = 3;
        tableData(iv) = clipmax;
        table1D->setTableData(tableData);
      } else {
        if (!saturation) {
          std::string modelName = std::string(name) + " OSat";
          saturation = new Saturation(modelName);
          addFCSModel(saturation);
        }
        
        Matrix tmp(1, 1);
        tmp(1, 1) = clipmax;
        saturation->setMaxSaturation(tmp);
      }
      
    } else if (token == "MIN") {
      real_type clipmin;
      datastr >> clipmin;
      
      if (type == "AEROSURFACE_SCALE") {
        TableData<1> tableData = table1D->getTableData();
        TableData<1>::Index iv;
        iv(1) = 1;
        tableData(iv) = clipmin;
        table1D->setTableData(tableData);
      } else {
        if (!saturation) {
          std::string modelName = std::string(name) + " OSat";
          saturation = new Saturation(modelName);
          addFCSModel(saturation);
        }
        
        Matrix tmp(1, 1);
        tmp(1, 1) = clipmin;
        saturation->setMinSaturation(tmp);
      }

    } else if (token == "NOSCALE") {

      if (type == "KINEMAT") {
        noScale = true;
      } else
        return error("No NOSCALE parameter allowed for \"" + type + "\"");
      
    } else if (token == "OUTPUT") {
      datastr >> token;
      // Make sure that the token is not already in the outlist ... FIXME
      outlist.remove(token);
      // Add it
      outlist.push_back(token);

    } else if (token == "ROWS") {
      int rows;
      datastr >> rows;
      
      TableData<1>::SizeVector sz;
      sz(1) = rows;
      TableData<1> tableData(sz);
      TableLookup lookup;
      if (!parseTable1D(datastr, tableData, lookup))
        return error("Cannot parse lookup table for \"" + type + "\"");

      if (table1D) {
        table1D->setTableData(tableData);
        table1D->setTableLookup(lookup);
      }

    } else if (token == "SCHEDULED_BY") {
      datastr >> token;
      
      if (table1D) {
        table1D->getInputPort(0)->connect(lookupJSBExpression(token));
      } else
        return error("SCHEDULED_BY without table ??");
      
    } else if (token == "WIDTH") {
      // deadband width
      real_type width;
      datastr >> width;

      if (type == "DEADBAND") {
        deadband->setWidth(width);
      } else
        return error("No WIDTH parameter allowed for \"" + type + "\"");

    } else
      return error("Unknown FCS COMPONENT keyword: \"" + token
                   + "\". Ignoring whole FCS component \"" + name + "\"");
  }

  // If no explicit input is given, use that braindead default of the
  // prevous block
  if (inputs.empty())
    inputs.push_back(mPrevousFCSOutput);

  unsigned idx = 0;
  for (std::list<shared_ptr<Port> >::iterator it = inputs.begin();
       it != inputs.end(); ++it) {
    // FIXME!!!!!!
    if (summer && summer->getNumInputPorts() <= idx)
      summer->setNumSummands(idx+1);
    model->getInputPort(idx++)->connect(*it);
  }

  // Sanity check, if the first FCS component does not have an input defined.
  if (!inputs.front()->isConnected()) {
    return error("No input value defined for FCS COMPONENT. "
                 "Ignoring whole FCS component \"" + name + "\"");
  }

  // Now put the expressions for the output chain together.
  // Doing that now will put them together in a well defined order.
  if (outbias) {
    outbias->getInputPort(0)->connect(out);
    out = outbias->getOutputPort(0);
  }
  if (outgain) {
    outgain->getInputPort(0)->connect(out);
    out = outgain->getOutputPort(0);
  }
  if (saturation) {
    saturation->getInputPort(0)->connect(out);
    out = saturation->getOutputPort(0);
  }
  if (outInvert) {
    out = addInverterModel(name, out);
  }
  // FIXME put in here a normalized out property, or at least a gain to
  // normalize

  // For KINEMATS ...
  if (type == "KINEMAT") {
    if (noScale) {
      gain->setGain(1);
      normOut = out;
    } else {
      Gain* normGain = new Gain(name + " Normalize Gain");
      normGain->setGain(1/gain->getGain());
      addFCSModel(normGain);
      normGain->getInputPort(0)->connect(out);
      normOut = normGain->getOutputPort(0);
    }
  }

  if (!normOut || !normOut->isConnected())
    normOut = inputs.front();

  // Register all output property names.
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

  // Finally set the current components output to the default input for the
  // next block.
  mPrevousFCSOutput = out;

  return true;
}

bool
LegacyJSBSimReader::convertPropulsion(const XMLElement* pElem)
{
  unsigned engineNumber = 0;
  unsigned tankNumber = 0;

  std::list<shared_ptr<XMLElement> > elems = pElem->getElements();
  std::list<shared_ptr<XMLElement> >::const_iterator it;
  for (it = elems.begin(); it != elems.end(); ++it) {
    if ((*it)->getName() == "AC_ENGINE") {
//       std::string engineFile = mEnginePath+(*it)->getAttribute("FILE")+".xml";
//       std::ifstream infile;
//       infile.open(engineFile.c_str());
//       if (!infile.is_open())
//         return;

//       XMLDomParser parser;
//       if (!parser.parseXML(infile))
//         return;
//       infile.close();

      std::stringstream sstr;
      sstr << engineNumber;
      ++engineNumber;
      if (!convertEngine((*it)->getData(), (*it)->getName(), sstr.str()))
        return error("Cannot parse engine");
    }
    else if ((*it)->getName() == "AC_TANK") {
      ++tankNumber;
    }
    else
      return error("Unexpected PROPULSION element \"" + (*it)->getName() + "\"");
  }

  return true;
}

bool
LegacyJSBSimReader::convertTank(const std::string& data,
                                const std::string& type,
                                const std::string& number)
{
  std::stringstream datastr(data);

  Vector3 loc = Vector3::zeros();
  real_type radius = 0;
  real_type capacity = 0;
  real_type contents = 0;
  real_type temperature = 0;

  for (;;) {
    std::string token;
    datastr >> token;
    if (!datastr)
      break;

    if (token == "XLOC") {
      datastr >> loc(1);
      
    } else if (token == "YLOC") {
      datastr >> loc(2);
      
    } else if (token == "ZLOC") {
      datastr >> loc(3);
      
    } else if (token == "RADIUS") {
      datastr >> radius;
      
    } else if (token == "CAPACITY") {
      datastr >> capacity;
      
    } else if (token == "CONTENTS") {
      datastr >> contents;
      
    } else if (token == "TEMPERATURE") {
      datastr >> temperature;
      
    } else
      return error("Unknown parameter in FG_ENGINE configuration");
  }
  return true;
}

bool
LegacyJSBSimReader::convertThruster(const std::string& data,
                                    const std::string& type,
                                    const std::string& number)
{
  std::stringstream datastr(data);

  Vector3 loc = Vector3::zeros();
  real_type pitch = 0;
  real_type yaw = 0;

  for (;;) {
    std::string token;
    datastr >> token;
    if (!datastr)
      break;

    if (token == "XLOC") {
      datastr >> loc(1);
      
    } else if (token == "YLOC") {
      datastr >> loc(2);
      
    } else if (token == "ZLOC") {
      datastr >> loc(3);
      
    } else if (token == "PITCH") {
      datastr >> pitch;
      
    } else if (token == "YAW") {
      datastr >> yaw;
      
    } else if (token == "P_FACTOR") {
      real_type d;
      datastr >> d;
      
    } else if (token == "SENSE") {
      real_type d;
      datastr >> d;
      
    } else
      return error("Unknown parameter in FG_TANK configuration");
  }
  return true;
}

bool
LegacyJSBSimReader::convertEngine(const std::string& data,
                                  const std::string& type,
                                  const std::string& number)
{
  std::stringstream datastr(data);

  Vector3 loc = Vector3::zeros();
  real_type pitch = 0;
  real_type yaw = 0;

  for (;;) {
    std::string token;
    datastr >> token;
    if (!datastr)
      break;

    if (token == "XLOC") {
      datastr >> loc(1);
      
    } else if (token == "YLOC") {
      datastr >> loc(2);
      
    } else if (token == "ZLOC") {
      datastr >> loc(3);
      
    } else if (token == "PITCH") {
      datastr >> pitch;
      
    } else if (token == "YAW") {
      datastr >> yaw;
      
    } else if (token == "FEED") {
      real_type d;
      datastr >> d;
      
    } else
      return error("Unknown parameter in engine configuration");
  }

  std::string namestr = "Engine<" + number + ">";
  DirectForce* engineForce = new DirectForce(namestr);
  engineForce->setDirection(Vector6(0, 0, 0, 4.4*1.5e4, 0, 0));
  engineForce->setPosition(structToBody(loc));
  engineForce->setOrientation(Quaternion::fromHeadAttBank(pitch, 0, yaw));

  std::string throttlename = "fcs/throttle-cmd-norm[" + number + "]";
  engineForce->getInputPort(0)->connect(lookupJSBExpression(throttlename));

  mVehicle->getTopBody()->addMultiBodyModel(engineForce);

  return true;
}

bool
LegacyJSBSimReader::convertAerodynamics(const XMLElement* aerodynamics)
{
  std::list<shared_ptr<XMLElement> > elems = aerodynamics->getElements();
  std::list<shared_ptr<XMLElement> >::const_iterator it;
  for (it = elems.begin(); it != elems.end(); ++it) {
    std::string axisname = (*it)->getAttribute("NAME");

    shared_ptr<UnitToSiExpressionImpl> toNewton
      = new UnitToSiExpressionImpl(uPoundForce);
    shared_ptr<UnitToSiExpressionImpl> toNewtonMeter
      = new UnitToSiExpressionImpl(uPoundForceFt);
    shared_ptr<SumExpressionImpl> sum = new SumExpressionImpl;
    toNewtonMeter->setInputProperty(TypedProperty<real_type>(sum));
    toNewton->setInputProperty(TypedProperty<real_type>(sum));
    if (axisname == "LIFT") {
      shared_ptr<MinusExpressionImpl> minus = new MinusExpressionImpl;
      minus->setInputProperty(TypedProperty<real_type>(toNewton));
      mAeroForce->addStabilityAxisSummand(AeroForce::LiftAxis,
                                          TypedProperty<real_type>(minus));
    }
    else if (axisname == "DRAG") {
      shared_ptr<MinusExpressionImpl> minus = new MinusExpressionImpl;
      minus->setInputProperty(TypedProperty<real_type>(toNewton));
      mAeroForce->addStabilityAxisSummand(AeroForce::DragAxis,
                                          TypedProperty<real_type>(minus));
    }
    else if (axisname == "SIDE") {
      mAeroForce->addBodyAxisSummand(AeroForce::SideAxis,
                                     TypedProperty<real_type>(toNewton));
    } else if (axisname == "ROLL") {
      mAeroForce->addBodyAxisSummand(AeroForce::RollAxis,
                                     TypedProperty<real_type>(toNewtonMeter));
    } else if (axisname == "PITCH") {
      mAeroForce->addBodyAxisSummand(AeroForce::PitchAxis,
                                     TypedProperty<real_type>(toNewtonMeter));
    } else if (axisname == "YAW") {
      mAeroForce->addBodyAxisSummand(AeroForce::YawAxis,
                                     TypedProperty<real_type>(toNewtonMeter));
    } else
      return error("Unknown aerodynamic axis!");

    // Now parse the summands
    if (!convertAEROSummands(*it, sum, 0))
      return error("Cannot convert aerodynamic summands for axis \"" + 
                   axisname  + "\"");
  }

  return true;
}

bool
LegacyJSBSimReader::convertAEROSummands(const XMLElement* aeroSummands,
                                        SumExpressionImpl* sum,
                                        ProductExpressionImpl* prod)
{
  std::list<shared_ptr<XMLElement> > elems = aeroSummands->getElements();
  std::list<shared_ptr<XMLElement> >::const_iterator it;
  for (it = elems.begin(); it != elems.end(); ++it) {
    if ((*it)->getName() == "GROUP") {
     
      shared_ptr<ProductExpressionImpl> newProd = new ProductExpressionImpl;
      sum->addInputProperty(TypedProperty<real_type>(newProd));
      shared_ptr<SumExpressionImpl> newSum = new SumExpressionImpl;
      newProd->addInputProperty(TypedProperty<real_type>(newSum));

      if (!convertAEROSummands(*it, newSum, newProd))
        return error("Error parsing aerodynamic tables");
    }
    else if ((*it)->getName() == "FACTOR") {
      if (!prod)
        return error("Error parsing aerodynamic tables, FACTOR without GROUP");

      std::string type = (*it)->getAttribute("TYPE");
      TypedProperty<real_type> prop
        = convertCoefficient((*it)->getData(), type);
      prod->addInputProperty(prop);
    }
    else if ((*it)->getName() == "COEFFICIENT") {
      std::string type = (*it)->getAttribute("TYPE");
      TypedProperty<real_type> prop
        = convertCoefficient((*it)->getData(), type);
      sum->addInputProperty(prop);
    }
  }

  return true;
}

TypedProperty<real_type>
LegacyJSBSimReader::convertCoefficient(const std::string& data,
                                       const std::string& type)
{
  ProductExpressionImpl* prod = new ProductExpressionImpl;

  unsigned ndims;
  if (type == "VALUE") {
    ndims = 0;
  } else if (type == "VECTOR") {
    ndims = 1;
  } else if (type == "TABLE") {
    ndims = 2;
  } else if (type == "TABLE3D") {
    ndims = 3;
  } else {
    std::cerr << "Unknown TYPE attribute \"" << type
              << "\" for COEFFICIENT tag! Ignoring!" << std::endl;
    return TypedProperty<real_type>(prod);
  }

  std::stringstream datastr(data);

  std::string token;
  // The fist token is some useless string.
  datastr >> token;

  // The number of table entries 
  unsigned n[3] = { 0 };
  for (unsigned i = 0; i < ndims; ++i)
    datastr >> n[i];

  // The table lookup values.
  Property inVal[3];
  for (unsigned i = 0; i < ndims; ++i) {
    datastr >> token;
    inVal[i] = lookupJSBExpression(token)->getProperty();
  }

  // The other factors in this product.
  std::string line;
  std::getline(datastr, line, '\n');
  std::getline(datastr, line, '\n');

  std::string::size_type pos;
  while ((pos = line.find('|')) != std::string::npos)
    line.replace(pos, 1, " ");
  std::stringstream linestream(line);
  while (linestream >> token) {
    if (token.empty() || token == "none")
      break;
    prod->addInputProperty(lookupJSBExpression(token)->getProperty());
  }
 
  // The lookup table values.
  if (ndims == 0) {
    real_type value;
    datastr >> value;
    prod->addInputProperty(Property(new ConstExpressionPropertyImpl<real_type>(value)));
  }
  else if (ndims == 1) {
    TableData<1>::SizeVector size;
    size(1) = n[0];
    TableData<1> table(size);
    TableLookup lookup;
    if (!parseTable1D(datastr, table, lookup))
      // FIXME
      std::cerr << "Cannot parse " + type + " table" << std::endl;
    TableExpressionImpl<1>* ti = new TableExpressionImpl<1>();
    ti->setTable(table);
    ti->setTableLookup(0, lookup);
    ti->setInputProperty(0, inVal[0]);
    prod->addInputProperty(Property(ti));
  }
  else if (ndims == 2) {
    TableData<2>::SizeVector size;
    size(1) = n[0];
    size(2) = n[1];
    TableData<2> table(size);
    TableLookup lookup[2];
    if (!parseTable2D(datastr, table, lookup))
      // FIXME
      std::cerr << "Cannot parse " + type + " table" << data << std::endl;
    TableExpressionImpl<2>* ti = new TableExpressionImpl<2>();
    ti->setTable(table);
    for (unsigned i = 0; i < 2; ++i) {
      ti->setTableLookup(i, lookup[i]);
      ti->setInputProperty(i, inVal[i]);
    }
    prod->addInputProperty(Property(ti));
  }
  else if (ndims == 3) {
    TableData<3>::SizeVector size;
    size(1) = n[0];
    size(2) = n[1];
    size(3) = n[2];
    TableData<3> table(size);
    TableLookup lookup[3];
    if (!parseTable3D(datastr, table, lookup))
      // FIXME
      std::cerr << "Cannot parse " + type + " table" << std::endl;
    TableExpressionImpl<3>* ti = new TableExpressionImpl<3>();
    ti->setTable(table);
    for (unsigned i = 0; i < 3; ++i) {
      ti->setTableLookup(i, lookup[i]);
      ti->setInputProperty(i, inVal[i]);
    }
    prod->addInputProperty(Property(ti));
  }

  return TypedProperty<real_type>(prod);
}

void
LegacyJSBSimReader::makeAeroprops(void)
{
  Property e = mAeroForce->getProperty("trueSpeed");
  Port* port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/vt-mps", port);
  SiToUnitExpressionImpl* c = new SiToUnitExpressionImpl(uFeetPSecond);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/vt-fps", port);
  c = new SiToUnitExpressionImpl(uKnots);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/vt-kts", port);

  // Mach numbers, are unitless.
  e = mAeroForce->getProperty("machNumber");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/mach-norm", port);

  // Rotational rates wrt air.
  e = mAeroForce->getProperty("p");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/p-rad_sec", port);
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/p-aero-rad_sec", port);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/p-aero-deg_sec", port);
  e = mAeroForce->getProperty("q");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/q-rad_sec", port);
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/q-aero-rad_sec", port);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/q-aero-deg_sec", port);
  e = mAeroForce->getProperty("r");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/r-rad_sec", port);
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/r-aero-rad_sec", port);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/r-aero-deg_sec", port);


  e = mAeroForce->getProperty("u");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/u-aero-mps", port);
  c = new SiToUnitExpressionImpl(uFeetPSecond);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/u-aero-fps", port);
  e = mAeroForce->getProperty("v");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/v-aero-mps", port);
  c = new SiToUnitExpressionImpl(uFeetPSecond);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/v-aero-fps", port);
  e = mAeroForce->getProperty("w");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/w-aero-mps", port);
  c = new SiToUnitExpressionImpl(uFeetPSecond);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/w-aero-fps", port);


  // Dynamic pressure values.
  e = mAeroForce->getProperty("dynamicPressure");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("aero/qbar-pa", port);
  c = new SiToUnitExpressionImpl(uPoundPFt2);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("aero/qbar-psf", port);

  // Temperature.
  e = mAeroForce->getProperty("temperature");
  c = new SiToUnitExpressionImpl(uRankine);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/tat-r", port);
  c = new SiToUnitExpressionImpl(uFahrenheit);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/tat-f", port);

  // Braindead: a pressure value in velocities ...
  e = mAeroForce->getProperty("pressure");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("velocities/pt-pascal", port);
  c = new SiToUnitExpressionImpl(uPoundPFt2);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("velocities/pt-lbs_sqft", port);



  e = mAeroForce->getProperty("wingSpan");
  c = new SiToUnitExpressionImpl(uFoot);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("metrics/bw-ft", port);

  e = mAeroForce->getProperty("wingArea");
  c = new SiToUnitExpressionImpl(uFoot2);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("metrics/Sw-sqft", port);

  e = mAeroForce->getProperty("coord");
  c = new SiToUnitExpressionImpl(uFoot);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("metrics/cbarw-ft", port);

  e = mAeroForce->getProperty("wingSpanOver2Speed");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("aero/bi2vel", port);
  e = mAeroForce->getProperty("coordOver2Speed");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("aero/ci2vel", port);

  // Angle of attack.
  e = mAeroForce->getProperty("alpha");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("aero/alpha-rad", port);
  AbsExpressionImpl* a = new AbsExpressionImpl();
  a->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(a));
  registerJSBExpression("aero/mag-alpha-rad", port);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("aero/alpha-deg", port);
  a = new AbsExpressionImpl();
  a->setInputProperty(Property(c));
  port = new Port;
  port->setProperty(Property(a));
  registerJSBExpression("aero/mag-alpha-deg", port);

  // Angle of sideslip.
  e = mAeroForce->getProperty("beta");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("aero/beta-rad", port);
  a = new AbsExpressionImpl();
  a->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(a));
  registerJSBExpression("aero/mag-beta-rad", port);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("aero/beta-deg", port);
  a = new AbsExpressionImpl();
  a->setInputProperty(Property(c));
  port = new Port;
  port->setProperty(Property(a));
  registerJSBExpression("aero/mag-beta-deg", port);


  // Time derivative of alpha.
  e = mAeroForce->getProperty("alphaDot");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("aero/alphadot-rad_sec", port);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("aero/alphadot-deg", port);

  // Time derivative of beta.
  e = mAeroForce->getProperty("betaDot");
  port = new Port;
  port->setProperty(e);
  registerJSBExpression("aero/betadot-rad_sec", port);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("aero/betadot-deg", port);

  // The quotient agl/wingspan
  e = mAeroForce->getProperty("hOverWingSpan");
  c = new SiToUnitExpressionImpl(uFoot);
  c->setInputProperty(e);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("aero/h_b-cg-ft", port);
  port = new Port;
  port->setProperty(Property(c));
  registerJSBExpression("aero/h_b-mac-ft", port);
}


} // namespace OpenFDM
