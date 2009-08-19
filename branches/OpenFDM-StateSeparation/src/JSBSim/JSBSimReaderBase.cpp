/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "JSBSimReaderBase.h"

#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>
#include <stack>

#include <OpenFDM/Vector.h>
#include <OpenFDM/Matrix.h>
#include <OpenFDM/Quaternion.h>

#include <OpenFDM/Bias.h>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/DeadBand.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/TransferFunction.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Input.h>
#include <OpenFDM/Output.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/MaxModel.h>
#include <OpenFDM/AirSpring.h>
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
#include <OpenFDM/UnaryFunction.h>
#include <OpenFDM/Unit.h>
#include <OpenFDM/UnitConversion.h>
#include <OpenFDM/WheelContact.h>
#include <OpenFDM/DiscBrake.h>

#include <OpenFDM/ReaderWriter.h>

#include <OpenFDM/XML/XMLReader.h>
#include <OpenFDM/XML/ContentHandler.h>
#include <OpenFDM/XML/ErrorHandler.h>
#include <OpenFDM/XML/Attributes.h>

#include <OpenFDM/XML/ExpatXMLReader.h> // FIXME

#include "JSBSimAerodynamic.h"
#include "JSBSimAerosurfaceScale.h"
#include "JSBSimKinemat.h"
#include "JSBSimScheduledGain.h"

namespace OpenFDM {

JSBSimReaderBase::JSBSimReaderBase(void)
{
  reset();
}

JSBSimReaderBase::~JSBSimReaderBase(void)
{
}

void
JSBSimReaderBase::reset(void)
{
  // Throw away any possibly loaded vehicle
  mSystem = 0;
  mTopLevelGroup = 0;
  mAeroForce = 0;
}

void
JSBSimReaderBase::addAircraftPath(const std::string& path)
{
  mAircraftPath.push_back(path);
}

void
JSBSimReaderBase::addEnginePath(const std::string& path)
{
  mEnginePath.push_back(path);
}

void
JSBSimReaderBase::addSystemPath(const std::string& path)
{
  mSystemPath.push_back(path);
}

bool
JSBSimReaderBase::openFile(const std::list<std::string>& paths,
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

SharedPtr<XMLElement>
JSBSimReaderBase::parseXMLStream(std::istream& stream)
{
  // Get a parser FIXME
  SharedPtr<XML::XMLReader> reader = new XML::ExpatXMLReader;
  
  // Set the handlers
  SharedPtr<SimpleContentHandler> ch = new SimpleContentHandler;
  reader->setContentHandler(ch);
  SharedPtr<SimpleErrorHandler> eh = new SimpleErrorHandler;
  reader->setErrorHandler(eh);

  // Parse ...
  reader->parse(stream);

  return ch->getTopElement();
}

std::string
JSBSimReaderBase::propNameFromJSBSim(const std::string& jsbSymbol)
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
JSBSimReaderBase::propMinusFromJSBSim(const std::string& jsbSymbol)
{
  /// Returns true in case the JSBSim property contains a minus
  return 0 < jsbSymbol.size() && jsbSymbol[0] == '-';
}

std::string
JSBSimReaderBase::normalizeComponentName(const std::string& name)
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

bool
JSBSimReaderBase::connectJSBExpression(const std::string& name,
                                       const Port* pa, bool recheckAeroProp)
{
  if (!pa)
    return false;

  if (!propMinusFromJSBSim(name)) {
    mPropertyManager.addConsumer(canonicalJSBProperty(name), pa);
  } else {
    SharedPtr<Group> group = getGroup(pa);
    if (!group)
      return error("Could not add output model \"" + name + "\"");

    Gain* gain = new Gain("Minus " + canonicalJSBProperty(name), -1);
    group->addChild(gain);
    if (!group->connect(gain->getOutputPort(), pa))
      return false;

    mPropertyManager.addConsumer(canonicalJSBProperty(name),
                                 gain->getInputPort(0));
  }

  return true;
}

bool
JSBSimReaderBase::registerExpression(const std::string& name, const Port* port)
{
  if (name.empty())
    return false;
  if (!port)
    return false;
  if (mPropertyManager.exists(name))
    // FIXME, at least warn about that ...
    return true;
    // return error("Already have an expression for \"" + name + "\"");

  mPropertyManager.setProvider(name, port);
  return true;
}

std::string
JSBSimReaderBase::canonicalJSBProperty(std::string fullName)
{
  if (fullName.empty())
    return fullName;

  if (fullName[0] == '-')
    fullName = fullName.substr(1);

  std::string path = JSBSimProperty::propertyPath(fullName);
  std::string name = JSBSimProperty::propertyName(fullName);

  if (!path.empty() && path[0] == '/')
    path = path.substr(1);
  else if (path.find("fdm/jsbsim/") != 0)
    path = "fdm/jsbsim/" + path;

  return JSBSimProperty::simplify(path + '/' + name);
}

bool
JSBSimReaderBase::registerJSBExpression(const std::string& name, const Port* port)
{
  if (name.empty())
    return error("Registering empty property?");
  if (!port)
    return error("Registering zero port?");
  if (name[0] == '/')
    registerExpression(name.substr(1), port);
  else
    registerExpression("fdm/jsbsim/" + name, port);

  // Well, just an other kind of black magic ...
  if (name[0] == '/') {
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
  return true;
}

bool
JSBSimReaderBase::provideSubstitutes()
{
  bool foundSubstitute;
  do {
    foundSubstitute = false;
    JSBSimPropertyManager::PropertyMap::iterator i;
    for (i = mPropertyManager.getPropertyMap().begin();
         i != mPropertyManager.getPropertyMap().end(); ++i) {
      if (i->second.hasProviderPort())
        continue;
      if (!provideSubstitute(i->first))
        return false;
      foundSubstitute = true;
    }
  } while (foundSubstitute);

  return true;
}

bool
JSBSimReaderBase::provideSubstitute(const std::string& propName)
{
  if (propName.find("/") == 0) {
    addInputModel("Property Input " + propName, propName);
    return true;
    
  } else if (propName.find("controls/") == 0) {
    addInputModel("Control " + propName, propName);
    return true;

  } else if (propName == "fdm/jsbsim/velocities/vt-mps") {
    registerExpression(propName, mAeroForce->getTrueAirSpeedPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/vc-mps") {
    registerExpression(propName, mAeroForce->getCalibratedAirSpeedPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/ve-mps") {
    registerExpression(propName, mAeroForce->getEquivalentAirSpeedPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/vg-mps") {
    registerExpression(propName, mAeroForce->getGroundSpeedPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/hdot-mps") {
    registerExpression(propName, mAeroForce->getClimbSpeedPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/mach-norm" ||
             propName == "fdm/jsbsim/velocities/mach") {
    registerExpression(propName, mAeroForce->getMachPort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/alpha-rad") {
    registerExpression(propName, mAeroForce->getAlphaPort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/mag-alpha-rad") {
    std::string inPropName = "fdm/jsbsim/aero/alpha-rad";
    const Port* port = addAbsModel(propName, inPropName);
    registerExpression(propName, port);
    return true;

  } else if (propName == "fdm/jsbsim/aero/alphadot-rad_sec") {
    registerExpression(propName, mAeroForce->getAlphaDotPort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/beta-rad") {
    registerExpression(propName, mAeroForce->getBetaPort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/mag-beta-rad") {
    std::string inPropName = "fdm/jsbsim/aero/beta-rad";
    const Port* port = addAbsModel(propName, inPropName);
    registerExpression(propName, port);
    return true;

  } else if (propName == "fdm/jsbsim/aero/betadot-rad_sec") {
    registerExpression(propName, mAeroForce->getBetaDotPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/p-rad_sec") {
    registerExpression(propName, mAeroForce->getPPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/q-rad_sec") {
    registerExpression(propName, mAeroForce->getQPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/r-rad_sec") {
    registerExpression(propName, mAeroForce->getRPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/p-aero-rad_sec") {
    registerExpression(propName, mAeroForce->getPAeroPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/q-aero-rad_sec") {
    registerExpression(propName, mAeroForce->getQAeroPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/r-aero-rad_sec") {
    registerExpression(propName, mAeroForce->getRAeroPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/u-aero-mps") {
    registerExpression(propName, mAeroForce->getUPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/v-aero-mps") {
    registerExpression(propName, mAeroForce->getVPort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/w-aero-mps") {
    registerExpression(propName, mAeroForce->getWPort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/qbar-pa") {
    registerExpression(propName, mAeroForce->getDynamicPressurePort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/pt-pa" ||
             propName == "fdm/jsbsim/propulsion/pt-pa") {
    // FIXME, is this static pressure in JSB or is that total pressure??
    registerExpression(propName, mAeroForce->getStaticPressurePort());
    return true;

  } else if (propName == "fdm/jsbsim/velocities/tat-k" ||
             propName == "fdm/jsbsim/propulsion/tat-k") {
    registerExpression(propName, mAeroForce->getTemperaturePort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/bi2vel") {
    registerExpression(propName, mAeroForce->getWingSpanOver2SpeedPort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/ci2vel") {
    registerExpression(propName, mAeroForce->getChordOver2SpeedPort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/ci2vel") {
    registerExpression(propName, mAeroForce->getChordOver2SpeedPort());
    return true;

  } else if (propName == "fdm/jsbsim/metrics/bw-m") {
    registerExpression(propName, mAeroForce->getWingSpanPort());
    return true;

  } else if (propName == "fdm/jsbsim/metrics/Sw-sqm") {
    registerExpression(propName, mAeroForce->getWingAreaPort());
    return true;

  } else if (propName == "fdm/jsbsim/metrics/cbarw-m") {
    registerExpression(propName, mAeroForce->getChordPort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/h_b-cg-m") {
    registerExpression(propName, mAeroForce->getHOverWingSpanPort());
    return true;

  } else if (propName == "fdm/jsbsim/aero/h_b-mac-m") {
    // FIXME wrong value
    registerExpression(propName, mAeroForce->getHOverWingSpanPort());
    return true;

  } else if (propName == "fdm/jsbsim/fcs/aileron-cmd-norm") {
    std::string control = "/controls/flight/aileron";
    registerExpression(propName, addInputModel("Aileron Input", control));
    return true;

  } else if (propName == "fdm/jsbsim/fcs/roll-trim-cmd-norm") {
    std::string control = "/controls/flight/aileron-trim";
    registerExpression(propName, addInputModel("Aileron Trim Input", control));
    return true;

  } else if (propName == "fdm/jsbsim/fcs/elevator-cmd-norm") {
    std::string control = "/controls/flight/elevator";
    registerExpression(propName, addInputModel("Elevator Input", control));
    return true;

  } else if (propName == "fdm/jsbsim/fcs/pitch-trim-cmd-norm") {
    std::string control = "/controls/flight/elevator-trim";
    registerExpression(propName, addInputModel("Elevator Trim Input", control));
    return true;

  } else if (propName == "fdm/jsbsim/fcs/steer-cmd-norm") {
    std::string control = "/controls/flight/rudder";
    registerExpression(propName, addInputModel("Rudder Input", control));
    return true;
  } else if (propName == "fdm/jsbsim/fcs/flap-cmd-norm") {
    std::string control = "/controls/flight/flaps";
    registerExpression(propName, addInputModel("Flaps Input", control));
    return true;
  } else if (propName == "fdm/jsbsim/fcs/flap-pos-norm") {
    Gain* gain = new Gain("Implicit fdm/jsbsim/fcs/flap-pos-norm");
    addFCSModel(gain);
    connectJSBExpression("fdm/jsbsim/fcs/flap-cmd-norm", gain->getInputPort(0));
    registerExpression(propName, gain->getOutputPort());
    return true;
  } else if (propName == "fdm/jsbsim/fcs/speedbrake-cmd-norm") {
    std::string control = "/controls/flight/speedbrake";
    registerExpression(propName, addInputModel("Speedbrake Input", control));
    return true;

  } else if (propName == "fdm/jsbsim/fcs/spoiler-cmd-norm") {
    std::string control = "/controls/flight/spoiler";
    registerExpression(propName, addInputModel("Spoiler Input", control));
    return true;

  } else if (propName == "fdm/jsbsim/gear/gear-cmd-norm") {
    std::string control = "/controls/gear/gear-down";
    registerExpression(propName, addInputModel("Gear Input", control));
    return true;

  } else if (propName == "fdm/jsbsim/fcs/rudder-cmd-norm") {
    std::string inPropName = "fdm/jsbsim/fcs/steer-cmd-norm";
    const Port* port = addInverterModel(inPropName, inPropName);
    registerExpression(propName, port);
    return true;

  } else if (propName == "fdm/jsbsim/fcs/yaw-trim-cmd-norm") {
    std::string inPropName = "/controls/flight/rudder-trim";
    const Port* port = addInverterModel(inPropName, inPropName);
    registerExpression(propName, port);
    return true;

  } else if (propName.find("fdm/jsbsim/fcs/throttle-cmd-norm") == 0) {
    std::string control = "/controls/engines/engine" +
      propName.substr(32) + "/throttle";
    registerExpression(propName, addInputModel("Throttle Input", control));
    return true;

  } else if (propName.find("fdm/jsbsim/fcs/mixture-cmd-norm") == 0) {
    std::string control = "/controls/engines/engine" + propName.substr(31)
      + "/mixture";
    registerExpression(propName, addInputModel("Mixture Input", control));
    return true;

  } else if (propName.find("fdm/jsbsim/fcs/advance-cmd-norm") == 0) {
    std::string control = "/controls/engines/engine" + 
      propName.substr(31) + "/propeller-pitch";
    registerExpression(propName, addInputModel("Advance Input", control));
    return true;

  } else if (propName == "fdm/jsbsim/gear/right-brake-cmd-norm") {
    MaxModel* maxModel = new MaxModel("Right Brake Max");
    maxModel->setNumMaxInputs(3);
    addFCSModel(maxModel);
    
    const Port* pilotBr = addInputModel("Right Brake",
                                        "/controls/gear/brake-right");
    
    mTopLevelGroup->connect(pilotBr, maxModel->getInputPort(0));
    
    const Port* copilotBr = addInputModel("Right Copilot Brake",
                                          "/controls/gear/copilot-brake-right");
    mTopLevelGroup->connect(copilotBr, maxModel->getInputPort(1));
    
    connectJSBExpression("/controls/gear/brake-parking",
                         maxModel->getInputPort(2));
    
    registerExpression(propName, maxModel->getPort("output"));
    return true;
    
  } else if (propName == "fdm/jsbsim/gear/left-brake-cmd-norm") {
    MaxModel* maxModel = new MaxModel("Left Brake Max");
    addFCSModel(maxModel);
    maxModel->setNumMaxInputs(3);
    
    const Port* pilotBr = addInputModel("Left Brake",
                                        "/controls/gear/brake-left");
    mTopLevelGroup->connect(pilotBr, maxModel->getInputPort(0));
    
    const Port* copilotBr = addInputModel("Left Copilot Brake",
                                          "/controls/gear/copilot-brake-left");
    mTopLevelGroup->connect(copilotBr, maxModel->getInputPort(1));
    
    connectJSBExpression("/controls/gear/brake-parking",
                         maxModel->getInputPort(2));
    
    registerExpression(propName, maxModel->getPort("output"));
    return true;

  } else if (JSBSimProperty::startsWith(propName, "fdm/jsbsim/fcs/mag-")) {
    std::string inPropName = "fdm/jsbsim/fcs/" + propName.substr(19);
    const Port* port = addAbsModel(propName, inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-deg")) {
    std::string inPropName = propName.substr(0, propName.size() - 3) + "rad";
    const Port* port = addFromUnit(propName, Unit::degree(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-rad")) {
    std::string inPropName = propName.substr(0, propName.size() - 3) + "deg";
    const Port* port = addToUnit(propName, Unit::degree(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-ft")) {
    std::string inPropName = propName.substr(0, propName.size() - 2) + "m";
    const Port* port = addToUnit(propName, Unit::foot(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-in")) {
    std::string inPropName = propName.substr(0, propName.size() - 2) + "m";
    const Port* port = addToUnit(propName, Unit::inch(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-sqft")) {
    std::string inPropName = propName.substr(0, propName.size() - 2) + "m";
    const Port* port = addToUnit(propName, Unit::squareFoot(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-fps")) {
    std::string inPropName = propName.substr(0, propName.size() - 3) + "mps";
    const Port* port = addToUnit(propName, Unit::footPerSecond(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-kts")) {
    std::string inPropName = propName.substr(0, propName.size() - 3) + "mps";
    const Port* port = addToUnit(propName, Unit::knots(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-deg_sec")) {
    std::string inPropName = propName.substr(0, propName.size()-7) + "rad_sec";
    const Port* port = addToUnit(propName, Unit::degree(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-psf")) {
    std::string inPropName = propName.substr(0, propName.size() - 3) + "pa";
    const Port* port = addToUnit(propName, Unit::lbfPerSquareFoot(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-lbs_sqft")) {
    std::string inPropName = propName.substr(0, propName.size() - 8) + "pa";
    const Port* port = addToUnit(propName, Unit::lbfPerSquareFoot(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-r")) {
    std::string inPropName = propName.substr(0, propName.size() - 1) + "k";
    const Port* port = addToUnit(propName, Unit::rankine(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-f")) {
    std::string inPropName = propName.substr(0, propName.size() - 1) + "k";
    const Port* port = addToUnit(propName, Unit::degreeFarenheit(), inPropName);
    registerExpression(propName, port);
    return true;

  } else if (JSBSimProperty::endsWith(propName, "-c")) {
    std::string inPropName = propName.substr(0, propName.size() - 1) + "k";
    const Port* port = addToUnit(propName, Unit::degreeCelsius(), inPropName);
    registerExpression(propName, port);
    return true;

  } else {
    std::cout << "Could not find model providing property \"" <<  propName
              << "\", setting value to zero." << std::endl;

    Matrix m(1, 1);
    m(0, 0) = 0;
    ConstModel* model = new ConstModel(propName, m);
    addFCSModel(model);
    registerExpression(propName, model->getPort("output"));
    return true;
  }

  return false;
}

bool
JSBSimReaderBase::connect()
{
  JSBSimPropertyManager::PropertyMap::iterator i;
  for (i = mPropertyManager.getPropertyMap().begin();
       i != mPropertyManager.getPropertyMap().end(); ++i) {
    if (!i->second.connect())
      return error("Error connecting \"" + i->first + "\"");
  }
  return true;
}

const Port*
JSBSimReaderBase::addInputModel(const std::string& name,
                                const std::string& propName, real_type gain)
{
  Input* input = new Input(name);
  input->setInputName(propName);
  input->setInputGain(gain);
  addFCSModel(input);
  const Port* port = input->getPort("output");
  registerExpression(propName, port);
  return port;
}

void
JSBSimReaderBase::addOutputModel(const Port* out,
                                 const std::string& name,
                                 const std::string& propName, real_type gain)
{
  SharedPtr<Group> group = getGroup(out);
  if (!group) {
    error("Could not add output model \"" + name + "\"");
    return;
  }
  Output* output = new Output(name + " Output");
  group->addChild(output);
  output->setOutputName(propName);
  output->setOutputGain(gain);
  group->connect(out, output->getPort("input"));
}

void
JSBSimReaderBase::addOutputModel(const std::string& inputProp,
                                 const std::string& name,
                                 const std::string& propName, real_type gain)
{
  Output* output = new Output(name + " Output");
  mTopLevelGroup->addChild(output);
  output->setOutputName(propName);
  output->setOutputGain(gain);
  connectJSBExpression(inputProp, output->getPort("input"));
}

const Port*
JSBSimReaderBase::addInverterModel(const std::string& name, const Port* in)
{
  SharedPtr<Group> group = getGroup(in);
  if (!group) {
    error("Could not add inverter model \"" + name + "\"");
    return 0;
  }
  UnaryFunction *unary
    = new UnaryFunction(name + " Inverter", UnaryFunction::Minus);
  group->addChild(unary);
  if (!group->connect(in, unary->getPort("input")))
    return 0;
  return unary->getPort("output");
}

const Port*
JSBSimReaderBase::addInverterModel(const std::string& name,
                                   const std::string& inProp)
{
  SharedPtr<Group> group = mTopLevelGroup;
  UnaryFunction *unary
    = new UnaryFunction(name + " Inverter", UnaryFunction::Minus);
  group->addChild(unary);
  if (!connectJSBExpression(inProp, unary->getInputPort(0)))
    return 0;
  return unary->getPort("output");
}

const Port*
JSBSimReaderBase::addAbsModel(const std::string& name, const Port* in)
{
  SharedPtr<Group> group = getGroup(in);
  if (!group) {
    error("Could not add inverter model \"" + name + "\"");
    return 0;
  }
  UnaryFunction *unary
    = new UnaryFunction(name + " Abs", UnaryFunction::Abs);
  group->addChild(unary);
  if (!group->connect(in, unary->getPort("input")))
    return 0;
  return unary->getPort("output");
}

const Port*
JSBSimReaderBase::addAbsModel(const std::string& name,
                              const std::string& inProp)
{
  SharedPtr<Group> group = mTopLevelGroup;
  UnaryFunction *unary
    = new UnaryFunction(name + " Abs", UnaryFunction::Abs);
  group->addChild(unary);
  if (!connectJSBExpression(inProp, unary->getInputPort(0)))
    return 0;
  return unary->getPort("output");
}

const Port*
JSBSimReaderBase::addConstModel(const std::string& name, real_type value,
                                const NodePath& path)
{
  Matrix m(1, 1);
  m(0, 0) = 0;
  ConstModel* cModel = new ConstModel(name, m);
  if (path.empty())
    addFCSModel(cModel);
  else {
    SharedPtr<const Node> parent = path.back();
    Group* group = const_cast<Group*>(dynamic_cast<const Group*>(parent.get()));
    group->addChild(cModel);
  }
  return cModel->getPort("output");
}

const Port*
JSBSimReaderBase::addToUnit(const std::string& name, Unit u, const Port* in)
{
  SharedPtr<Group> group = getGroup(in);
  if (!group) {
    error("Could not add unit conversion model \"" + name + "\"");
    return 0;
  }
  UnitConversion* unitConv
    = new UnitConversion(name, UnitConversion::BaseUnitToUnit, u);
  group->addChild(unitConv);
  if (!group->connect(in, unitConv->getPort("input")))
    return 0;
  return unitConv->getOutputPort();
}

const Port*
JSBSimReaderBase::addToUnit(const std::string& name, Unit u,
                            const std::string& inProp)
{
  SharedPtr<Group> group = mTopLevelGroup;
  UnitConversion* unitConv
    = new UnitConversion(name, UnitConversion::BaseUnitToUnit, u);
  group->addChild(unitConv);
  if (!connectJSBExpression(inProp, unitConv->getInputPort(0)))
    return 0;
  return unitConv->getOutputPort();
}

const Port*
JSBSimReaderBase::addFromUnit(const std::string& name, Unit u, const Port* in)
{
  SharedPtr<Group> group = getGroup(in);
  if (!group) {
    error("Could not add unit conversion model \"" + name + "\"");
    return 0;
  }
  UnitConversion* unitConv
    = new UnitConversion(name, UnitConversion::UnitToBaseUnit, u);
  group->addChild(unitConv);
  if (!group->connect(in, unitConv->getPort("input")))
    return 0;
  return unitConv->getPort("output");
}

const Port*
JSBSimReaderBase::addFromUnit(const std::string& name, Unit u,
                            const std::string& inProp)
{
  SharedPtr<Group> group = mTopLevelGroup;
  UnitConversion* unitConv
    = new UnitConversion(name, UnitConversion::UnitToBaseUnit, u);
  group->addChild(unitConv);
  if (!connectJSBExpression(inProp, unitConv->getInputPort(0)))
    return 0;
  return unitConv->getOutputPort();
}

SharedPtr<Group>
JSBSimReaderBase::getGroup(const Port* in)
{
  if (!in) {
    error("Could not find model group for input port: no port given!");
    return 0;
  }
  SharedPtr<const Node> node = in->getNode();
  if (!node) {
    error("Could not find model group for input port: "
          "port does not belong to a Node!");
    return 0;
  }
  SharedPtr<const Node> parent = node->getParent(0).lock();
  SharedPtr<Group> group = const_cast<Group*>(dynamic_cast<const Group*>(parent.get()));
  if (!group) {
    error("Could not find model group for input port: "
          "model has no parent!");
    return 0;
  }
  return group;
}

void
JSBSimReaderBase::addFCSModel(Node* model)
{
  // FIXME
  mTopLevelGroup->addChild(model);
}

const Port*
JSBSimReaderBase::addMultiBodyConstModel(const std::string& name, real_type value)
{
  Matrix m(1, 1);
  m(0, 0) = value;
  ConstModel* cModel = new ConstModel(name, m);
  addMultiBodyModel(cModel);
  return cModel->getPort("output");
}

void
JSBSimReaderBase::addMultiBodyModel(Node* model)
{
  // FIXME
  mTopLevelGroup->addChild(model);
}

const Port*
JSBSimReaderBase::getTablePrelookup(const std::string& name,
                                    const std::string& inputProperty,
                                    const BreakPointVector& tl)
{
  std::string normalizedInputProperty = JSBSimProperty::simplify(inputProperty);
  // First check if we already have a table lookup for this port/brakepoint
  // combination. If so return that output port
  std::vector<BreakPointLookupEntry>::iterator it;
  for (it = mBreakPointVectors.begin(); it != mBreakPointVectors.end(); ++it) {
    if (tl == it->lookup->getBreakPointVector()
        && normalizedInputProperty == it->propertyName)
      return it->lookup->getPort("output");
  }

  // No sharable table lookup found, we need to create a new one
  BreakPointLookup* tablePreLookup
    = new BreakPointLookup(name + " Table Prelookup");
  addMultiBodyModel(tablePreLookup);
  tablePreLookup->setBreakPointVector(tl);
  connectJSBExpression(normalizedInputProperty,
                       tablePreLookup->getInputPort(0));
  mBreakPointVectors.push_back(BreakPointLookupEntry(normalizedInputProperty,
                                                     tablePreLookup));
  return tablePreLookup->getOutputPort();
}

} // namespace OpenFDM
