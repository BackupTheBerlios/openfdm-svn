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

const Port*
JSBSimReaderBase::lookupJSBExpression(const std::string& name,
                                      const NodePath& path,
                                      bool recheckAeroProp)
{
  // Convert to something being able to look up
  std::string propName = propNameFromJSBSim(name);
  
  const Port* port = 0;
  if (!mExpressionTable.exists(propName)) {
    // Not yet available, so look and see if it is an input
    if (recheckAeroProp)
      port = createAndScheduleAeroProp(propName, path);

    if (!port /*|| !port->isConnected()*/) {
      // Not yet available, so look and see if it is an input
      port = createAndScheduleInput(propName, path);
      
      // Ok, still not available, create a constant zero thing and bail out ...
      if (!port /*|| !port->isConnected()*/) {
        std::cerr << "Creating expression \"" << propName << "\"" << std::endl;

        return addConstModel(propName + " constant", 0, path);
      }
    }
  }

  // Just get that one, if we already have it
  port = mExpressionTable.routeTo(propName, path);

  // If we need the negative input, just multiply with a negative gain
  if (propMinusFromJSBSim(name))
    return addInverterModel(name.substr(name.rfind('/')+1), port);
  else
    return port;
}

bool
JSBSimReaderBase::connectJSBExpression(const std::string& name,
                                       const Port* pa, bool recheckAeroProp)
{
  if (!pa)
    return false;
  SharedPtr<const Node> model = pa->getNode();
  if (!model)
    return false;
  NodePath path = model->getNodePathList().front();
  const Port* pp = lookupJSBExpression(name, path, recheckAeroProp);
  SharedPtr<const Node> parent = model->getParent(0).lock();
  Group* group = const_cast<Group*>(dynamic_cast<const Group*>(parent.get()));
  return group->connect(pp, pa);
}

void
JSBSimReaderBase::registerExpression(const std::string& name, const Port* port)
{
  if (name.size() <= 0)
    return;
  if (!port /*|| !port->isConnected()*/)
    return;
  if (mExpressionTable.exists(name)) {
    std::cerr << "Already have an expression for " << name << std::endl;
    return;
  }

  mExpressionTable.registerPort(name, port);
}

void
JSBSimReaderBase::registerJSBExpression(const std::string& name, const Port* port)
{
  if (name.empty())
    return;
  if (!port /*|| !port->isConnected()*/)
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

const Port*
JSBSimReaderBase::createAndScheduleInput(const std::string& propName,
                                         const NodePath& path)
{
  // This routine checks if the given propName is a special JSBSim
  // input property. If so, it schedules and registers a discrete input model.
  // If the propName points directly into the controls directory,
  // schedule an input
  if (propName.find("controls/") == 0) {
//     std::string inputName = propName.substr(propName.rfind('/'));
    std::string inputName = propName;
    return addInputModel("Control " + inputName, propName);
  } else {
    const Port* port = 0;
    if (propName == "fdm/jsbsim/fcs/aileron-cmd-norm") {
      port = addInputModel("Aileron",
                           "controls/flight/aileron");

    } else if (propName == "fdm/jsbsim/fcs/roll-trim-cmd-norm") {
      port = addInputModel("Aileron Trim",
                           "controls/flight/aileron-trim");

    } else if (propName == "fdm/jsbsim/fcs/elevator-cmd-norm") {
      port = addInputModel("Elevator",
                           "controls/flight/elevator");

    } else if (propName == "fdm/jsbsim/fcs/pitch-trim-cmd-norm") {
      port = addInputModel("Elevator Trim",
                           "controls/flight/elevator-trim");

    } else if (propName == "fdm/jsbsim/fcs/rudder-cmd-norm") {
//       port = addInputModel("Rudder", "controls/flight/rudder");
      port = lookupJSBExpression("fcs/steer-cmd-norm", path);
      port = addInverterModel("Rudder inverter", port);

    } else if (propName == "fdm/jsbsim/fcs/yaw-trim-cmd-norm") {
      port = addInputModel("Yaw Trim",
                           "controls/flight/rudder-trim");
      port = addInverterModel("Yaw Trim inverter", port);

    } else if (propName == "fdm/jsbsim/fcs/steer-cmd-norm") {
      // FIXME is seperate in flightgear ???
      // port = addInputModel("Steering", "controls/gear/steering");
      // if this is replaced note that the above line needs to be chenged too
      port = addInputModel("Rudder", "controls/flight/rudder");

    } else if (propName.find("fdm/jsbsim/fcs/steer-pos-deg") == 0) {
      return lookupJSBExpression("fcs/steer-cmd-norm", path);

    } else if (propName == "fdm/jsbsim/fcs/flap-cmd-norm") {
      port = addInputModel("Flaps",
                           "controls/flight/flaps");

    } else if (propName == "fdm/jsbsim/fcs/speedbrake-cmd-norm") {
      port = addInputModel("Speedbrake",
                           "controls/flight/speedbrake");

    } else if (propName == "fdm/jsbsim/fcs/spoiler-cmd-norm") {
      port = addInputModel("Spoiler",
                           "controls/flight/spoiler");


    } else if (propName.find("fdm/jsbsim/fcs/throttle-cmd-norm") == 0) {
      std::string control = "controls/engines/engine" +
        propName.substr(32) + "/throttle";
      port = addInputModel("Throttle Input " + propName.substr(32),
                           control);

    } else if (propName.find("fdm/jsbsim/fcs/throttle-pos-norm") == 0) {
      std::string cmd = "fcs/throttle-cmd-norm" + propName.substr(32);
      return lookupJSBExpression(cmd, path);


    } else if (propName.find("fdm/jsbsim/fcs/mixture-cmd-norm") == 0) {
      std::string control = "controls/engines/engine" + 
        propName.substr(31) + "/mixture";
      std::string number = "0";
      if (32 <= propName.size())
        number = propName.substr(32, 1);
      port = addInputModel("Mixture Input " + number, control);

    } else if (propName.find("fdm/jsbsim/fcs/mixture-pos-norm") == 0) {
      std::string cmd = "fcs/mixture-cmd-norm" + propName.substr(31);
      return lookupJSBExpression(cmd, path);


    } else if (propName.find("fdm/jsbsim/fcs/advance-cmd-norm") == 0) {
      std::string control = "controls/engines/engine" + 
        propName.substr(31) + "/propeller-pitch";
      std::string number = "0";
      if (32 <= propName.size())
        number = propName.substr(32, 1);
      port = addInputModel("Propeller Pitch Input " + number, control);

    } else if (propName.find("fdm/jsbsim/fcs/advance-pos-norm") == 0) {
      std::string cmd = "fcs/advance-cmd-norm" + propName.substr(31);
      return lookupJSBExpression(cmd, path);

    } else if (propName == "fdm/jsbsim/gear/gear-cmd-norm") {
      port = addInputModel("Gear Retract",
                           "controls/gear/gear-down");

    } else if (propName == "fdm/jsbsim/gear/gear-pos-norm") {
      return lookupJSBExpression("gear/gear-cmd-norm", path);

    } else if (propName == "controls/gear/brake-parking") {
      port = addInputModel("Parking Brake",
                           "controls/gear/brake-parking");

    } else if (propName == "fdm/jsbsim/gear/right-brake-pos-norm") {
      MaxModel* maxModel = new MaxModel("Right Brake Max");
      maxModel->setNumMaxInputs(3);
      addFCSModel(maxModel);

      const Port* pilotBr = addInputModel("Right Brake",
                                    "controls/gear/brake-right");

      mTopLevelGroup->connect(pilotBr, maxModel->getInputPort(0));

      const Port* copilotBr = addInputModel("Right Copilot Brake",
                                      "controls/gear/copilot-brake-right");
      mTopLevelGroup->connect(copilotBr, maxModel->getInputPort(1));

      const Port* parkBr = lookupJSBExpression("/controls/gear/brake-parking", maxModel->getNodePathList().front());
      mTopLevelGroup->connect(parkBr, maxModel->getInputPort(2));

      port = maxModel->getPort("output");

    } else if (propName == "fdm/jsbsim/gear/left-brake-pos-norm") {
      MaxModel* maxModel = new MaxModel("Left Brake Max");
      addFCSModel(maxModel);
      maxModel->setNumMaxInputs(3);

      const Port* pilotBr = addInputModel("Left Brake",
                                    "controls/gear/brake-left");
      mTopLevelGroup->connect(pilotBr, maxModel->getInputPort(0));

      const Port* copilotBr = addInputModel("Left Copilot Brake",
                                      "controls/gear/copilot-brake-left");
      mTopLevelGroup->connect(copilotBr, maxModel->getInputPort(1));
      
      const Port* parkBr = lookupJSBExpression("/controls/gear/brake-parking", maxModel->getNodePathList().front());
      mTopLevelGroup->connect(parkBr, maxModel->getInputPort(2));

      port = maxModel->getPort("output");

    } else if (propName == "fdm/jsbsim/fcs/elevator-pos-rad") {
      port = lookupJSBExpression("fcs/elevator-pos-deg", path, false);
      port = addFromUnit("elevator-pos-rad unit", Unit::degree(), port);

    } else if (propName == "fdm/jsbsim/fcs/elevator-pos-deg") {
      port = lookupJSBExpression("fcs/elevator-pos-rad", path, false);
      port = addToUnit("elevator-pos-deg unit", Unit::degree(), port);

    } else if (propName == "fdm/jsbsim/fcs/left-aileron-pos-rad") {
      port = lookupJSBExpression("fcs/left-aileron-pos-deg", path, false);
      port = addFromUnit("left-aileron-pos-rad unit", Unit::degree(), port);

    } else if (propName == "fdm/jsbsim/fcs/left-aileron-pos-deg") {
      port = lookupJSBExpression("fcs/left-aileron-pos-rad", path, false);
      port = addToUnit("left-aileron-pos-deg unit", Unit::degree(), port);

    } else if (propName == "fdm/jsbsim/fcs/right-aileron-pos-rad") {
      port = lookupJSBExpression("fcs/right-aileron-pos-deg", path, false);
      port = addFromUnit("right-aileron-pos-rad unit", Unit::degree(), port);

    } else if (propName == "fdm/jsbsim/fcs/right-aileron-pos-deg") {
      port = lookupJSBExpression("fcs/right-aileron-pos-rad", path, false);
      port = addToUnit("right-aileron-pos-deg unit", Unit::degree(), port);

    } else if (propName == "fdm/jsbsim/fcs/aileron-pos-rad") {
      port = lookupJSBExpression("fcs/aileron-pos-deg", path, false);
      port = addFromUnit("aileron-pos-rad unit", Unit::degree(), port);

    } else if (propName == "fdm/jsbsim/fcs/aileron-pos-deg") {
      port = lookupJSBExpression("fcs/aileron-pos-rad", path, false);
      port = addToUnit("aileron-pos-deg unit", Unit::degree(), port);

    } else if (propName == "fdm/jsbsim/fcs/rudder-pos-rad") {
      port = lookupJSBExpression("fcs/rudder-pos-deg", path, false);
      port = addFromUnit("rudder-pos-rad unit", Unit::degree(), port);

    } else if (propName == "fdm/jsbsim/fcs/rudder-pos-deg") {
      port = lookupJSBExpression("fcs/rudder-pos-rad", path, false);
      port = addToUnit("rudder-pos-deg unit", Unit::degree(), port);

    } else if (propName.find("fdm/jsbsim/fcs/mag-") == 0) {
      // Special absolute modules for fcs/mag-*
      // remove the 'mag-' substring here and use that as input for the
      // Abs block
      std::string name = "fcs/" + propName.substr(19);
      const Port* in = lookupJSBExpression(name, path);
      port = addAbsModel(propName.substr(15), in);

    }

    if (port /*&& port->isConnected()*/)
      registerExpression(propName, port);

    return port;
  }

  return 0;
}

const Port*
JSBSimReaderBase::createAndScheduleAeroProp(const std::string& propName,
                                            const NodePath& path)
{
  // This routine checks if the given propName is a aerodynamic reference
  // point property. If so, it schedules and registers a discrete input model.
  const Port* port = 0;
  if (propName == "fdm/jsbsim/velocities/vt-mps") {
    port = mAeroForce->getTrueAirSpeedPort();
  } else if (propName == "fdm/jsbsim/velocities/vt-fps") {
    port = mAeroForce->getTrueAirSpeedPort();
    port = addToUnit("True Speed fps", Unit::footPerSecond(), port);
  } else if (propName == "fdm/jsbsim/velocities/vt-kts") {
    port = mAeroForce->getTrueAirSpeedPort();
    port = addToUnit("True Speed kts", Unit::knots(), port);

  } else if (propName == "fdm/jsbsim/velocities/vc-fps") {
    port = mAeroForce->getCalibratedAirSpeedPort();
    port = addToUnit("Calibrated Speed fps", Unit::footPerSecond(), port);
  } else if (propName == "fdm/jsbsim/velocities/vc-kts") {
    port = mAeroForce->getCalibratedAirSpeedPort();
    port = addToUnit("Calibrated Speed kts", Unit::knots(), port);

  } else if (propName == "fdm/jsbsim/velocities/ve-fps") {
    port = mAeroForce->getEquivalentAirSpeedPort();
    port = addToUnit("Equivalent Speed fps", Unit::footPerSecond(), port);
  } else if (propName == "fdm/jsbsim/velocities/ve-kts") {
    port = mAeroForce->getEquivalentAirSpeedPort();
    port = addToUnit("Equivalent Speed kts", Unit::knots(), port);

  } else if (propName == "fdm/jsbsim/velocities/mach-norm" ||
             propName == "fdm/jsbsim/velocities/mach") {
    port = mAeroForce->getMachPort();

  } else if (propName == "fdm/jsbsim/velocities/p-rad_sec") {
    port = mAeroForce->getPPort();
  } else if (propName == "fdm/jsbsim/velocities/p-deg_sec") {
    port = mAeroForce->getPPort();
    port = addToUnit("P deg_sec", Unit::degree(), port);

  } else if (propName == "fdm/jsbsim/velocities/q-rad_sec") {
    port = mAeroForce->getQPort();
  } else if (propName == "fdm/jsbsim/velocities/q-deg_sec") {
    port = mAeroForce->getQPort();
    port = addToUnit("Q deg_sec", Unit::degree(), port);

  } else if (propName == "fdm/jsbsim/velocities/r-rad_sec") {
    port = mAeroForce->getRPort();
  } else if (propName == "fdm/jsbsim/velocities/r-deg_sec") {
    port = mAeroForce->getRPort();
    port = addToUnit("R deg_sec", Unit::degree(), port);

    /// FIXME: the aero stuff is yet missing!!!
  } else if (propName == "fdm/jsbsim/velocities/p-aero-rad_sec") {
    port = mAeroForce->getPPort();
  } else if (propName == "fdm/jsbsim/velocities/p-aero-deg_sec") {
    port = mAeroForce->getPPort();
    port = addToUnit("P-aero deg_sec", Unit::degree(), port);

  } else if (propName == "fdm/jsbsim/velocities/q-aero-rad_sec") {
    port = mAeroForce->getQPort();
  } else if (propName == "fdm/jsbsim/velocities/q-aero-deg_sec") {
    port = mAeroForce->getQPort();
    port = addToUnit("Q-aero deg_sec", Unit::degree(), port);

  } else if (propName == "fdm/jsbsim/velocities/r-aero-rad_sec") {
    port = mAeroForce->getRPort();
  } else if (propName == "fdm/jsbsim/velocities/r-aero-deg_sec") {
    port = mAeroForce->getRPort();
    port = addToUnit("R-aero deg_sec", Unit::degree(), port);

  } else if (propName == "fdm/jsbsim/velocities/u-aero-mps") {
    port = mAeroForce->getUPort();
  } else if (propName == "fdm/jsbsim/velocities/u-aero-fps") {
    port = mAeroForce->getUPort();
    port = addToUnit("U-aero fps", Unit::footPerSecond(), port);

  } else if (propName == "fdm/jsbsim/velocities/v-aero-mps") {
    port = mAeroForce->getVPort();
  } else if (propName == "fdm/jsbsim/velocities/v-aero-fps") {
    port = mAeroForce->getVPort();
    port = addToUnit("V-aero fps", Unit::footPerSecond(), port);

  } else if (propName == "fdm/jsbsim/velocities/w-aero-mps") {
    port = mAeroForce->getWPort();
  } else if (propName == "fdm/jsbsim/velocities/w-aero-fps") {
    port = mAeroForce->getWPort();
    port = addToUnit("W-aero fps", Unit::footPerSecond(), port);

  } else if (propName == "fdm/jsbsim/aero/qbar-pa") {
    port = mAeroForce->getDynamicPressurePort();
  } else if (propName == "fdm/jsbsim/aero/qbar-psf") {
    port = mAeroForce->getDynamicPressurePort();
    port = addToUnit("Dynamic pressure psf", Unit::lbfPerSquareFoot(), port);

  } else if (propName == "fdm/jsbsim/propulsion/tat-r") {
    port = mAeroForce->getTemperaturePort();
    port = addToUnit("Temperature Rankine", Unit::rankine(), port);
  } else if (propName == "fdm/jsbsim/propulsion/tat-f") {
    port = mAeroForce->getTemperaturePort();
    port = addToUnit("Degree Fahrenheit", Unit::degreeFarenheit(), port);
  } else if (propName == "fdm/jsbsim/propulsion/tat-c") {
    port = mAeroForce->getTemperaturePort();
    port = addToUnit("Degree Centigrade", Unit::degreeCelsius(), port);
    // Braindead: a tepmerature value in velocities ...
  } else if (propName == "fdm/jsbsim/velocities/tat-r") {
    port = lookupJSBExpression("propulsion/tat-r", path);
  } else if (propName == "fdm/jsbsim/velocities/tat-f") {
    port = lookupJSBExpression("propulsion/tat-f", path);
  } else if (propName == "fdm/jsbsim/velocities/tat-c") {
    port = lookupJSBExpression("propulsion/tat-c", path);

  } else if (propName == "fdm/jsbsim/propulsion/pt-pa") {
    // FIXME is this really static pressure in JSBSim
    port = mAeroForce->getStaticPressurePort();
  } else if (propName == "fdm/jsbsim/propulsion/pt-lbs_sqft") {
    // FIXME is this really static pressure in JSBSim
    port = mAeroForce->getStaticPressurePort();
    port = addToUnit("Static pressure psf", Unit::lbfPerSquareFoot(), port);
    // Braindead: a pressure value in velocities ...
  } else if (propName == "fdm/jsbsim/velocities/pt-pa") {
    port = lookupJSBExpression("propulsion/pt-pa", path);
  } else if (propName == "fdm/jsbsim/velocities/pt-lbs_sqft") {
    port = lookupJSBExpression("propulsion/pt-lbs_sqft", path);

  } else if (propName == "fdm/jsbsim/velocities/vg-mps") {
    port = mAeroForce->getGroundSpeedPort();
  } else if (propName == "fdm/jsbsim/velocities/vg-fps") {
    port = mAeroForce->getGroundSpeedPort();
    port = addToUnit("Ground Speed fps", Unit::footPerSecond(), port);

  } else if (propName == "fdm/jsbsim/aero/alpha-rad") {
    port = mAeroForce->getAlphaPort();
  } else if (propName == "fdm/jsbsim/aero/mag-alpha-rad") {
    port = mAeroForce->getAlphaPort();
    port = addAbsModel("Angle of attack mag", port);
  } else if (propName == "fdm/jsbsim/aero/alpha-deg") {
    port = mAeroForce->getAlphaPort();
    port = addToUnit("Angle of attack deg", Unit::degree(), port);
  } else if (propName == "fdm/jsbsim/aero/mag-alpha-deg") {
    port = lookupJSBExpression("aero/alpha-deg", path);
    port = addAbsModel("Angle of attack mag deg", port);

  } else if (propName == "fdm/jsbsim/aero/beta-rad") {
    port = mAeroForce->getBetaPort();
  } else if (propName == "fdm/jsbsim/aero/mag-beta-rad") {
    port = mAeroForce->getBetaPort();
    port = addAbsModel("Angle of sideslip mag", port);
  } else if (propName == "fdm/jsbsim/aero/beta-deg") {
    port = mAeroForce->getBetaPort();
    port = addToUnit("Angle of sideslip deg", Unit::degree(), port);
  } else if (propName == "fdm/jsbsim/aero/mag-beta-deg") {
    port = lookupJSBExpression("aero/beta-deg", path);
    port = addAbsModel("Angle of sideslip mag deg", port);

  } else if (propName == "fdm/jsbsim/aero/alphadot-rad_sec") {
    port = mAeroForce->getAlphaDotPort();
  } else if (propName == "fdm/jsbsim/aero/alphadot-deg_sec") {
    port = mAeroForce->getAlphaDotPort();
    port = addToUnit("Angle of attack deriv deg_sec", Unit::degree(), port);

  } else if (propName == "fdm/jsbsim/aero/betadot-rad_sec") {
    port = mAeroForce->getBetaDotPort();
  } else if (propName == "fdm/jsbsim/aero/betadot-deg_sec") {
    port = mAeroForce->getBetaDotPort();
    port = addToUnit("Angle of sideslip deriv deg_sec", Unit::degree(), port);

  } else if (propName == "fdm/jsbsim/metrics/bw-ft") {
    port = mAeroForce->getWingSpanPort();
    port = addToUnit("Wingspan ft", Unit::foot(), port);

  } else if (propName == "fdm/jsbsim/metrics/Sw-sqft") {
    port = mAeroForce->getWingAreaPort();
    port = addToUnit("Wingarea ft2", Unit::squareFoot(), port);

  } else if (propName == "fdm/jsbsim/metrics/cbarw-ft") {
    port = mAeroForce->getChordPort();
    port = addToUnit("Chord ft", Unit::foot(), port);

  } else if (propName == "fdm/jsbsim/aero/bi2vel") {
    port = mAeroForce->getWingSpanOver2SpeedPort();

  } else if (propName == "fdm/jsbsim/aero/ci2vel") {
    port = mAeroForce->getChordOver2SpeedPort();

  } else if (propName == "fdm/jsbsim/aero/h_b-cg-ft") {
    port = mAeroForce->getHOverWingSpanPort();

  } else if (propName == "fdm/jsbsim/aero/h_b-mac-ft") {
    /// Hmmm, FIXME
    port = lookupJSBExpression("aero/h_b-cg-ft", path);

  }

  if (port /*&& port->isConnected()*/)
    registerExpression(propName, port);
  
  return port;
}

const Port*
JSBSimReaderBase::addInputModel(const std::string& name,
                                const std::string& propName, real_type gain)
{
  Input* input = new Input(name + " Input");
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
    std::cerr << "Could not add output model \"" << name << "\"" << std::endl;
    return;
  }
  Output* output = new Output(name + " Output");
  group->addChild(output);
  output->setOutputName(propName);
  output->setOutputGain(gain);
  group->connect(out, output->getPort("input"));
}

const Port*
JSBSimReaderBase::addInverterModel(const std::string& name, const Port* in)
{
  SharedPtr<Group> group = getGroup(in);
  if (!group) {
    std::cerr << "Could not add inverter model \"" << name << "\"" << std::endl;
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
JSBSimReaderBase::addAbsModel(const std::string& name, const Port* in)
{
  SharedPtr<Group> group = getGroup(in);
  if (!group) {
    std::cerr << "Could not add inverter model \"" << name << "\"" << std::endl;
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
    std::cerr << "Could not add inverter model \"" << name << "\"" << std::endl;
    return 0;
  }
  UnitConversion* unitConv
    = new UnitConversion(name, UnitConversion::BaseUnitToUnit, u);
  group->addChild(unitConv);
  if (!group->connect(in, unitConv->getPort("input")))
    return 0;
  return unitConv->getPort("output");
}

const Port*
JSBSimReaderBase::addFromUnit(const std::string& name, Unit u, const Port* in)
{
  SharedPtr<Group> group = getGroup(in);
  if (!group) {
    std::cerr << "Could not add inverter model \"" << name << "\"" << std::endl;
    return 0;
  }
  UnitConversion* unitConv
    = new UnitConversion(name, UnitConversion::UnitToBaseUnit, u);
  group->addChild(unitConv);
  if (!group->connect(in, unitConv->getPort("input")))
    return 0;
  return unitConv->getPort("output");
}

SharedPtr<Group>
JSBSimReaderBase::getGroup(const Port* in)
{
  if (!in) {
    std::cerr << "Could not find model group for input port: "
      "no port given!" << std::endl;
    return 0;
  }
  SharedPtr<const Node> node = in->getNode();
  if (!node) {
    std::cerr << "Could not find model group for input port: "
      "port does not belong to a Node!" << std::endl;
    return 0;
  }
  SharedPtr<const Node> parent = node->getParent(0).lock();
  SharedPtr<Group> group = const_cast<Group*>(dynamic_cast<const Group*>(parent.get()));
  if (!group) {
    std::cerr << "Could not find model group for input port: "
      "model has no parent!" << std::endl;
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
JSBSimReaderBase::getTablePrelookup(const std::string& name, const Port* in,
                                    const BreakPointVector& tl)
{
  if (!in)
    return 0;

  // First check if we already have a table lookup for this port/brakepoint
  // combination. If so return that output port
  std::vector<BreakPointLookupEntry>::iterator it;
  for (it = mBreakPointVectors.begin(); it != mBreakPointVectors.end(); ++it) {
    if (tl == it->lookup->getBreakPointVector() && in == it->inputConnection)
      return it->lookup->getPort("output");
  }

  // No sharable table lookup found, we need to create a new one
  BreakPointLookup* tablePreLookup
    = new BreakPointLookup(name + " Table Prelookup");
  addMultiBodyModel(tablePreLookup);
  tablePreLookup->setBreakPointVector(tl);
  mTopLevelGroup->connect(in, tablePreLookup->getInputPort(0));
  mBreakPointVectors.push_back(BreakPointLookupEntry(in, tablePreLookup));
  return tablePreLookup->getOutputPort();
}

} // namespace OpenFDM
