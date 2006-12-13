/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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

#include <OpenFDM/AeroForce.h>
#include <OpenFDM/Bias.h>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/DeadBand.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/ExternalForceModel.h>
#include <OpenFDM/TransferFunction.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Input.h>
#include <OpenFDM/Output.h>
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

#include <OpenFDM/XML/XMLReader.h>
#include <OpenFDM/XML/ContentHandler.h>
#include <OpenFDM/XML/ErrorHandler.h>
#include <OpenFDM/XML/Attributes.h>

#include <OpenFDM/XML/EasyXMLReader.h> // FIXME

#include "JSBSimAerosurfaceScale.h"
#include "JSBSimKinemat.h"
#include "JSBSimScheduledGain.h"

namespace OpenFDM {

JSBSimReaderBase::JSBSimReaderBase(void)
{
}

JSBSimReaderBase::~JSBSimReaderBase(void)
{
}

void
JSBSimReaderBase::reset(void)
{
  // Throw away any possibly loaded vehicle
  mVehicle = 0;
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
  SharedPtr<XML::XMLReader> reader = new XML::EasyXMLReader;
  
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

PortProvider*
JSBSimReaderBase::lookupJSBExpression(const std::string& name,
                                      const Model::Path& path,
                                      bool recheckAeroProp)
{
  // Convert to something being able to look up
  std::string propName = propNameFromJSBSim(name);
  
  PortProvider* port = 0;
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
                                       PortAcceptor* pa, bool recheckAeroProp)
{
  SharedPtr<Model> model = pa->getModel().lock();
  if (!model)
    return false;
  Model::Path path = model->getPath();
  PortProvider* pp = lookupJSBExpression(name, path, recheckAeroProp);
  return Port::Success == Connection::connect(pp, pa);
}

void
JSBSimReaderBase::registerExpression(const std::string& name, PortProvider* port)
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
JSBSimReaderBase::registerJSBExpression(const std::string& name, PortProvider* port)
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

PortProvider*
JSBSimReaderBase::createAndScheduleInput(const std::string& propName,
                                         const Model::Path& path)
{
  // This routine checks if the given propName is a special JSBSim
  // input property. If so, it schedules and registers a discrete input model.
  // If the propName points directly into the controls directory,
  // schedule an input
  if (propName.substr(0, 9) == "controls/") {
//     std::string inputName = propName.substr(propName.rfind('/'));
    std::string inputName = propName;
    return addInputModel("Control " + inputName, propName);
  } else {
    PortProvider* port = 0;
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

    } else if (propName.substr(0, 28) == "fdm/jsbsim/fcs/steer-pos-deg") {
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


    } else if (propName.substr(0, 32) == "fdm/jsbsim/fcs/throttle-cmd-norm") {
      std::string control = "controls/engines/engine" +
        propName.substr(32) + "/throttle";
      port = addInputModel("Throttle Input " + propName.substr(32),
                           control);

    } else if (propName.substr(0, 32) == "fdm/jsbsim/fcs/throttle-pos-norm") {
      std::string cmd = "fcs/throttle-cmd-norm" + propName.substr(32);
      return lookupJSBExpression(cmd, path);


    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/mixture-cmd-norm") {
      std::string control = "controls/engines/engine" + 
        propName.substr(31) + "/mixture";
      port = addInputModel("Mixture Input " + propName.substr(32, 1), control);

    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/mixture-pos-norm") {
      std::string cmd = "fcs/mixture-cmd-norm" + propName.substr(31);
      return lookupJSBExpression(cmd, path);


    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/advance-cmd-norm") {
      std::string control = "controls/engines/engine" + 
        propName.substr(31) + "/propeller-pitch";
      port = addInputModel("Propeller Pitch Input " + propName.substr(32, 1),
                           control);

    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/advance-pos-norm") {
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

      PortProvider* pilotBr = addInputModel("Right Brake",
                                    "controls/gear/brake-right");
      Connection::connect(pilotBr, maxModel->getInputPort(0));

      PortProvider* copilotBr = addInputModel("Right Copilot Brake",
                                      "controls/gear/copilot-brake-right");
      Connection::connect(copilotBr, maxModel->getInputPort(1));

      PortProvider* parkBr = lookupJSBExpression("/controls/gear/brake-parking", maxModel->getPath());
      Connection::connect(parkBr, maxModel->getInputPort(2));

      port = maxModel->getOutputPort(0);

    } else if (propName == "fdm/jsbsim/gear/left-brake-pos-norm") {
      MaxModel* maxModel = new MaxModel("Left Brake Max");
      addFCSModel(maxModel);
      maxModel->setNumMaxInputs(3);

      PortProvider* pilotBr = addInputModel("Left Brake",
                                    "controls/gear/brake-left");
      Connection::connect(pilotBr, maxModel->getInputPort(0));

      PortProvider* copilotBr = addInputModel("Left Copilot Brake",
                                      "controls/gear/copilot-brake-left");
      Connection::connect(copilotBr, maxModel->getInputPort(1));
      
      PortProvider* parkBr = lookupJSBExpression("/controls/gear/brake-parking", maxModel->getPath());
      Connection::connect(parkBr, maxModel->getInputPort(2));

      port = maxModel->getOutputPort(0);

    } else if (propName == "fdm/jsbsim/fcs/elevator-pos-rad") {
      port = lookupJSBExpression("fcs/elevator-pos-deg", path, false);
      port = addFromUnit("elevator-pos-rad unit", uDegree, port);

    } else if (propName == "fdm/jsbsim/fcs/elevator-pos-deg") {
      port = lookupJSBExpression("fcs/elevator-pos-rad", path, false);
      port = addToUnit("elevator-pos-deg unit", uDegree, port);

    } else if (propName == "fdm/jsbsim/fcs/left-aileron-pos-rad") {
      port = lookupJSBExpression("fcs/left-aileron-pos-deg", path, false);
      port = addFromUnit("left-aileron-pos-rad unit", uDegree, port);

    } else if (propName == "fdm/jsbsim/fcs/left-aileron-pos-deg") {
      port = lookupJSBExpression("fcs/left-aileron-pos-rad", path, false);
      port = addToUnit("left-aileron-pos-deg unit", uDegree, port);

    } else if (propName == "fdm/jsbsim/fcs/right-aileron-pos-rad") {
      port = lookupJSBExpression("fcs/right-aileron-pos-deg", path, false);
      port = addFromUnit("right-aileron-pos-rad unit", uDegree, port);

    } else if (propName == "fdm/jsbsim/fcs/right-aileron-pos-deg") {
      port = lookupJSBExpression("fcs/right-aileron-pos-rad", path, false);
      port = addToUnit("right-aileron-pos-deg unit", uDegree, port);

    } else if (propName == "fdm/jsbsim/fcs/aileron-pos-rad") {
      port = lookupJSBExpression("fcs/aileron-pos-deg", path, false);
      port = addFromUnit("aileron-pos-rad unit", uDegree, port);

    } else if (propName == "fdm/jsbsim/fcs/aileron-pos-deg") {
      port = lookupJSBExpression("fcs/aileron-pos-rad", path, false);
      port = addToUnit("aileron-pos-deg unit", uDegree, port);

    } else if (propName == "fdm/jsbsim/fcs/rudder-pos-rad") {
      port = lookupJSBExpression("fcs/rudder-pos-deg", path, false);
      port = addFromUnit("rudder-pos-rad unit", uDegree, port);

    } else if (propName == "fdm/jsbsim/fcs/rudder-pos-deg") {
      port = lookupJSBExpression("fcs/rudder-pos-rad", path, false);
      port = addToUnit("rudder-pos-deg unit", uDegree, port);

    } else if (propName.substr(0, 19) == "fdm/jsbsim/fcs/mag-") {
      // Special absolute modules for fcs/mag-*
      // remove the 'mag-' substring here and use that as input for the
      // Abs block
      std::string name = "fcs/" + propName.substr(19);
      PortProvider* in = lookupJSBExpression(name, path);
      port = addAbsModel(propName.substr(15), in);

    }

    if (port /*&& port->isConnected()*/)
      registerExpression(propName, port);

    return port;
  }

  return 0;
}

PortProvider*
JSBSimReaderBase::createAndScheduleAeroProp(const std::string& propName,
                                            const Model::Path& path)
{
  // This routine checks if the given propName is a aerodynamic reference
  // point property. If so, it schedules and registers a discrete input model.
  PortProvider* port = 0;
  if (propName == "fdm/jsbsim/velocities/vt-mps") {
    port = mAeroForce->getOutputPort("trueSpeed");
  } else if (propName == "fdm/jsbsim/velocities/vt-fps") {
    port = mAeroForce->getOutputPort("trueSpeed");
    port = addToUnit("True Speed fps", uFeetPSecond, port);
  } else if (propName == "fdm/jsbsim/velocities/vt-kts") {
    port = mAeroForce->getOutputPort("trueSpeed");
    port = addToUnit("True Speed kts", uKnots, port);

  } else if (propName == "fdm/jsbsim/velocities/vc-fps") {
    port = mAeroForce->getOutputPort("calibratedAirSpeed");
    port = addToUnit("Calibrated Speed fps", uFeetPSecond, port);
  } else if (propName == "fdm/jsbsim/velocities/vc-kts") {
    port = mAeroForce->getOutputPort("calibratedAirSpeed");
    port = addToUnit("Calibrated Speed kts", uKnots, port);

  } else if (propName == "fdm/jsbsim/velocities/ve-fps") {
    port = mAeroForce->getOutputPort("calibratedAirSpeed");
    port = addToUnit("Equivalent Speed fps", uFeetPSecond, port);
  } else if (propName == "fdm/jsbsim/velocities/ve-kts") {
    port = mAeroForce->getOutputPort("calibratedAirSpeed");
    port = addToUnit("Equivalent Speed kts", uKnots, port);

  } else if (propName == "fdm/jsbsim/velocities/mach-norm" ||
             propName == "fdm/jsbsim/velocities/mach") {
    port = mAeroForce->getOutputPort("machNumber");

  } else if (propName == "fdm/jsbsim/velocities/p-rad_sec") {
    port = mAeroForce->getOutputPort("p");
  } else if (propName == "fdm/jsbsim/velocities/p-deg_sec") {
    port = mAeroForce->getOutputPort("p");
    port = addToUnit("P deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/q-rad_sec") {
    port = mAeroForce->getOutputPort("q");
  } else if (propName == "fdm/jsbsim/velocities/q-deg_sec") {
    port = mAeroForce->getOutputPort("q");
    port = addToUnit("Q deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/r-rad_sec") {
    port = mAeroForce->getOutputPort("r");
  } else if (propName == "fdm/jsbsim/velocities/r-deg_sec") {
    port = mAeroForce->getOutputPort("r");
    port = addToUnit("R deg_sec", uDegree, port);

    /// FIXME: the aero stuff is yet missing!!!
  } else if (propName == "fdm/jsbsim/velocities/p-aero-rad_sec") {
    port = mAeroForce->getOutputPort("p");
  } else if (propName == "fdm/jsbsim/velocities/p-aero-deg_sec") {
    port = mAeroForce->getOutputPort("p");
    port = addToUnit("P-aero deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/q-aero-rad_sec") {
    port = mAeroForce->getOutputPort("q");
  } else if (propName == "fdm/jsbsim/velocities/q-aero-deg_sec") {
    port = mAeroForce->getOutputPort("q");
    port = addToUnit("Q-aero deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/r-aero-rad_sec") {
    port = mAeroForce->getOutputPort("r");
  } else if (propName == "fdm/jsbsim/velocities/r-aero-deg_sec") {
    port = mAeroForce->getOutputPort("r");
    port = addToUnit("R-aero deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/u-aero-mps") {
    port = mAeroForce->getOutputPort("u");
  } else if (propName == "fdm/jsbsim/velocities/u-aero-fps") {
    port = mAeroForce->getOutputPort("u");
    port = addToUnit("U-aero fps", uFeetPSecond, port);

  } else if (propName == "fdm/jsbsim/velocities/v-aero-mps") {
    port = mAeroForce->getOutputPort("v");
  } else if (propName == "fdm/jsbsim/velocities/v-aero-fps") {
    port = mAeroForce->getOutputPort("v");
    port = addToUnit("V-aero fps", uFeetPSecond, port);

  } else if (propName == "fdm/jsbsim/velocities/w-aero-mps") {
    port = mAeroForce->getOutputPort("w");
  } else if (propName == "fdm/jsbsim/velocities/w-aero-fps") {
    port = mAeroForce->getOutputPort("w");
    port = addToUnit("W-aero fps", uFeetPSecond, port);

  } else if (propName == "fdm/jsbsim/aero/qbar-pa") {
    port = mAeroForce->getOutputPort("dynamicPressure");
  } else if (propName == "fdm/jsbsim/aero/qbar-psf") {
    port = mAeroForce->getOutputPort("dynamicPressure");
    port = addToUnit("Dynamic pressure psf", uPoundPFt2, port);

  } else if (propName == "fdm/jsbsim/propulsion/tat-r") {
    port = mAeroForce->getOutputPort("temperature");
    port = addToUnit("Temperature Rankine", uRankine, port);
  } else if (propName == "fdm/jsbsim/propulsion/tat-f") {
    port = mAeroForce->getOutputPort("temperature");
    port = addToUnit("Degree Fahrenheit", uFahrenheit, port);
  } else if (propName == "fdm/jsbsim/propulsion/tat-c") {
    port = mAeroForce->getOutputPort("temperature");
    port = addToUnit("Degree Centigrade", uDegC, port);
    // Braindead: a tepmerature value in velocities ...
  } else if (propName == "fdm/jsbsim/velocities/tat-r") {
    port = lookupJSBExpression("propulsion/tat-r", path);
  } else if (propName == "fdm/jsbsim/velocities/tat-f") {
    port = lookupJSBExpression("propulsion/tat-f", path);
  } else if (propName == "fdm/jsbsim/velocities/tat-c") {
    port = lookupJSBExpression("propulsion/tat-c", path);

  } else if (propName == "fdm/jsbsim/propulsion/pt-pa") {
    port = mAeroForce->getOutputPort("pressure");
  } else if (propName == "fdm/jsbsim/propulsion/pt-lbs_sqft") {
    port = mAeroForce->getOutputPort("pressure");
    port = addToUnit("Static pressure psf", uPoundPFt2, port);
    // Braindead: a pressure value in velocities ...
  } else if (propName == "fdm/jsbsim/velocities/pt-pa") {
    port = lookupJSBExpression("propulsion/pt-pa", path);
  } else if (propName == "fdm/jsbsim/velocities/pt-lbs_sqft") {
    port = lookupJSBExpression("propulsion/pt-lbs_sqft", path);

  } else if (propName == "fdm/jsbsim/aero/alpha-rad") {
    port = mAeroForce->getOutputPort("alpha");
  } else if (propName == "fdm/jsbsim/aero/mag-alpha-rad") {
    port = mAeroForce->getOutputPort("alpha");
    port = addAbsModel("Angle of attack mag", port);
  } else if (propName == "fdm/jsbsim/aero/alpha-deg") {
    port = mAeroForce->getOutputPort("alpha");
    port = addToUnit("Angle of attack deg", uDegree, port);
  } else if (propName == "fdm/jsbsim/aero/mag-alpha-deg") {
    port = lookupJSBExpression("aero/alpha-deg", path);
    port = addAbsModel("Angle of attack mag deg", port);

  } else if (propName == "fdm/jsbsim/aero/beta-rad") {
    port = mAeroForce->getOutputPort("beta");
  } else if (propName == "fdm/jsbsim/aero/mag-beta-rad") {
    port = mAeroForce->getOutputPort("beta");
    port = addAbsModel("Angle of sideslip mag", port);
  } else if (propName == "fdm/jsbsim/aero/beta-deg") {
    port = mAeroForce->getOutputPort("beta");
    port = addToUnit("Angle of sideslip deg", uDegree, port);
  } else if (propName == "fdm/jsbsim/aero/mag-beta-deg") {
    port = lookupJSBExpression("aero/beta-deg", path);
    port = addAbsModel("Angle of sideslip mag deg", port);

  } else if (propName == "fdm/jsbsim/aero/alphadot-rad_sec") {
    port = mAeroForce->getOutputPort("alphaDot");
  } else if (propName == "fdm/jsbsim/aero/alphadot-deg_sec") {
    port = mAeroForce->getOutputPort("alphaDot");
    port = addToUnit("Angle of attack deriv deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/aero/betadot-rad_sec") {
    port = mAeroForce->getOutputPort("betaDot");
  } else if (propName == "fdm/jsbsim/aero/betadot-deg_sec") {
    port = mAeroForce->getOutputPort("betaDot");
    port = addToUnit("Angle of sideslip deriv deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/metrics/bw-ft") {
    /// FIXME, just schedule a constant block for that??
    port = mAeroForce->getOutputPort("wingSpan");
    port = addToUnit("Wingspan ft", uFoot, port);

  } else if (propName == "fdm/jsbsim/metrics/Sw-sqft") {
    /// FIXME, just schedule a constant block for that??
    port = mAeroForce->getOutputPort("wingArea");
    port = addToUnit("Wingarea ft2", uFoot2, port);

  } else if (propName == "fdm/jsbsim/metrics/cbarw-ft") {
    /// FIXME, just schedule a constant block for that??
    port = mAeroForce->getOutputPort("coord");
    port = addToUnit("Coord ft", uFoot, port);

  } else if (propName == "fdm/jsbsim/aero/bi2vel") {
    port = mAeroForce->getOutputPort("wingSpanOver2Speed");

  } else if (propName == "fdm/jsbsim/aero/ci2vel") {
    port = mAeroForce->getOutputPort("coordOver2Speed");

  } else if (propName == "fdm/jsbsim/aero/h_b-cg-ft") {
    port = mAeroForce->getOutputPort("hOverWingSpan");

  } else if (propName == "fdm/jsbsim/aero/h_b-mac-ft") {
    /// Hmmm, FIXME
    port = lookupJSBExpression("aero/h_b-cg-ft", path);

  }

  if (port /*&& port->isConnected()*/)
    registerExpression(propName, port);
  
  return port;
}

PortProvider*
JSBSimReaderBase::addInputModel(const std::string& name,
                                const std::string& propName, real_type gain)
{
  Input* input = new Input(name + " Input");
  input->setInputName(propName);
  input->setInputGain(gain);
  addFCSModel(input);
  PortProvider* port = input->getOutputPort(0);
  registerExpression(propName, port);
  return port;
}

void
JSBSimReaderBase::addOutputModel(PortProvider* out,
                                 const std::string& name,
                                 const std::string& propName, real_type gain)
{
  SharedPtr<ModelGroup> modelGroup = getModelGroup(out);
  if (!modelGroup) {
    std::cerr << "Could not add output model " << name << std::endl;
    return;
  }
  Output* output = new Output(name + " Output");
  modelGroup->addModel(output, true);
  output->setOutputName(propName);
  output->setOutputGain(gain);
  Connection::connect(out, output->getInputPort(0));
}

PortProvider*
JSBSimReaderBase::addInverterModel(const std::string& name, PortProvider* in)
{
  SharedPtr<ModelGroup> modelGroup = getModelGroup(in);
  if (!modelGroup) {
    std::cerr << "Could not add inverter model " << name << std::endl;
    return 0;
  }
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Inverter", UnaryFunctionModel::Minus);
  modelGroup->addModel(unary, true);
  if (Port::Success != Connection::connect(in, unary->getInputPort(0)))
    return 0;
  return unary->getOutputPort(0);
}

PortProvider*
JSBSimReaderBase::addAbsModel(const std::string& name, PortProvider* in)
{
  SharedPtr<ModelGroup> modelGroup = getModelGroup(in);
  if (!modelGroup) {
    std::cerr << "Could not add inverter model " << name << std::endl;
    return 0;
  }
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Abs", UnaryFunctionModel::Abs);
  modelGroup->addModel(unary, true);
  if (Port::Success != Connection::connect(in, unary->getInputPort(0)))
    return 0;
  return unary->getOutputPort(0);
}

PortProvider*
JSBSimReaderBase::addConstModel(const std::string& name, real_type value,
                                const Model::Path& path)
{
  Matrix m(1, 1);
  m(1, 1) = 0;
  ConstModel* cModel = new ConstModel(name, m);
  if (path.empty())
    addFCSModel(cModel);
  else
    path.back()->addModel(cModel, true);
  return cModel->getOutputPort(0);
}

PortProvider*
JSBSimReaderBase::addToUnit(const std::string& name, Unit u, PortProvider* in)
{
  SharedPtr<ModelGroup> modelGroup = getModelGroup(in);
  if (!modelGroup) {
    std::cerr << "Could not add inverter model " << name << std::endl;
    return 0;
  }
  UnitConversionModel* unitConv
    = new UnitConversionModel(name, UnitConversionModel::SiToUnit, u);
  modelGroup->addModel(unitConv, true);
  if (Port::Success != Connection::connect(in, unitConv->getInputPort(0)))
    return 0;
  return unitConv->getOutputPort(0);
}

PortProvider*
JSBSimReaderBase::addFromUnit(const std::string& name, Unit u, PortProvider* in)
{
  SharedPtr<ModelGroup> modelGroup = getModelGroup(in);
  if (!modelGroup) {
    std::cerr << "Could not add inverter model " << name << std::endl;
    return 0;
  }
  UnitConversionModel* unitConv
    = new UnitConversionModel(name, UnitConversionModel::UnitToSi, u);
  modelGroup->addModel(unitConv, true);
  if (Port::Success != Connection::connect(in, unitConv->getInputPort(0)))
    return 0;
  return unitConv->getOutputPort(0);
}

SharedPtr<ModelGroup>
JSBSimReaderBase::getModelGroup(PortProvider* in)
{
  if (!in) {
    std::cerr << "Could not find model group for input port" << std::endl;
    return 0;
  }
  SharedPtr<Model> model = in->getModel().lock();
  if (!model) {
    std::cerr << "Could not find model group for input port" << std::endl;
    return 0;
  }
  SharedPtr<ModelGroup> modelGroup = model->getParent();
  if (!modelGroup) {
    std::cerr << "Could not find model group for input port" << std::endl;
    return 0;
  }
  return modelGroup;
}

void
JSBSimReaderBase::addFCSModel(Model* model)
{
  // FIXME
  mVehicle->getModelGroup()->addModel(model, true);
}

PortProvider*
JSBSimReaderBase::addMultiBodyConstModel(const std::string& name, real_type value)
{
  Matrix m(1, 1);
  m(1, 1) = value;
  ConstModel* cModel = new ConstModel(name, m);
  addMultiBodyModel(cModel);
  return cModel->getOutputPort(0);
}

void
JSBSimReaderBase::addMultiBodyModel(Model* model)
{
  // FIXME
  mVehicle->getMultiBodySystem()->addModel(model, true);
}

PortProvider*
JSBSimReaderBase::getTablePrelookup(const std::string& name, PortProvider* in,
                                    const TableLookup& tl)
{
  if (!in)
    return 0;
  NumericPortProvider* nin = dynamic_cast<NumericPortProvider*>(in);
  if (!nin)
    return 0;

  // First check if we already have a table lookup for this port/brakepoint
  // combination. If so return that output port
  std::vector<SharedPtr<TablePreLookup> >::iterator it;
  for (it = mTableLookups.begin(); it != mTableLookups.end(); ++it) {
    if (tl == (*it)->getTableLookup() &&
        nin->getPortInterface() == (*it)->getInputPort(0)->getPortInterface())
      return (*it)->getOutputPort(0);
  }

  // No sharable table lookup found, we need to create a new one
  TablePreLookup* tablePreLookup
    = new TablePreLookup(name + " Table Prelookup");
  addMultiBodyModel(tablePreLookup);
  tablePreLookup->setTableLookup(tl);
  Connection::connect(in, tablePreLookup->getInputPort(0));
  mTableLookups.push_back(tablePreLookup);
  return tablePreLookup->getOutputPort(0);
}

} // namespace OpenFDM
