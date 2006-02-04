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

#include <OpenFDM/XML/XMLReader.h>
#include <OpenFDM/XML/ContentHandler.h>
#include <OpenFDM/XML/ErrorHandler.h>
#include <OpenFDM/XML/Attributes.h>

#include <OpenFDM/XML/EasyXMLReader.h> // FIXME

#include "JSBSimAerosurfaceScale.h"
#include "JSBSimKinemat.h"
#include "JSBSimScheduledGain.h"

#include "JSBSimReaderBase.h"

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

Port*
JSBSimReaderBase::lookupJSBExpression(const std::string& name)
{
  // Convert to something being able to look up
  std::string propName = propNameFromJSBSim(name);
  
  Port* port;
  if (mExpressionTable.count(propName) <= 0) {
    // Not yet available, so look and see if it is an input
    port = createAndScheduleAeroProp(propName);

    if (!port || !port->isConnected()) {
      // Not yet available, so look and see if it is an input
      port = createAndScheduleInput(propName);
      
      // Ok, still not available, create a constant zero thing and bail out ...
      if (!port || !port->isConnected()) {
        std::cerr << "Creating expression \"" << propName << "\"" << std::endl;
        
        return addConstModel(propName + " constant", 0);
      }
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
JSBSimReaderBase::registerExpression(const std::string& name, Port* port)
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
JSBSimReaderBase::registerJSBExpression(const std::string& name, Port* port)
{
  if (name.empty())
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
JSBSimReaderBase::createAndScheduleInput(const std::string& propName)
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
    Port* port = 0;
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
      // FIXME is inverted in JSBSim ...
      port = addInputModel("Rudder",
                           "controls/flight/rudder");

    } else if (propName == "fdm/jsbsim/fcs/yaw-trim-cmd-norm") {
      // FIXME also with a minus
      port = addInputModel("Yaw Trim",
                           "controls/flight/rudder-trim");

    } else if (propName == "fdm/jsbsim/fcs/steer-cmd-norm") {
      // FIXME is seperate in flightgear ???
      // port = addInputModel("Steering", "controls/gear/steering");
      port = addInputModel("Steering",
                           "controls/flight/rudder");

    } else if (propName.substr(0, 28) == "fdm/jsbsim/fcs/steer-pos-deg") {
      return lookupJSBExpression("fcs/steer-cmd-norm");

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
      port = addInputModel("Gear Retract",
                           "controls/gear/gear-down");

    } else if (propName == "fdm/jsbsim/gear/gear-pos-norm") {
      return lookupJSBExpression("gear/gear-cmd-norm");

    } else if (propName == "controls/gear/brake-parking") {
      port = addInputModel("Parking Brake",
                           "controls/gear/brake-parking");

    } else if (propName == "fdm/jsbsim/gear/right-brake-pos-norm") {
      MaxModel* maxModel = new MaxModel("Right Brake Max");
      maxModel->setNumMaxInputs(3);

      Port* pilotBr = addInputModel("Right Brake",
                                    "controls/gear/brake-right");
      maxModel->getInputPort(0)->connect(pilotBr);

      Port* copilotBr = addInputModel("Right Copilot Brake",
                                      "controls/gear/copilot-brake-right");
      maxModel->getInputPort(1)->connect(copilotBr);

      Port* parkBr = lookupJSBExpression("/controls/gear/brake-parking");
      maxModel->getInputPort(2)->connect(parkBr);

      addFCSModel(maxModel);
      port = maxModel->getOutputPort(0);

    } else if (propName == "fdm/jsbsim/gear/left-brake-pos-norm") {
      MaxModel* maxModel = new MaxModel("Left Brake Max");
      maxModel->setNumMaxInputs(3);

      Port* pilotBr = addInputModel("Left Brake",
                                    "controls/gear/brake-left");
      maxModel->getInputPort(0)->connect(pilotBr);

      Port* copilotBr = addInputModel("Left Copilot Brake",
                                      "controls/gear/copilot-brake-left");
      maxModel->getInputPort(1)->connect(copilotBr);
      
      Port* parkBr = lookupJSBExpression("/controls/gear/brake-parking");
      maxModel->getInputPort(2)->connect(parkBr);

      addFCSModel(maxModel);
      port = maxModel->getOutputPort(0);

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
JSBSimReaderBase::createAndScheduleAeroProp(const std::string& propName)
{
  // This routine checks if the given propName is a aerodynamic reference
  // point property. If so, it schedules and registers a discrete input model.
  Port* port = 0;
  if (propName == "fdm/jsbsim/velocities/vt-mps") {
    port = mAeroForce->getOutputPort("trueSpeed");
  } else if (propName == "fdm/jsbsim/velocities/vt-fps") {
    port = mAeroForce->getOutputPort("trueSpeed");
    port = addMultiBodyToUnit("True Speed fps", uFeetPSecond, port);
  } else if (propName == "fdm/jsbsim/velocities/vt-kts") {
    port = mAeroForce->getOutputPort("trueSpeed");
    port = addMultiBodyToUnit("True Speed kts", uKnots, port);

  } else if (propName == "fdm/jsbsim/velocities/mach-norm" ||
             propName == "fdm/jsbsim/velocities/mach") {
    port = mAeroForce->getOutputPort("machNumber");

  } else if (propName == "fdm/jsbsim/velocities/p-rad_sec") {
    port = mAeroForce->getOutputPort("p");
  } else if (propName == "fdm/jsbsim/velocities/p-deg_sec") {
    port = mAeroForce->getOutputPort("p");
    port = addMultiBodyToUnit("P deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/q-rad_sec") {
    port = mAeroForce->getOutputPort("q");
  } else if (propName == "fdm/jsbsim/velocities/q-deg_sec") {
    port = mAeroForce->getOutputPort("q");
    port = addMultiBodyToUnit("Q deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/r-rad_sec") {
    port = mAeroForce->getOutputPort("r");
  } else if (propName == "fdm/jsbsim/velocities/r-deg_sec") {
    port = mAeroForce->getOutputPort("r");
    port = addMultiBodyToUnit("R deg_sec", uDegree, port);

    /// FIXME: the aero stuff is yet missing!!!
  } else if (propName == "fdm/jsbsim/velocities/p-aero-rad_sec") {
    port = mAeroForce->getOutputPort("p");
  } else if (propName == "fdm/jsbsim/velocities/p-aero-deg_sec") {
    port = mAeroForce->getOutputPort("p");
    port = addMultiBodyToUnit("P-aero deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/q-aero-rad_sec") {
    port = mAeroForce->getOutputPort("q");
  } else if (propName == "fdm/jsbsim/velocities/q-aero-deg_sec") {
    port = mAeroForce->getOutputPort("q");
    port = addMultiBodyToUnit("Q-aero deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/r-aero-rad_sec") {
    port = mAeroForce->getOutputPort("r");
  } else if (propName == "fdm/jsbsim/velocities/r-aero-deg_sec") {
    port = mAeroForce->getOutputPort("r");
    port = addMultiBodyToUnit("R-aero deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/velocities/u-aero-mps") {
    port = mAeroForce->getOutputPort("u");
  } else if (propName == "fdm/jsbsim/velocities/u-aero-fps") {
    port = mAeroForce->getOutputPort("u");
    port = addMultiBodyToUnit("U-aero fps", uFeetPSecond, port);

  } else if (propName == "fdm/jsbsim/velocities/v-aero-mps") {
    port = mAeroForce->getOutputPort("v");
  } else if (propName == "fdm/jsbsim/velocities/v-aero-fps") {
    port = mAeroForce->getOutputPort("v");
    port = addMultiBodyToUnit("V-aero fps", uFeetPSecond, port);

  } else if (propName == "fdm/jsbsim/velocities/w-aero-mps") {
    port = mAeroForce->getOutputPort("w");
  } else if (propName == "fdm/jsbsim/velocities/w-aero-fps") {
    port = mAeroForce->getOutputPort("w");
    port = addMultiBodyToUnit("W-aero fps", uFeetPSecond, port);

  } else if (propName == "fdm/jsbsim/aero/qbar-pa") {
    port = mAeroForce->getOutputPort("dynamicPressure");
  } else if (propName == "fdm/jsbsim/aero/qbar-psf") {
    port = mAeroForce->getOutputPort("dynamicPressure");
    port = addMultiBodyToUnit("Dynamic pressure psf", uPoundPFt2, port);

  } else if (propName == "fdm/jsbsim/velocities/tat-r") {
    port = mAeroForce->getOutputPort("temperature");
    port = addMultiBodyToUnit("Temperature Rankine", uRankine, port);
  } else if (propName == "fdm/jsbsim/velocities/tat-f") {
    port = mAeroForce->getOutputPort("temperature");
    port = addMultiBodyToUnit("Degree Fahrenheit", uFahrenheit, port);

    // Braindead: a pressure value in velocities ...
  } else if (propName == "fdm/jsbsim/velocities/pt-pa") {
    port = mAeroForce->getOutputPort("pressure");
  } else if (propName == "fdm/jsbsim/velocities/pt-lbs_sqft") {
    port = mAeroForce->getOutputPort("temperature");
    port = addMultiBodyToUnit("Static pressure psf", uPoundPFt2, port);

  } else if (propName == "fdm/jsbsim/aero/alpha-rad") {
    port = mAeroForce->getOutputPort("alpha");
  } else if (propName == "fdm/jsbsim/aero/mag-alpha-rad") {
    port = mAeroForce->getOutputPort("alpha");
    port = addMultiBodyAbsModel("Angle of attack mag", port);
  } else if (propName == "fdm/jsbsim/aero/alpha-deg") {
    port = mAeroForce->getOutputPort("alpha");
    port = addMultiBodyToUnit("Angle of attack deg", uDegree, port);
  } else if (propName == "fdm/jsbsim/aero/mag-alpha-deg") {
    port = lookupJSBExpression("aero/alpha-deg");
    port = addMultiBodyAbsModel("Angle of attack mag deg", port);

  } else if (propName == "fdm/jsbsim/aero/beta-rad") {
    port = mAeroForce->getOutputPort("beta");
  } else if (propName == "fdm/jsbsim/aero/mag-beta-rad") {
    port = mAeroForce->getOutputPort("beta");
    port = addMultiBodyAbsModel("Angle of attack mag", port);
  } else if (propName == "fdm/jsbsim/aero/beta-deg") {
    port = mAeroForce->getOutputPort("beta");
    port = addMultiBodyToUnit("Angle of attack deg", uDegree, port);
  } else if (propName == "fdm/jsbsim/aero/mag-beta-deg") {
    port = lookupJSBExpression("aero/beta-deg");
    port = addMultiBodyAbsModel("Angle of attack mag deg", port);

  } else if (propName == "fdm/jsbsim/aero/alphadot-rad_sec") {
    port = mAeroForce->getOutputPort("alphaDot");
  } else if (propName == "fdm/jsbsim/aero/alphadot-deg_sec") {
    port = mAeroForce->getOutputPort("alphaDot");
    port = addMultiBodyToUnit("Angle of attack deriv deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/aero/betadot-rad_sec") {
    port = mAeroForce->getOutputPort("betaDot");
  } else if (propName == "fdm/jsbsim/aero/betadot-deg_sec") {
    port = mAeroForce->getOutputPort("betaDot");
    port = addMultiBodyToUnit("Angle of attack deriv deg_sec", uDegree, port);

  } else if (propName == "fdm/jsbsim/metrics/bw-ft") {
    /// FIXME, just schedule a constant block for that??
    port = mAeroForce->getOutputPort("wingSpan");
    port = addMultiBodyToUnit("Wingspan ft", uFoot, port);

  } else if (propName == "fdm/jsbsim/metrics/Sw-sqft") {
    /// FIXME, just schedule a constant block for that??
    port = mAeroForce->getOutputPort("wingArea");
    port = addMultiBodyToUnit("Wingarea ft2", uFoot2, port);

  } else if (propName == "fdm/jsbsim/metrics/cbarw-ft") {
    /// FIXME, just schedule a constant block for that??
    port = mAeroForce->getOutputPort("coord");
    port = addMultiBodyToUnit("Coord ft", uFoot, port);

  } else if (propName == "fdm/jsbsim/aero/bi2vel") {
    port = mAeroForce->getOutputPort("wingSpanOver2Speed");

  } else if (propName == "fdm/jsbsim/aero/ci2vel") {
    port = mAeroForce->getOutputPort("coordOver2Speed");

  } else if (propName == "fdm/jsbsim/aero/h_b-cg-ft") {
    port = mAeroForce->getOutputPort("hOverWingSpan");

  } else if (propName == "fdm/jsbsim/aero/h_b-mac-ft") {
    /// Hmmm, FIXME
    port = lookupJSBExpression("aero/h_b-cg-ft");

  }

  if (port && port->isConnected())
    registerExpression(propName, port);
  
  return port;
}

Port*
JSBSimReaderBase::addInputModel(const std::string& name,
                            const std::string& propName, real_type gain)
{
  Input* input = new Input(name + " Input");
  input->setInputName(propName);
  input->setInputGain(gain);
  addFCSModel(input);
  Port* port = input->getOutputPort(0);
  registerExpression(propName, port);
  return port;
}

void
JSBSimReaderBase::addOutputModel(Port* out,
                             const std::string& name,
                             const std::string& propName, real_type gain)
{
  Output* output = new Output(name + " Output");
  output->getInputPort(0)->connect(out);
  output->setOutputName(propName);
  output->setOutputGain(gain);
  addFCSModel(output);
}

Port*
JSBSimReaderBase::addInverterModel(const std::string& name, Port* in)
{
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Inverter", UnaryFunctionModel::Minus);
  unary->getInputPort(0)->connect(in);
  addFCSModel(unary);
  return unary->getOutputPort(0);
}

Port*
JSBSimReaderBase::addAbsModel(const std::string& name, Port* in)
{
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Abs", UnaryFunctionModel::Abs);
  unary->getInputPort(0)->connect(in);
  addFCSModel(unary);
  return unary->getOutputPort(0);
}

Port*
JSBSimReaderBase::addConstModel(const std::string& name, real_type value)
{
  Matrix m(1, 1);
  m(1, 1) = 0;
  ConstModel* cModel = new ConstModel(name, m);
  addFCSModel(cModel);
  return cModel->getOutputPort(0);
}

void
JSBSimReaderBase::addFCSModel(Model* model)
{
  // FIXME
  while (mVehicle->getModelGroup()->addModel(model) == ~0u) {
    model->setName(model->getName() + "x");
  }
}

Port*
JSBSimReaderBase::addMultiBodyToUnit(const std::string& name, Unit u, Port* in)
{
  if (!in)
    return 0;
  UnitConversionModel* unitConv
    = new UnitConversionModel(name, UnitConversionModel::SiToUnit, u);
  addMultiBodyModel(unitConv);
  unitConv->getInputPort(0)->connect(in);
  return unitConv->getOutputPort(0);
}

Port*
JSBSimReaderBase::addMultiBodyFromUnit(const std::string& name, Unit u, Port* in)
{
  if (!in)
    return 0;
  UnitConversionModel* unitConv
    = new UnitConversionModel(name, UnitConversionModel::UnitToSi, u);
  addMultiBodyModel(unitConv);
  unitConv->getInputPort(0)->connect(in);
  return unitConv->getOutputPort(0);
}

Port*
JSBSimReaderBase::addMultiBodyAbsModel(const std::string& name, Port* in)
{
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Abs", UnaryFunctionModel::Abs);
  unary->getInputPort(0)->connect(in);
  addMultiBodyModel(unary);
  return unary->getOutputPort(0);
}

void
JSBSimReaderBase::addMultiBodyModel(Model* model)
{
  // FIXME
  while (mVehicle->getMultiBodySystem()->addModel(model) == ~0u) {
    model->setName(model->getName() + "x");
  }
}

Port*
JSBSimReaderBase::getTablePrelookup(const std::string& name, Port* in,
                                    const TableLookup& tl)
{
  if (!in)
    return 0;
  // First check if we already have a table lookup for this port/brakepoint
  // combination. If so return that output port
  std::vector<SharedPtr<TablePreLookup> >::iterator it;
  for (it = mTableLookups.begin(); it != mTableLookups.end(); ++it) {
    if (tl == (*it)->getTableLookup() &&
        in->hasSameSource((*it)->getInputPort(0))) {
      return (*it)->getOutputPort(0);
    }
  }

  // No sharable table lookup found, we need to create a new one
  TablePreLookup* tablePreLookup
    = new TablePreLookup(name + " Table Prelookup");
  addMultiBodyModel(tablePreLookup);
  tablePreLookup->setTableLookup(tl);
  tablePreLookup->getInputPort(0)->connect(in);
  mTableLookups.push_back(tablePreLookup);
  return tablePreLookup->getOutputPort(0);
}

} // namespace OpenFDM
