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
#include <OpenFDM/ConstSystem.h>
#include <OpenFDM/DeadBand.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/TransferFunction.h>
#include <OpenFDM/DirectForce.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Input.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/LinearSpring.h>
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
#include <OpenFDM/TimeDerivative.h>
#include <OpenFDM/UnaryFunctionModel.h>
#include <OpenFDM/Units.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/WheelContact.h>
#include <OpenFDM/DiscBrake.h>

#include <OpenFDM/ReaderWriter.h>
#include <OpenFDM/XML/Tablereader.h>
#include <OpenFDM/XML/XMLReader.h>

#include "JSBSimAerosurfaceScale.h"
#include "JSBSimKinemat.h"
#include "JSBSimScheduledGain.h"

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
  mAeroForce = new AeroForce("Aerodynamic force");
  mVehicle->getTopBody()->addInteract(mAeroForce);
  // Default discrete stepsize of JSBSim
  mVehicle->getModelGroup()->addSampleTime(SampleTime(1.0/120));

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
      MaxModel* maxModel = new MaxModel("Right Brake Max");
      maxModel->setNumMaxInputs(3);

      Port* pilotBr = addInputModel("Right Brake Input",
                                    "controls/gear/brake-right");
      maxModel->getInputPort(0)->connect(pilotBr);

      Port* copilotBr = addInputModel("Right Copilot Brake Input",
                                      "controls/gear/copilot-brake-right");
      maxModel->getInputPort(1)->connect(copilotBr);

      Port* parkBr = lookupJSBExpression("/controls/gear/brake-parking");
      maxModel->getInputPort(2)->connect(parkBr);

      addFCSModel(maxModel);
      port = maxModel->getOutputPort(0);

    } else if (propName == "fdm/jsbsim/gear/left-brake-pos-norm") {
      MaxModel* maxModel = new MaxModel("Left Brake Max");
      maxModel->setNumMaxInputs(3);

      Port* pilotBr = addInputModel("Left Brake Input",
                                    "controls/gear/brake-left");
      maxModel->getInputPort(0)->connect(pilotBr);

      Port* copilotBr = addInputModel("Left Copilot Brake Input",
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
LegacyJSBSimReader::createAndScheduleAeroProp(const std::string& propName)
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
    port = addMultiBodyToUnit("Wingarea ft", uFoot2, port);

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
LegacyJSBSimReader::addInputModel(const std::string& name,
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
LegacyJSBSimReader::addOutputModel(Port* out,
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
LegacyJSBSimReader::addInverterModel(const std::string& name, Port* in)
{
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Inverter", UnaryFunctionModel::Minus);
  unary->getInputPort(0)->connect(in);
  addFCSModel(unary);
  return unary->getOutputPort(0);
}

Port*
LegacyJSBSimReader::addAbsModel(const std::string& name, Port* in)
{
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Abs", UnaryFunctionModel::Abs);
  unary->getInputPort(0)->connect(in);
  addFCSModel(unary);
  return unary->getOutputPort(0);
}

Port*
LegacyJSBSimReader::addConstModel(const std::string& name, real_type value)
{
  Matrix m(1, 1);
  m(1, 1) = 0;
  ConstSystem* cModel = new ConstSystem(name, m);
  addFCSModel(cModel);
  return cModel->getOutputPort(0);
}

void
LegacyJSBSimReader::addFCSModel(Model* model)
{
  // FIXME
  while (mVehicle->getModelGroup()->addModel(model) == ~0u) {
    model->setName(model->getName() + "x");
  }
}

Port*
LegacyJSBSimReader::addMultiBodyToUnit(const std::string& name, Unit u,
                                       Port* in)
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
LegacyJSBSimReader::addMultiBodyFromUnit(const std::string& name, Unit u,
                                         Port* in)
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
LegacyJSBSimReader::addMultiBodyAbsModel(const std::string& name, Port* in)
{
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Abs", UnaryFunctionModel::Abs);
  unary->getInputPort(0)->connect(in);
  addMultiBodyModel(unary);
  return unary->getOutputPort(0);
}

void
LegacyJSBSimReader::addMultiBodyModel(Model* model)
{
  // FIXME
  while (mVehicle->getMultiBodySystem()->addModel(model) == ~0u) {
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
      Port* port = addConstModel("metrics/iw-deg constant", value);
      registerJSBExpression("metrics/iw-deg", port);
    } else if (name == "AC_CHORD") {
      real_type value;
      datastr >> value;
      mAeroForce->setCoord(convertFrom(uFoot, value));
    } else if (name == "AC_HTAILAREA") {
      real_type value;
      datastr >> value;
      Port* port = addConstModel("metrics/Sh-sqft constant", value);
      registerJSBExpression("metrics/Sh-sqft", port);
    } else if (name == "AC_HTAILARM") {
      real_type value;
      datastr >> value;
      Port* port = addConstModel("metrics/lh-ft constant", value);
      registerJSBExpression("metrics/lh-ft", port);
    } else if (name == "AC_VTAILAREA") {
      real_type value;
      datastr >> value;
      Port* port = addConstModel("metrics/Sv-sqft constant", value);
      registerJSBExpression("metrics/Sv-sqft", port);
    } else if (name == "AC_VTAILARM") {
      real_type value;
      datastr >> value;
      Port* port = addConstModel("metrics/lv-ft constant", value);
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
  mVehicle->getTopBody()->addInteract(new Mass("Emptyweight Mass", spi));

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
  InertiaMatrix wheelInertia(10, 0, 0, 30, 0, 10);
  wheel->addInteract(new Mass(name + " Wheel Inertia",
                              SpatialInertia(wheelInertia, 30)));
  mVehicle->getMultiBodySystem()->addRigidBody(wheel);
  
  RevoluteJoint* wj = new RevoluteJoint(name + " Wheel Joint");
  parent->addInteract(wj);
  wheel->setInboardJoint(wj);
  wj->setJointAxis(Vector3(0, 1, 0));
  wj->setPosition(pos);
  wj->setOrientation(Quaternion::unit());
  wj->setJointPos(0);
  wj->setJointVel(0);

  // Add a brake force
  if (brake == "LEFT" || brake == "RIGHT") {
    DiscBrake* brakeF = new DiscBrake(name + " Brake Force");
    brakeF->setMinForce(8e1);
    brakeF->setMaxForce(1e4);
    if (brake == "LEFT") {
      Port* port = lookupJSBExpression("gear/left-brake-pos-norm");
      brakeF->getInputPort(0)->connect(port);
    } else if (brake == "RIGHT") {
      Port* port = lookupJSBExpression("gear/right-brake-pos-norm");
      brakeF->getInputPort(0)->connect(port);
    }
    // That one reads the joint position and velocity ...
    brakeF->getInputPort(1)->connect(wj->getOutputPort(1));
    // ... and provides an output force
    wj->getInputPort(0)->connect(brakeF->getOutputPort(0));
    mVehicle->getMultiBodySystem()->addModel(brakeF);
  } else {
    // Just some 'rolloing friction' FIXME: does this belong here?
    Gain* rollingFric = new Gain(name + " Rolling Friction Force");
    rollingFric->setGain(-10);
    rollingFric->getInputPort(0)->connect(wj->getOutputPort(1));
    // ... and provides an output force
    wj->getInputPort(0)->connect(rollingFric->getOutputPort(0));
    mVehicle->getMultiBodySystem()->addModel(rollingFric);
  }
  
  WheelContact* wc = new WheelContact(name + " Wheel Contact");
  wc->setWheelRadius(0.5*wheelDiam);
  wc->setSpringConstant(convertFrom(uPoundForcePFt, tireSpring));
  wc->setSpringDamping(convertFrom(uPoundForcePFt, tireDamp));
  wc->setFrictionCoeficient(0.9);
  wheel->addInteract(wc);
  
  Port* port = wj->getOutputPort(0);
  std::string nameBase = "Wheel " + numStr + " Position";
  addOutputModel(port, nameBase,
                 "gear/gear[" + numStr + "]/wheel-position-rad");
  UnitConversionModel* unitModel
    = new UnitConversionModel(nameBase + " converter",
                              UnitConversionModel::SiToUnit, uDegree);
  unitModel->getInputPort(0)->connect(port);
  addFCSModel(unitModel);
  addOutputModel(unitModel->getOutputPort(0), nameBase + " Deg",
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
        SimpleContact* sc = new SimpleContact(name);
        sc->setPosition(structToBody(Vector3(x, y, z)));

        sc->setSpringConstant(convertFrom(uPoundForcePFt, k));
        // FIXME: conversion factor:
        // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
        // Note that friction coefficients are different from that but
        // viscosous friction is just used in that way ...
        sc->setSpringDamping(convertFrom(uPoundForcePFt, d));
        sc->setFrictionCoeficient(0.1*fs);
        
        mVehicle->getTopBody()->addInteract(sc);

      } else {
        // For jsbsim use simple gears
        SimpleGear* sg = new SimpleGear(name);
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


          UnitConversionModel* unitConv
            = new UnitConversionModel(name + " Degree Conversion",
                                      UnitConversionModel::UnitToSi,
                                      uDegree);
          unitConv->getInputPort(0)->connect(gain->getOutputPort(0));
          addFCSModel(unitConv);

          sg->getInputPort("steeringAngle")->connect(unitConv->getOutputPort(0));
        }
        
        if (brake == "LEFT") {
          Port* port = lookupJSBExpression("gear/left-brake-pos-norm");
          sg->getInputPort("brakeCommand")->connect(port);
        } else if (brake == "RIGHT") {
          Port* port = lookupJSBExpression("gear/right-brake-pos-norm");
          sg->getInputPort("brakeCommand")->connect(port);
        }
        
        mVehicle->getTopBody()->addInteract(sg);
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
      mVehicle->getMultiBodySystem()->addRigidBody(arm);
      arm->addInteract(new Mass(name + " Strut Mass", inertiaFrom(Vector3(-1, 0, 0), SpatialInertia(100))));

      // Connect that with a revolute joint to the main body
      RevoluteJoint* rj = new RevoluteJoint(name + " Arm Joint");
      mVehicle->getTopBody()->addInteract(rj);
      arm->setInboardJoint(rj);
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
      // That one reads the joint position and velocity ...
      aoDamp->getInputPort(0)->connect(rj->getOutputPort(0));
      aoDamp->getInputPort(1)->connect(rj->getOutputPort(1));
      // ... and provides an output force
      rj->getInputPort(0)->connect(aoDamp->getOutputPort(0));
      mVehicle->getMultiBodySystem()->addModel(aoDamp);

      // Attach a wheel to that strut part.
      attachWheel(name, Vector3(-armLength, 0, 0), brake, numStr, wheelDiam,
                  tireSpring, tireDamp, arm);

      Port* port = rj->getOutputPort(0);
      addOutputModel(port, "Gear " + numStr + " Compression",
                     "/gear/gear[" + numStr + "]/compression-rad");

      /// FIXME add a retract joint ...
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
        mVehicle->getMultiBodySystem()->addRigidBody(steer);

        // connect that via a revolute joint to the toplevel body.
        // Note the 0.05m below, most steering wheels have some kind of
        // castering auto line up behavour. That is doe with this 0.05m.
        RevoluteActuator* sj = new RevoluteActuator(name + " Steer Joint");
        strutParent->addInteract(sj);
        steer->setInboardJoint(sj);
        sj->setJointAxis(Vector3(0, 0, 1));
        sj->setJointPos(0);
        sj->setJointVel(0);
        sj->setPosition(structToBody(compressJointPos)
                        + Vector3(0.05, 0, 0));
        sj->setOrientation(Quaternion::unit());

        Port* port = lookupJSBExpression("fcs/steer-cmd-norm");
        sj->getInputPort(0)->connect(port);
        
        strutParent = steer;
        
        // Prepare outputs
        port = sj->getOutputPort(0);
        std::string nameBase = "Steering " + numStr + " Position";
        addOutputModel(port, nameBase,
                       "/gear/gear[" + numStr + "]/steering-pos-rad");
        UnitConversionModel* unitModel
          = new UnitConversionModel(nameBase + " converter",
                                    UnitConversionModel::SiToUnit, uDegree);
        unitModel->getInputPort(0)->connect(port);
        addFCSModel(unitModel);
        addOutputModel(unitModel->getOutputPort(0), nameBase + " Deg",
                       "/gear/gear[" + numStr + "]/steering-pos-deg");
      }


      // Now the compressible part of the strut
      RigidBody* arm = new RigidBody(name + " Strut");
      mVehicle->getMultiBodySystem()->addRigidBody(arm);
      arm->addInteract(new Mass(name + " Strut Mass", inertiaFrom(Vector3(0, 0, 1), SpatialInertia(100))));

      // This time it is a prismatic joint
      PrismaticJoint* pj = new PrismaticJoint(name + " Compress Joint");
      strutParent->addInteract(pj);
      arm->setInboardJoint(pj);
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
      pj->getInputPort(0)->connect(aoDamp->getOutputPort(0));
      aoDamp->getInputPort(0)->connect(pj->getOutputPort(0));
      aoDamp->getInputPort(1)->connect(pj->getOutputPort(1));
      mVehicle->getMultiBodySystem()->addModel(aoDamp);

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
      SimpleContact* sc = new SimpleContact(name);
      sc->setPosition(structToBody(Vector3(x, y, z)));

      sc->setSpringConstant(convertFrom(uPoundForcePFt, k));
      // FIXME: conversion factor:
      // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
      // Note that friction coefficients are different from that but
      // viscosous friction is just used in that way ...
      sc->setSpringDamping(convertFrom(uPoundForcePFt, d));
      sc->setFrictionCoeficient(fs);

      mVehicle->getTopBody()->addInteract(sc);
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
  
  std::list<SharedPtr<const XMLElement> > comps
    = fcsElem->getElements("COMPONENT");
  std::list<SharedPtr<const XMLElement> >::const_iterator it;
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
  SharedPtr<Model> model;
  SharedPtr<DeadBand> deadband;
  SharedPtr<Summer> summer;
  SharedPtr<Gain> gain;
  SharedPtr<DiscreteTransferFunction> discreteTransfFunc;
  SharedPtr<JSBSimAerosurfaceScale> asScale;
  SharedPtr<JSBSimKinemat> kinemat;
  SharedPtr<JSBSimScheduledGain> sGain;

  // The final output property.
  SharedPtr<Port> out;
  SharedPtr<Port> normOut;

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

    // Use that special proxy class
    kinemat = new JSBSimKinemat(name);
    model = kinemat->getModelGroup();
    addFCSModel(model);
    out = kinemat->getOutputPort();
    normOut = kinemat->getOutputNormPort();

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
    asScale = new JSBSimAerosurfaceScale(name);
    model = asScale->getModelGroup();
    addFCSModel(model);
    out = asScale->getOutputPort();
    normOut = asScale->getOutputNormPort();

  } else if (type == "SCHEDULED_GAIN") {
    sGain = new JSBSimScheduledGain(name);
    model = sGain->getModelGroup();
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
  std::list<SharedPtr<Port> > inputs;

  // Collect any expressions for the output chain here.
  SharedPtr<Saturation> saturation;
  SharedPtr<Bias> outbias;
  SharedPtr<Gain> outgain;
  bool outInvert = false;

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
        if (allTime != 0)
          kinemat->setRateLimit(fabs((maxVal-minVal)/allTime));
        else
          kinemat->setRateLimit(Limits<real_type>::max());
        kinemat->setMinValue(minVal);
        kinemat->setMaxValue(maxVal);

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
        asScale->setMaxValue(clipmax);

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
        asScale->setMinValue(clipmin);

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
        kinemat->setNoScale(true);
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

      sGain->setTableData(tableData, lookup);

    } else if (token == "SCHEDULED_BY") {
      datastr >> token;
      
      if (sGain) {
        model->getInputPort(1)->connect(lookupJSBExpression(token));
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
  for (std::list<SharedPtr<Port> >::iterator it = inputs.begin();
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

  std::list<SharedPtr<XMLElement> > elems = pElem->getElements();
  std::list<SharedPtr<XMLElement> >::const_iterator it;
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

  mVehicle->getTopBody()->addInteract(engineForce);

  return true;
}

bool
LegacyJSBSimReader::convertAerodynamics(const XMLElement* aerodynamics)
{
  std::list<SharedPtr<XMLElement> > elems = aerodynamics->getElements();
  std::list<SharedPtr<XMLElement> >::const_iterator it;
  for (it = elems.begin(); it != elems.end(); ++it) {
    std::string axisname = (*it)->getAttribute("NAME");

    SharedPtr<Summer> sum = new Summer(axisname + " Sum");
    sum->setNumSummands(0);

    // Now parse the summands
    if (!convertAEROSummands(*it, sum, 0))
      return error("Cannot convert aerodynamic summands for axis \"" + 
                   axisname  + "\"");
    if (!sum->getNumSummands())
      continue;
    addMultiBodyModel(sum);
    Port* port = sum->getOutputPort(0);

    if (axisname == "LIFT") {
      port = addMultiBodyFromUnit("LIFT unit converter", uPoundForce, port);
      mAeroForce->getInputPort("lift")->connect(port);
    }
    else if (axisname == "DRAG") {
      port = addMultiBodyFromUnit("DRAG unit converter", uPoundForce, port);
      mAeroForce->getInputPort("drag")->connect(port);
    }
    else if (axisname == "SIDE") {
      port = addMultiBodyFromUnit("SIDE unit converter", uPoundForce, port);
      mAeroForce->getInputPort("side")->connect(port);
    } else if (axisname == "ROLL") {
      port = addMultiBodyFromUnit("ROLL unit converter", uPoundForceFt, port);
      mAeroForce->getInputPort("roll")->connect(port);
    } else if (axisname == "PITCH") {
      port = addMultiBodyFromUnit("PITCH unit converter", uPoundForceFt, port);
      mAeroForce->getInputPort("pitch")->connect(port);
    } else if (axisname == "YAW") {
      port = addMultiBodyFromUnit("YAW unit converter", uPoundForceFt, port);
      mAeroForce->getInputPort("yaw")->connect(port);
    } else
      return error("Unknown aerodynamic axis!");
  }

  return true;
}

bool
LegacyJSBSimReader::convertAEROSummands(const XMLElement* aeroSummands,
                                        Summer* sum, Product* prod)
{
  std::list<SharedPtr<XMLElement> > elems = aeroSummands->getElements();
  std::list<SharedPtr<XMLElement> >::const_iterator it;
  for (it = elems.begin(); it != elems.end(); ++it) {
    if ((*it)->getName() == "GROUP") {
     
      SharedPtr<Product> newProd = new Product("blub FIXME Product");
      addMultiBodyModel(newProd);
      Port* port = newProd->getOutputPort(0);
      unsigned ns = sum->getNumSummands();
      sum->setNumSummands(ns+1);
      sum->getInputPort(ns)->connect(port);

      SharedPtr<Summer> newSum = new Summer("blub FIXME Summer");
      newSum->setNumSummands(0);
      newProd->setNumFactors(1);
      newProd->getInputPort(0)->connect(newSum->getOutputPort(0));
      addMultiBodyModel(newSum);
      
      if (!convertAEROSummands(*it, newSum, newProd))
        return error("Error parsing aerodynamic tables");
    }
    else if ((*it)->getName() == "FACTOR") {
      if (!prod)
        return error("Error parsing aerodynamic tables, FACTOR without GROUP");

      std::string type = (*it)->getAttribute("TYPE");
      Port* port = convertCoefficient((*it)->getData(), type);
      unsigned nf = prod->getNumFactors();
      prod->setNumFactors(nf+1);
      prod->getInputPort(nf)->connect(port);
    }
    else if ((*it)->getName() == "COEFFICIENT") {
      std::string type = (*it)->getAttribute("TYPE");
      Port* port = convertCoefficient((*it)->getData(), type);
      unsigned ns = sum->getNumSummands();
      sum->setNumSummands(ns+1);
      sum->getInputPort(ns)->connect(port);
    }
  }

  return true;
}

Port*
LegacyJSBSimReader::convertCoefficient(const std::string& data,
                                       const std::string& type)
{
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
    return 0;
  }

  std::stringstream datastr(data);

  std::string token;
  // The fist token is some name string ...
  datastr >> token;

  SharedPtr<Product> prod = new Product(token);
  prod->setNumFactors(0);
  addMultiBodyModel(prod);

  // The number of table entries 
  unsigned n[3] = { 0 };
  for (unsigned i = 0; i < ndims; ++i)
    datastr >> n[i];

  // The table lookup values.
  SharedPtr<Port> inVal[3];
  for (unsigned i = 0; i < ndims; ++i) {
    datastr >> token;
    inVal[i] = lookupJSBExpression(token);
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
    unsigned nf = prod->getNumFactors();
    prod->setNumFactors(nf+1);
    prod->getInputPort(nf)->connect(lookupJSBExpression(token));
  }
 
  // The lookup table values.
  if (ndims == 0) {
    Matrix value(1, 1);
    datastr >> value(1, 1);
    ConstSystem* constModel
      = new ConstSystem(prod->getName() + " Factor", value);
    addMultiBodyModel(constModel);

    unsigned nf = prod->getNumFactors();
    prod->setNumFactors(nf+1);
    prod->getInputPort(nf)->connect(constModel->getOutputPort(0));
  }
  else if (ndims == 1) {
    TableData<1>::SizeVector size;
    size(1) = n[0];
    TableData<1> table(size);
    TableLookup lookup;
    if (!parseTable1D(datastr, table, lookup))
      // FIXME
      std::cerr << "Cannot parse " + type + " table" << std::endl;

    Table1D* table1D
      = new Table1D(prod->getName() + " Table");
    addMultiBodyModel(table1D);
    table1D->setTableData(table);
    Port* lPort = getTablePrelookup(prod->getName(), inVal[0], lookup);
    table1D->getInputPort(0)->connect(lPort);

    unsigned nf = prod->getNumFactors();
    prod->setNumFactors(nf+1);
    prod->getInputPort(nf)->connect(table1D->getOutputPort(0));
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

    Table2D* table2D
      = new Table2D(prod->getName() + " Table");
    addMultiBodyModel(table2D);
    table2D->setTableData(table);
    for (unsigned i = 0; i < 2; ++i) {
      Port* lPort = getTablePrelookup(prod->getName(), inVal[i], lookup[i]);
      table2D->getInputPort(i)->connect(lPort);
    }

    unsigned nf = prod->getNumFactors();
    prod->setNumFactors(nf+1);
    prod->getInputPort(nf)->connect(table2D->getOutputPort(0));
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

    Table3D* table3D
      = new Table3D(prod->getName() + " Table");
    addMultiBodyModel(table3D);
    table3D->setTableData(table);
    for (unsigned i = 0; i < 3; ++i) {
      Port* lPort = getTablePrelookup(prod->getName(), inVal[i], lookup[i]);
      table3D->getInputPort(i)->connect(lPort);
    }

    unsigned nf = prod->getNumFactors();
    prod->setNumFactors(nf+1);
    prod->getInputPort(nf)->connect(table3D->getOutputPort(0));
  }

  return prod->getOutputPort(0);
}

Port*
LegacyJSBSimReader::getTablePrelookup(const std::string& name, Port* in,
                                      const TableLookup& tl)
{
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
