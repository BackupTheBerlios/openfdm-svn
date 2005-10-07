/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>
#include <sstream>
#include <iostream>
#include <list>
#include <map>
#include <stack>
#include <locale>

#include <simgear/xml/easyxml.hxx>

#include <OpenFDM/Expression.h>
#include <OpenFDM/Model.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/TimeDerivative.h>
#include <OpenFDM/TransferFunction.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/SimpleGear.h>
#include <OpenFDM/SimpleContact.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/DeadBand.h>
#include <OpenFDM/UnaryFunctionModel.h>
#include <OpenFDM/Bias.h>
#include <OpenFDM/Input.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/DirectForce.h>
#include <OpenFDM/Property.h>
#include "JSBReader.h"
#include <OpenFDM/XML/Tablereader.h>

namespace OpenFDM {

JSBReader::JSBReader(void)
  : mVehicle(new Vehicle),
    mEngineNumber(0),
    mGearNumber(0)
{
  /// FIXME make independent of mEnvironment
  mAeroForce = new AeroForce(mVehicle->mEnvironment, "Aerodynamic reference frame");
  // Attach to the vehicle.
  mVehicle->mTopBody->addMultiBodyModel(mAeroForce);
  // Create the aerodynamic properties of JSBSim.
  makeAeroprops();
}

JSBReader::~JSBReader(void)
{
  // Clean up, this can happen if an xml reader exception is thrown.
  while (!mElementStack.empty()) {
    delete mElementStack.top();
    mElementStack.pop();
  }
}

void
JSBReader::startElement(const char *name, const XMLAttributes& atts)
{
  mElementStack.push(new tagdata(name, atts));
  tagdata* current = mElementStack.top();
  int depth = mElementStack.size();

  if (depth == 1 && strcmp("FDM_CONFIG", name) == 0) {
  }
  else if (depth == 2 && strcmp("METRICS", name) == 0) {
  }
  else if (depth == 2 && strcmp("UNDERCARRIAGE", name) == 0) {
  }
  else if (depth == 2 && strcmp("PROPULSION", name) == 0) {
    mParseMode = PROPULSIONMode;
  }
  else if (depth == 2 && strcmp("OUTPUT", name) == 0) {
  }
  else if (depth == 3 && mParseMode == PROPULSIONMode &&
           strcmp("AC_ENGINE", name) == 0) {
  }
  else if (depth == 4 && mParseMode == PROPULSIONMode &&
           strcmp("AC_THRUSTER", name) == 0) {
  }
  else if (depth == 3 && mParseMode == PROPULSIONMode &&
           strcmp("AC_TANK", name) == 0) {
  }
  else if (depth == 2 && (strcmp("FLIGHT_CONTROL", name) == 0 ||
                          strcmp("AUTOPILOT", name) == 0)) {
    mParseMode = FCSMode;
  }
  else if (depth == 3 && mParseMode == FCSMode &&
           strcmp("COMPONENT", name) == 0) {
  }
  else if (depth == 2 && strcmp("AERODYNAMICS", name) == 0) {
    mParseMode = AERODYNAMICSMode;
  }
  else if (depth == 3 && mParseMode == AERODYNAMICSMode &&
           strcmp("AXIS", name) == 0) {
    const char* axisname
      = ((XMLAttributes&)current->attributes).getValue("NAME");
    if (!axisname) {
      cerr << "AXIS tag without NAME attribute! Ignoring!" << endl;
      return;
    }
    shared_ptr<UnitToSiExpressionImpl> toNewton
      = new UnitToSiExpressionImpl(uPoundForce);
    shared_ptr<UnitToSiExpressionImpl> toNewtonMeter
      = new UnitToSiExpressionImpl(uPoundForceFt);
    shared_ptr<SumExpressionImpl> sum = new SumExpressionImpl;
    toNewtonMeter->setInputProperty(TypedProperty<real_type>(sum));
    toNewton->setInputProperty(TypedProperty<real_type>(sum));
    mCurrentAxis.push(sum);
    if (strcmp(axisname, "LIFT") == 0) {
      shared_ptr<MinusExpressionImpl> minus = new MinusExpressionImpl;
      minus->setInputProperty(TypedProperty<real_type>(toNewton));
      mAeroForce->addStabilityAxisSummand(AeroForce::LiftAxis, TypedProperty<real_type>(minus));
    }
    else if (strcmp(axisname, "DRAG") == 0) {
      shared_ptr<MinusExpressionImpl> minus = new MinusExpressionImpl;
      minus->setInputProperty(TypedProperty<real_type>(toNewton));
      mAeroForce->addStabilityAxisSummand(AeroForce::DragAxis, TypedProperty<real_type>(minus));
    }
    else if (strcmp(axisname, "SIDE") == 0) {
      mAeroForce->addBodyAxisSummand(AeroForce::SideAxis, TypedProperty<real_type>(toNewton));
    } else if (strcmp(axisname, "ROLL") == 0) {
      mAeroForce->addBodyAxisSummand(AeroForce::RollAxis, TypedProperty<real_type>(toNewtonMeter));
    } else if (strcmp(axisname, "PITCH") == 0) {
      mAeroForce->addBodyAxisSummand(AeroForce::PitchAxis, TypedProperty<real_type>(toNewtonMeter));
    } else if (strcmp(axisname, "YAW") == 0) {
      mAeroForce->addBodyAxisSummand(AeroForce::YawAxis, TypedProperty<real_type>(toNewtonMeter));
    } else {
      cerr << "AXIS tag with unknown NAME attribute! Ignoring!" << endl;
      return;
    }
  }
  else if (depth >= 4 && mParseMode == AERODYNAMICSMode &&
           strcmp("COEFFICIENT", name) == 0) {
  }
  else if (depth >= 4 && mParseMode == AERODYNAMICSMode &&
           strcmp("GROUP", name) == 0) {
    shared_ptr<ProductExpressionImpl> prod = new ProductExpressionImpl;
    mCurrentAxisProd.push(prod);
    mCurrentAxis.top()->addInputProperty(TypedProperty<real_type>(prod));
    shared_ptr<SumExpressionImpl> sum = new SumExpressionImpl;
    mCurrentAxis.push(sum);
    prod->addInputProperty(TypedProperty<real_type>(sum));
  }
  else if (depth >= 4 && mParseMode == AERODYNAMICSMode &&
           strcmp("FACTOR", name) == 0) {
  }
  else {
    cerr << "Oooops, unknown tag \"" << name << "\"! Ignoring!" << endl;
  }
}

void
JSBReader::endElement(const char *name)
{
  tagdata* current = mElementStack.top();
  int depth = mElementStack.size();
  mElementStack.pop();
  
  if (depth == 1 && strcmp("FDM_CONFIG", name) == 0) {
  }
  else if (depth == 2 && strcmp("METRICS", name) == 0) {
    parseMetrics(current->data);
  }
  else if (depth == 2 && strcmp("UNDERCARRIAGE", name) == 0) {
    parseUndercarriage(current->data);
  }
  else if (depth == 2 && strcmp("PROPULSION", name) == 0) {
    mParseMode = UNKOWNMode;
  }
  else if (depth == 3 && mParseMode == PROPULSIONMode &&
           strcmp("AC_ENGINE", name) == 0) {
    const char* file = ((XMLAttributes&)current->attributes).getValue("FILE");
    parseEngine(current->data, file);
  }
  else if (depth == 4 && mParseMode == PROPULSIONMode &&
           strcmp("AC_THRUSTER", name) == 0) {
    const char* file = ((XMLAttributes&)current->attributes).getValue("FILE");
    parseThruster(current->data, file);
  }
  else if (depth == 3 && mParseMode == PROPULSIONMode &&
           strcmp("AC_TANK", name) == 0) {
    const char* type = ((XMLAttributes&)current->attributes).getValue("TYPE");
    const char* number = ((XMLAttributes&)current->attributes).getValue("NUMBER");
    parseTank(current->data, type, number);
  }
  else if (depth == 2 && (strcmp("FLIGHT_CONTROL", name) == 0 ||
                          strcmp("AUTOPILOT", name) == 0)) {
    mParseMode = UNKOWNMode;
  }
  else if (depth == 3 && mParseMode == FCSMode &&
           strcmp("COMPONENT", name) == 0) {
    // name lookup problem ...
    const char* name = ((XMLAttributes&)current->attributes).getValue("NAME");
    const char* type = ((XMLAttributes&)current->attributes).getValue("TYPE");
    parseFCSComponent(current->data, name, type);
  }
  else if (depth == 2 && strcmp("AERODYNAMICS", name) == 0) {
    mParseMode = UNKOWNMode;
  }
  else if (depth == 3 && mParseMode == AERODYNAMICSMode &&
           strcmp("AXIS", name) == 0) {
    mCurrentAxis.pop();
  }
  else if (depth >= 4 && mParseMode == AERODYNAMICSMode &&
           strcmp("COEFFICIENT", name) == 0) {
    const char* type = ((XMLAttributes&)current->attributes).getValue("TYPE");
    mCurrentAxis.top()->addInputProperty(parseCoefficient(current->data, type));
  }
  else if (depth >= 4 && mParseMode == AERODYNAMICSMode &&
           strcmp("GROUP", name) == 0) {
    mCurrentAxis.pop();
    mCurrentAxisProd.pop();
  }
  else if (depth >= 4 && mParseMode == AERODYNAMICSMode &&
           strcmp("FACTOR", name) == 0) {
    const char* type = ((XMLAttributes&)current->attributes).getValue("TYPE");
    mCurrentAxisProd.top()->addInputProperty(parseCoefficient(current->data, type));
  }
  
  delete current;
}

void
JSBReader::data(const char *s, int length)
{
  mElementStack.top()->data << std::string(s, length);
}

void
JSBReader::warning(const char *message, int line, int column)
{
}

std::string
JSBReader::propNameFromJSBSim(const std::string& jsbSymbol)
{
  /// Convert a JSBSim, property name like it can appear in config files
  /// into a flightgear property name. That is it strips the optional - sign
  /// and prepends the property with fdm/jsbsim/

  if (jsbSymbol.empty())
    return jsbSymbol;

  string propName = jsbSymbol;

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
JSBReader::propMinusFromJSBSim(const std::string& jsbSymbol)
{
  /// Returns true in case the JSBSim property contains a minus
  return 0 < jsbSymbol.size() && jsbSymbol[0] == '-';
}

std::string
JSBReader::normalizeComponentName(const std::string& name)
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

Property
JSBReader::lookupJSBExpression(const std::string& name)
{
  // Convert to something being able to look up
  std::string propName = propNameFromJSBSim(name);
  
  Property prop;
  if (mExpressionTable.count(propName) <= 0) {
    // Not yet available, so look and see if it is an input
    prop = createAndScheduleInput(propName);

    // Ok, still not available, create a constant zero thing and bail out ...
    if (!prop.isValid()) {
      cerr << "Creating expression \"" << propName << "\"" << endl;
      return Property(new ConstExpressionPropertyImpl<real_type>(0));
    }
    
  } else {
    // Just get that one, if we already have it
    prop = mExpressionTable[propName];
  }

  // If we need the negative input, just multiply with a negative gain
  if (propMinusFromJSBSim(name))
    return addInverterModel(name.substr(name.rfind('/')), prop);
  else
    return prop;
}

void
JSBReader::registerExpression(const std::string& name, Property expr)
{
  if (name.size() <= 0)
    return;
  if (!expr.isValid())
    return;
  if (0 < mExpressionTable.count(name)) {
    cerr << "Already have an expression for " << name << endl;
    return;
  }

  mExpressionTable[name] = expr;
}

void
JSBReader::registerJSBExpression(const std::string& name, Property expr)
{
  if (name.size() <= 0)
    return;
  if (!expr.isValid())
    return;
  if (name[0] == '/')
    registerExpression(name.substr(1), expr);
  else
    registerExpression("fdm/jsbsim/" + name, expr);

  // Well, just an other kind of black magic ...
  if (0 < name.size() && name[0] == '/') {
    addOutputModel(expr, name.substr(name.rfind('/')+1), name.substr(1));
  } else if (name == "fcs/elevator-pos-norm") {
    addOutputModel(expr, name.substr(4),
                   "surface-positions/elevator-pos-norm");
  } else if (name == "fcs/left-aileron-pos-norm" ||
             name == "fcs/aileron-pos-norm") {
    addOutputModel(expr, name.substr(4),
                   "surface-positions/left-aileron-pos-norm");
  } else if (name == "fcs/right-aileron-pos-norm") {
    addOutputModel(expr, name.substr(4),
                   "surface-positions/right-aileron-pos-norm");
  } else if (name == "fcs/rudder-pos-norm") {
    addOutputModel(expr, name.substr(4),
                   "surface-positions/rudder-pos-norm");
  } else if (name == "fcs/speedbrake-pos-norm") {
    addOutputModel(expr, name.substr(4),
                   "surface-positions/speedbrake-pos-norm");
  } else if (name == "fcs/spoiler-pos-norm") {
    addOutputModel(expr, name.substr(4),
                   "surface-positions/spoiler-pos-norm");
  } else if (name == "fcs/flap-pos-norm") {
    addOutputModel(expr, name.substr(4),
                   "surface-positions/flap-pos-norm");
  }
}

Property
JSBReader::createAndScheduleInput(const std::string& propName)
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
    Property prop;
    if (propName == "fdm/jsbsim/fcs/aileron-cmd-norm") {
      prop = addInputModel("Aileron Input",
                           "controls/flight/aileron");

    } else if (propName == "fdm/jsbsim/fcs/roll-trim-cmd-norm") {
      prop = addInputModel("Aileron Trim Input",
                           "controls/flight/aileron-trim");

    } else if (propName == "fdm/jsbsim/fcs/elevator-cmd-norm") {
      prop = addInputModel("Elevator Input",
                           "controls/flight/elevator");

    } else if (propName == "fdm/jsbsim/fcs/pitch-trim-cmd-norm") {
      prop = addInputModel("Elevator Trim Input",
                           "controls/flight/elevator-trim");

    } else if (propName == "fdm/jsbsim/fcs/rudder-cmd-norm") {
      // FIXME is inverted in JSBSim ...
      prop = addInputModel("Rudder Input",
                           "controls/flight/rudder");

    } else if (propName == "fdm/jsbsim/fcs/yaw-trim-cmd-norm") {
      // FIXME also with a minus
      prop = addInputModel("Yaw Trim Input",
                           "controls/flight/rudder-trim");

    } else if (propName == "fdm/jsbsim/fcs/steer-cmd-norm") {
      // FIXME is seperate in flightgear ???
      // prop = addInputModel("Steering Input", "controls/gear/steering");
      prop = addInputModel("Steering Input",
                           "controls/flight/rudder");

    } else if (propName.substr(0, 28) == "fdm/jsbsim/fcs/steer-pos-deg") {
      return lookupJSBExpression("fcs/steer-cmd-norm");

    } else if (propName == "fdm/jsbsim/fcs/flap-cmd-norm") {
      prop = addInputModel("Flaps Input",
                           "controls/flight/flaps");

    } else if (propName == "fdm/jsbsim/fcs/speedbrake-cmd-norm") {
      prop = addInputModel("Speedbrake Input",
                           "controls/flight/speedbrake");

    } else if (propName == "fdm/jsbsim/fcs/spoiler-cmd-norm") {
      prop = addInputModel("Spoiler Input",
                           "controls/flight/spoiler");


    } else if (propName.substr(0, 32) == "fdm/jsbsim/fcs/throttle-cmd-norm") {
      std::string control = "controls/engines/engine" +
        propName.substr(32) + "/throttle";
      prop = addInputModel("Throttle Input " + propName.substr(33, 1),
                           control);

    } else if (propName.substr(0, 32) == "fdm/jsbsim/fcs/throttle-pos-norm") {
      std::string cmd = "fcs/throttle-cmd-norm" + propName.substr(32);
      return lookupJSBExpression(cmd);


    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/mixture-cmd-norm") {
      std::string control = "controls/engines/engine" + 
        propName.substr(31) + "/mixture";
      prop = addInputModel("Mixture Input " + propName.substr(32, 1), control);

    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/mixture-pos-norm") {
      std::string cmd = "fcs/mixture-cmd-norm" + propName.substr(31);
      return lookupJSBExpression(cmd);


    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/advance-cmd-norm") {
      std::string control = "controls/engines/engine" + 
        propName.substr(31) + "/propeller-pitch";
      prop = addInputModel("Propeller Pitch Input " + propName.substr(32, 1),
                           control);

    } else if (propName.substr(0, 31) == "fdm/jsbsim/fcs/advance-pos-norm") {
      std::string cmd = "fcs/advance-cmd-norm" + propName.substr(31);
      return lookupJSBExpression(cmd);

    } else if (propName == "fdm/jsbsim/gear/gear-cmd-norm") {
      prop = addInputModel("Gear Retract Input",
                           "controls/gear/gear-down");

    } else if (propName == "fdm/jsbsim/gear/gear-pos-norm") {
      return lookupJSBExpression("gear/gear-cmd-norm");


    } else if (propName.substr(0, 19) == "fdm/jsbsim/fcs/mag-") {
      // Special absolute modules for fcs/mag-*
      // remove the 'mag-' substring here and use that as input for the
      // Abs block
      std::string name = "fcs/" + propName.substr(19);
      Property in = lookupJSBExpression(name);
      prop = addAbsModel(propName.substr(15), in);



//     } else if (propName == "fdm/jsbsim/") {
//       prop = addInputModel(" Input", "");
    }

    if (prop.isValid())
      registerExpression(propName, prop);

    return prop;
  }



#if 0
/controls/gear/brake-parking
/controls/gear/brake-left
/controls/gear/brake-right
/controls/gear/copilot-brake-left
/controls/gear/copilot-brake-right

/controls/gear/tailhook
/controls/gear/launchbar
/controls/gear/catapult-launch-cmd
/controls/gear/tailwheel-lock


 
    for (i = 0; i < Propulsion->GetNumTanks(); i++) {
      SGPropertyNode * node = fgGetNode("/consumables/fuel/tank", i, true);
      FGTank * tank = Propulsion->GetTank(i);
      tank->SetContents(node->getDoubleValue("level-gal_us") * 6.6);
//       tank->SetContents(node->getDoubleValue("level-lb"));
    }
    SGPropertyNode* node = fgGetNode("/systems/refuel", true);
    Propulsion->SetRefuel(node->getDoubleValue("contact"));
    Propulsion->SetFuelFreeze((fgGetNode("/sim/freeze/fuel",true))->getBoolValue());

#endif

  return Property();
}

Property
JSBReader::addInputModel(const std::string& name, const std::string& propName,
                         real_type gain)
{
  Input* input = new Input(name);
  input->setInputName(propName);
  input->setInputGain(gain);
  addFCSModel(input);
  Property prop = input->getOutputPort(0);
  registerExpression(propName, prop);
  return prop;
}

void
JSBReader::addOutputModel(const Property& out, const std::string& name,
                          const std::string& propName, real_type gain)
{
  Output* output = new Output(std::string(name) + " output");
  output->setInputPort(0, out);
  output->setOutputName(propName);
  output->setOutputGain(gain);
  addFCSModel(output);
}

Property
JSBReader::addInverterModel(const std::string& name, Property& in)
{
  Gain *gain = new Gain(name + " Inverter");
  gain->setInputPort(0, in);
  gain->setGain(-1);
  addFCSModel(gain);
  return gain->getOutputPort(0);
}

Property
JSBReader::addAbsModel(const std::string& name, Property& in)
{
  UnaryFunctionModel *unary
    = new UnaryFunctionModel(name + " Abs", new AbsExpressionImpl);
  unary->setInputPort(0, in);
  addFCSModel(unary);
  return unary->getOutputPort(0);
}

void
JSBReader::addFCSModel(Model* model)
{
  // FIXME
  while (!mVehicle->getModelGroup()->addModel(model)) {
    model->setName(model->getName() + "x");
  }
}

void
JSBReader::parseMetrics(istream& data)
{
  // Parse the METRICS section.
  // Since all locations within the aircraft are given in the structural frame,
  // we need to know the body frame location in the veihcle first.
  // That is parse all entries and than translate them into OpenFDM parts.
  InertiaMatrix I(0, 0, 0, 0, 0, 0);
  real_type mass = 0;
  Vector3 vrp, ap, ep;
  typedef std::pair<Vector3,real_type> masspoint;
  typedef list<masspoint> masslist;
  masslist masses;

  while (data) {
    std::string name;
    data >> name;
    
    if (name == "AC_WINGAREA") {
      double value;
      data >> value;
      mAeroForce->setWingArea(convertFrom(uFoot2, value));
    } else if (name == "AC_WINGSPAN") {
      double value;
      data >> value;
      mAeroForce->setWingSpan(convertFrom(uFoot, value));
    } else if (name == "AC_WINGINCIDENCE") {
      double value;
      data >> value;
      registerJSBExpression("metrics/iw-deg",
                            Property(new ConstExpressionPropertyImpl<real_type>(value)));
    } else if (name == "AC_CHORD") {
      double value;
      data >> value;
      mAeroForce->setCoord(convertFrom(uFoot, value));
    } else if (name == "AC_HTAILAREA") {
      double value;
      data >> value;
      registerJSBExpression("metrics/Sh-sqft",
                            Property(new ConstExpressionPropertyImpl<real_type>(value)));
    } else if (name == "AC_HTAILARM") {
      double value;
      data >> value;
      registerJSBExpression("metrics/lh-ft",
                            Property(new ConstExpressionPropertyImpl<real_type>(value)));
    } else if (name == "AC_VTAILAREA") {
      double value;
      data >> value;
      registerJSBExpression("metrics/Sv-sqft",
                            Property(new ConstExpressionPropertyImpl<real_type>(value)));
    } else if (name == "AC_VTAILARM") {
      double value;
      data >> value;
      registerJSBExpression("metrics/lv-ft",
                            Property(new ConstExpressionPropertyImpl<real_type>(value)));
    } else if (name == "AC_IXX") {
      double value;
      data >> value;
      I(1, 1) = convertFrom(uSlugFt2, value);
    } else if (name == "AC_IYY") {
      double value;
      data >> value;
      I(2, 2) = convertFrom(uSlugFt2, value);
    } else if (name == "AC_IZZ") {
      double value;
      data >> value;
      I(3, 3) = convertFrom(uSlugFt2, value);
    } else if (name == "AC_IXY") {
      double value;
      data >> value;
      I(1, 2) = -convertFrom(uSlugFt2, value);;
    } else if (name == "AC_IXZ") {
      double value;
      data >> value;
      I(1, 3) = -convertFrom(uSlugFt2, value);
    } else if (name == "AC_IYZ") {
      double value;
      data >> value;
      I(2, 3) = -convertFrom(uSlugFt2, value);
    } else if (name == "AC_EMPTYWT") {
      data >> mass;
      mass = convertFrom(uPoundSealevel, mass);
    } else if (name == "AC_CGLOC") {
      double value[3];
      data >> value[0] >> value[1] >> value[2];
      mCG = Vector3(value[0], value[1], value[2]);
    } else if (name == "AC_EYEPTLOC") {
      double value[3];
      data >> value[0] >> value[1] >> value[2];
      ep = Vector3(value[0], value[1], value[2]);
    } else if (name == "AC_AERORP") {
      double value[3];
      data >> value[0] >> value[1] >> value[2];
      ap = Vector3(value[0], value[1], value[2]);
    } else if (name == "AC_VRP") {
      double value[3];
      data >> value[0] >> value[1] >> value[2];
      vrp = Vector3(value[0], value[1], value[2]);
    } else if (name == "AC_POINTMASS") {
      Vector3 loc;
      double mpmass;
      data >> mpmass >> loc(1) >> loc(2) >> loc(3);
      masses.push_back(masspoint(loc, mpmass));
    }
  }

  // Now collect all static inertia values starting with the emptyweight
  // and empty inertia together in spi.
  SpatialInertia spi(I, mass);
  masslist::iterator it = masses.begin();
  while (it != masses.end()) {
    SpatialInertia inertia(convertFrom(uPoundSealevel, it->second));
    spi += inertiaFrom(structToBody(it->first), inertia);
    ++it;
  }
  mVehicle->mTopBody->addMultiBodyModel(new Mass(spi));

  // Attach the visual reference point.
  FreeFrame* vrpFrame = new FreeFrame;
  vrpFrame->setPosition(structToBody(vrp));
  mVehicle->mTopBody->addChildFrame(vrpFrame);

  // Attach the eye point.
  FreeFrame* epFrame = new FreeFrame;
  epFrame->setPosition(structToBody(ep));
  mVehicle->mTopBody->addChildFrame(epFrame);

  // Set the position of the aerodynamic force frame.
  mAeroForce->setPosition(structToBody(ap));
}

void
JSBReader::parseUndercarriage(istream& data)
{
  // Undercarriage parsing.
  while (data) {
    std::string uctype;
    data >> uctype;
    
    if (uctype == "AC_GEAR") {
      string name, type, brake, retract;
      double x, y, z, k, d, fs, fd, rr, sa;
      data >> name >> x >> y >> z >> k >> d >> fs >> fd >> rr
           >> type >> brake >> sa >> retract;

      if (type == "CASTERING") {
        // Modelling castering gars as simple contacs without a special
        // direction
        SimpleContact* sc = new SimpleContact(name, mVehicle->mEnvironment);
        sc->setPosition(structToBody(Vector3(x, y, z)));

        sc->setSpringConstant(convertFrom(uPoundForcePFt, k));
        // FIXME: conversion factor:
        // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
        // Note that friction coefficients are different from that but
        // viscosous friction is just used in that way ...
        sc->setSpringDamping(convertFrom(uPoundForcePFt, d));
        sc->setFrictionCoeficient(0.1*fs);
        
        mVehicle->mTopBody->addMultiBodyModel(sc);

      } else {
        // For jsbsim use simple gears
        SimpleGear* sg = new SimpleGear(name, mVehicle->mEnvironment);
        sg->setPosition(structToBody(Vector3(x, y, z)));
        
        sg->setSpringConstant(convertFrom(uPoundForcePFt, k));
        // FIXME: conversion factor:
        // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
        // Note that friction coefficients are different from that but
        // viscosous friction is just used in that way ...
        sg->setSpringDamping(convertFrom(uPoundForcePFt, d));
        sg->setFrictionCoeficient(fs);
        
        // Connect apprioriate input and output models

        std::stringstream sstr;
        sstr << mGearNumber;
        std::string numStr = sstr.str();

        // FIXME
        // missing output properties are "wow" and "tire-pressure-norm"

        if (retract == "RETRACT") {
          Property prop = lookupJSBExpression("gear/gear-pos-norm");
          sg->setInputPort("enabled", prop);
          // Well, connect that directly to the input
          addOutputModel(prop, "Gear " + numStr + " Position",
                         "/gear/gear[" + numStr + "]/position-norm");
        }

        if (type == "STEERABLE") {
          // FIXME: FCS might later define something for that gain ...
//           prop = lookupJSBExpression("fcs/steer-pos-deg[" + numStr + "]");
          Property prop = lookupJSBExpression("fcs/steer-cmd-norm");
          Gain* gain = new Gain(name + " Steer Gain");
          gain->setGain(sa);
          gain->setInputPort(0, prop);
          addFCSModel(gain);
          addOutputModel(prop, "Gear " + numStr + " Steering Output",
                         "/gear/gear[" + numStr + "]/steering-norm");

          UnaryFunctionModel *unary
            = new UnaryFunctionModel(name + " Degree Conversion",
                                     new UnitToSiExpressionImpl(uDegree));
          unary->setInputPort(0, gain->getOutputPort(0));
          addFCSModel(unary);

          sg->setInputPort("steeringAngle", unary->getOutputPort(0));
        }
        
        if (brake == "LEFT") {
          MaxExpressionImpl* mex = new MaxExpressionImpl;
          Property prop = lookupJSBExpression("/controls/gear/brake-left");
          mex->addInputProperty(prop);
          prop = lookupJSBExpression("/controls/gear/copilot-brake-left");
          mex->addInputProperty(prop);
          prop = lookupJSBExpression("/controls/gear/brake-parking");
          mex->addInputProperty(prop);
          sg->setInputPort("brakeCommand", Property(mex));
        } else if (brake == "RIGHT") {
          MaxExpressionImpl* mex = new MaxExpressionImpl;
          Property prop = lookupJSBExpression("/controls/gear/brake-right");
          mex->addInputProperty(prop);
          prop = lookupJSBExpression("/controls/gear/copilot-brake-right");
          mex->addInputProperty(prop);
          prop = lookupJSBExpression("/controls/gear/brake-parking");
          mex->addInputProperty(prop);
          sg->setInputPort("brakeCommand", Property(mex));
        }
        
        mVehicle->mTopBody->addMultiBodyModel(sg);
      }
      
    } else if (uctype == "AC_LAUNCHBAR") {
      string d;
      data >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d;

    } else if (uctype == "AC_HOOK") {
      string d;
      data >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d >> d;

    } else if (uctype == "AC_CONTACT") {
      string name, type, brake, retract;
      double x, y, z, k, d, fs, fd, rr, sa;
      data >> name >> x >> y >> z >> k >> d >> fs >> fd >> rr
           >> type >> brake >> sa >> retract;

      SimpleContact* sc = new SimpleContact(name, mVehicle->mEnvironment);
      sc->setPosition(structToBody(Vector3(x, y, z)));

      sc->setSpringConstant(convertFrom(uPoundForcePFt, k));
      // FIXME: conversion factor:
      // Works since it is used as N/(m/s)=Ns/m (seconds don't change)
      // Note that friction coefficients are different from that but
      // viscosous friction is just used in that way ...
      sc->setSpringDamping(convertFrom(uPoundForcePFt, d));
      sc->setFrictionCoeficient(fs);

      mVehicle->mTopBody->addMultiBodyModel(sc);
    }

    // Increment the gear number
    ++mGearNumber;
  }
}

void
JSBReader::parseFCSComponent(istream& data, const char* name, const char* type)
{
  if (!type) {
    cerr << "No FCS TYPE attribute given in ??? FIXME" << endl;
    return;
  }

  // The model we put into the fcs group in the end ...
  shared_ptr<Model> model;
  shared_ptr<DeadBand> deadband;
  shared_ptr<Summer> summer;
  shared_ptr<Gain> gain;
  shared_ptr<DiscreteTransferFunction> discreteTransfFunc;
  shared_ptr<Table1D> table1D;

  // The final output property.
  Property out;

  // JSBSim FCS output values contain some implicit rules.
  // From the component name a default output property is formed.
  // If we find an OUTPUT line this is the output value too.
  // So collect them first and register them later when the final output
  // expression is known.
  std::list<std::string> outlist;
  outlist.push_back(string("fcs/") + normalizeComponentName(name));

  FCSComponent cType;
  if (strcmp(type, "SUMMER") == 0) {
    cType = SummerComponent;
    summer = new Summer(name);
    summer->setNumSummands(0);
    model = summer;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "DEADBAND") == 0) {
    cType = DeadbandComponent;

    deadband = new DeadBand(name);
    model = deadband;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "GRADIENT") == 0) {
    cType = GradientComponent;
    model = new TimeDerivative(name);
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "SWITCH") == 0) {
    std::cout << "Ignoring SWITCH" << std::endl;
    cType = SwitchComponent;

  } else if (strcmp(type, "KINEMAT") == 0) {
    std::cout << "Ignoring KINEMAT" << std::endl;
    cType = KinematComponent;

    gain = new Gain(name);
    gain->setGain(1);
    model = gain;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "PURE_GAIN") == 0) {
    cType = GainComponent;

    gain = new Gain(name);
    gain->setGain(1);
    model = gain;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "AEROSURFACE_SCALE") == 0) {
    cType = AeroScaleComponent;

    table1D = new Table1D(name);
    TableLookup tl;
    tl.addBreakPoint(-1);
    tl.addBreakPoint(0);
    tl.addBreakPoint(1);
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

    model = table1D;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "SCHEDULED_GAIN") == 0) {
    cType = SchedGainComponent;

    Product* prod = new Product(name);
    prod->setNumFactors(2);
    table1D = new Table1D(std::string("Lookup table for ") + name);
    addFCSModel(table1D);
    prod->setInputPort(1, table1D->getOutputPort(0));
    model = prod;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "INTEGRATOR") == 0) {
    cType = IntegratorComponent;

    model = new DiscreteIntegrator(name);
    out = model->getOutputPort(0);
    addFCSModel(model);

  } else if (strcmp(type, "LAG_FILTER") == 0) {
    //   C1
    // ------
    // s + C1
    cType = FilterComponent;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    Vector v(1);
    discreteTransfFunc->setNumerator(v);
    v.resize(2);
    v(1) = 1;
    discreteTransfFunc->setDenominator(v);
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "LEAD_LAG_FILTER") == 0) {
    // C1*s + C2
    // ---------
    // C3*s + C4
    cType = FilterComponent;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    discreteTransfFunc->setNumerator(Vector(2));
    discreteTransfFunc->setDenominator(Vector(2));
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "WASHOUT_FILTER") == 0) {
    //   s
    // ------
    // s + C1
    cType = FilterComponent;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    Vector v(2);
    v(1) = 1;
    v(2) = 0;
    discreteTransfFunc->setNumerator(v);
    discreteTransfFunc->setDenominator(v);
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else if (strcmp(type, "SECOND_ORDER_FILTER") == 0) {
    // C1*s + C2*s + C3
    // ----------------
    // C4*s + C5*s + C6
    cType = FilterComponent;
    discreteTransfFunc = new DiscreteTransferFunction(name);
    discreteTransfFunc->setNumerator(Vector(3));
    discreteTransfFunc->setDenominator(Vector(3));
    model = discreteTransfFunc;
    addFCSModel(model);
    out = model->getOutputPort(0);

  } else {
    cerr << "Unknown FCS COMPONENT type: \"" << type
         << "\". Ignoring whole FCS component \""
         << name << "\"!" << endl;
    return;
  }

  OpenFDMAssert(out.isValid());

  // The output expression from the prevous FCS block.
  // This is the default input value for this FCS block.
  std::list<Property> inputs;

  // Collect any expressions for the output chain here.
  shared_ptr<Saturation> saturation;
  shared_ptr<Bias> outbias;
  shared_ptr<Gain> outgain;
  bool outInvert = false;
  bool noScale = false;

  for (;;) {
    std::string token;
    data >> token;
    if (!data)
      break;

    if (token == "BIAS") {
      double value;
      data >> value;

      std::string modelName = std::string("Output Bias for ") + name;
      outbias = new Bias(modelName);
      addFCSModel(outbias);

      Matrix tmp(1, 1);
      tmp(1, 1) = value;
      outbias->setBias(tmp);
      
    } else if (token == "CLIPTO") {
      double clipmin, clipmax;
      data >> clipmin >> clipmax;
      
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
      double value;
      data >> value;

      if (cType == IntegratorComponent) {
        std::string modelName = std::string("Output Gain for ") + name;
        outgain = new Gain(modelName);
        addFCSModel(outgain);
        outgain->setGain(value);
      } else if (cType == FilterComponent) {
        if (strcmp(type, "LAG_FILTER") == 0) {
          //   C1
          // ------
          // s + C1
          Vector v = discreteTransfFunc->getNumerator();
          v(2) = value;
          discreteTransfFunc->setNumerator(v);
          v = discreteTransfFunc->getDenominator();
          v(2) = value;
          discreteTransfFunc->setDenominator(v);

        } else if (strcmp(type, "LEAD_LAG_FILTER") == 0) {
          // C1*s + C2
          // ---------
          // C3*s + C4
          Vector v = discreteTransfFunc->getNumerator();
          v(1) = value;
          discreteTransfFunc->setNumerator(v);

        } else if (strcmp(type, "WASHOUT_FILTER") == 0) {
          //   s
          // ------
          // s + C1
          Vector v = discreteTransfFunc->getDenominator();
          v(2) = value;
          discreteTransfFunc->setDenominator(v);

        } else if (strcmp(type, "SECOND_ORDER_FILTER") == 0) {
          // C1*s + C2*s + C3
          // ----------------
          // C4*s + C5*s + C6
          Vector v = discreteTransfFunc->getNumerator();
          v(1) = value;
          discreteTransfFunc->setNumerator(v);
        } else
          cerr << "FIXME: ERROR" << endl;
      } else
        cerr << "FIXME: ERROR" << endl;
      
    } else if (token == "C2") {
      double value;
      data >> value;

      if (strcmp(type, "LEAD_LAG_FILTER") == 0) {
        // C1*s + C2
        // ---------
        // C3*s + C4
        Vector v = discreteTransfFunc->getNumerator();
        v(2) = value;
        discreteTransfFunc->setNumerator(v);
        
      } else if (strcmp(type, "SECOND_ORDER_FILTER") == 0) {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getNumerator();
        v(2) = value;
        discreteTransfFunc->setNumerator(v);
      } else
        cerr << "FIXME: ERROR" << endl;
      
    } else if (token == "C3") {
      double value;
      data >> value;

      if (strcmp(type, "LEAD_LAG_FILTER") == 0) {
        // C1*s + C2
        // ---------
        // C3*s + C4
        Vector v = discreteTransfFunc->getDenominator();
        v(1) = value;
        discreteTransfFunc->setDenominator(v);
        
      } else if (strcmp(type, "SECOND_ORDER_FILTER") == 0) {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getNumerator();
        v(3) = value;
        discreteTransfFunc->setNumerator(v);

      } else
        cerr << "FIXME: ERROR" << endl;
      
    } else if (token == "C4") {
      double value;
      data >> value;

      if (strcmp(type, "LEAD_LAG_FILTER") == 0) {
        // C1*s + C2
        // ---------
        // C3*s + C4
        Vector v = discreteTransfFunc->getDenominator();
        v(2) = value;
        discreteTransfFunc->setDenominator(v);
        
      } else if (strcmp(type, "SECOND_ORDER_FILTER") == 0) {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getDenominator();
        v(1) = value;
        discreteTransfFunc->setDenominator(v);

      } else
        cerr << "FIXME: ERROR" << endl;
      
    } else if (token == "C5") {
      double value;
      data >> value;

      if (strcmp(type, "SECOND_ORDER_FILTER") == 0) {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getDenominator();
        v(2) = value;
        discreteTransfFunc->setDenominator(v);

      } else
        cerr << "FIXME: ERROR" << endl;
      
    } else if (token == "C6") {
      double value;
      data >> value;

      if (strcmp(type, "SECOND_ORDER_FILTER") == 0) {
        // C1*s + C2*s + C3
        // ----------------
        // C4*s + C5*s + C6
        Vector v = discreteTransfFunc->getDenominator();
        v(3) = value;
        discreteTransfFunc->setDenominator(v);

      } else
        cerr << "FIXME: ERROR" << endl;
      
    } else if (cType == KinematComponent && token == "DETENTS") {
      int detents;
      data >> detents;

      real_type maxVal = 0;
      while (data && 0 <= --detents) {
        real_type dummy;
        data >> maxVal >> dummy;
//         Kinemat::table_entry te;
//         data >> te.first >> te.second;
//         kinemat->addTableEntry(te);
      }

      if (gain && !noScale) {
        gain->setGain(maxVal);
      }

    } else if (token == "GAIN") {
      double value;
      data >> value;

      if (gain) {
        gain->setGain(value);
      } else {
        std::string modelName = std::string(name) + " OGain";
        outgain = new Gain(modelName);
        addFCSModel(outgain);
        outgain->setGain(value);
      }
      
    } else if (token == "INPUT") {
      data >> token;
      inputs.push_back(lookupJSBExpression(token));
      
    } else if (token == "INVERT") {
      // Append a minus expression to the output chain.
      outInvert = true;
      
    } else if (token == "MAX") {
      double clipmax;
      data >> clipmax;
      
      if (cType == AeroScaleComponent) {
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
      double clipmin;
      data >> clipmin;
      
      if (cType == AeroScaleComponent) {
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

    } else if (cType == KinematComponent && token == "NOSCALE") {
      noScale = true;
      if (gain)
        gain->setGain(1);
      
    } else if (token == "OUTPUT") {
      data >> token;
      // Make sure that the token is not already in the outlist ... FIXME
      outlist.remove(token);
      // Add it
      outlist.push_back(token);

    } else if (token == "ROWS") {
      int rows;
      data >> rows;
      
      TableData<1>::SizeVector sz;
      sz(1) = rows;
      TableData<1> tableData(sz);
      TableLookup lookup;
      parseTable1D(data, tableData, lookup);

      if (table1D) {
        table1D->setTableData(tableData);
        table1D->setTableLookup(lookup);
      }

    } else if (token == "SCHEDULED_BY") {
      data >> token;
      
      if (table1D) {
        table1D->setInputPort(0, lookupJSBExpression(token));
      } else {
        cerr << "SCHEDULED_BY without table ??" << endl;
      }
      
    } else if (cType == DeadbandComponent && token == "WIDTH") {
      // deadband width
      double width;
      data >> width;
      deadband->setWidth(width);

    } else {
      cerr << "Unknown FCS COMPONENT keyword: \"" << token
           << "\". Ignoring whole FCS component \""
           << name << "\"!" << endl;
      return;
    }
  }

  // If no explicit input is given, use that braindead default of the
  // prevous block
  if (inputs.empty())
    inputs.push_back(mPrevousFCSOutput);

  unsigned idx = 0;
  for (std::list<Property>::iterator it = inputs.begin();
       it != inputs.end(); ++it) {
    // FIXME!!!!!!
    if (summer && summer->getNumInputPorts() <= idx)
      summer->setNumSummands(idx+1);
    model->setInputPort(idx++, *it);
  }

  // Sanity check, if the first FCS component does not have an input defined.
  if (!inputs.front().isValid()) {
    cerr << "No input value defined for FCS COMPONENT. "
         << "Ignoring whole FCS component \""
         << name << "\"!" << endl;
    return;
  }

  // Now put the expressions for the output chain together.
  // Doing that now will put them together in a well defined order.
  if (outbias) {
    outbias->setInputPort(0, out);
    out = outbias->getOutputPort(0);
  }
  if (outgain) {
    outgain->setInputPort(0, out);
    out = outgain->getOutputPort(0);
  }
  if (saturation) {
    saturation->setInputPort(0, out);
    out = saturation->getOutputPort(0);
  }
  if (outInvert) {
    out = addInverterModel(name, out);
  }
  // FIXME put in here a normalized out property, or at least a gain to
  // normalize

  // Register all output property names.
  std::list<std::string>::iterator it;
  for (it = outlist.begin(); it != outlist.end(); ++it) {
    std::string propName = *it;
    registerJSBExpression(propName, out);

    // Well, just an other kind of black magic ...
    if (propName == "fcs/elevator-pos-rad") {
      registerJSBExpression("fcs/elevator-pos-norm", inputs.front());
    } else if (propName == "fcs/left-aileron-pos-rad" ||
               propName == "fcs/aileron-pos-rad") {
      registerJSBExpression("fcs/left-aileron-pos-norm", inputs.front());
    } else if (propName == "fcs/right-aileron-pos-rad") {
      registerJSBExpression("fcs/right-aileron-pos-norm", inputs.front());
    } else if (propName == "fcs/rudder-pos-rad") {
      registerJSBExpression("fcs/rudder-pos-norm", inputs.front());
    } else if (propName == "fcs/speedbrake-pos-rad") {
      registerJSBExpression("fcs/speedbrake-pos-norm", inputs.front());
    } else if (propName == "fcs/spoiler-pos-rad") {
      registerJSBExpression("fcs/spoiler-pos-norm", inputs.front());
    } else if (propName == "fcs/flap-pos-deg") {
      registerJSBExpression("fcs/flap-pos-norm", inputs.front());
    }
  }

  // Finally set the current components output to the default input for the
  // next block.
  mPrevousFCSOutput = out;
}

TypedProperty<real_type>
JSBReader::parseCoefficient(istream& data, const char* type)
{
  ProductExpressionImpl* prod = new ProductExpressionImpl;
  if (!type) {
    cerr << "TYPE attribute missing for COEFFICIENT tag! Ignoring!" << endl;
    return TypedProperty<real_type>(prod);
  }

  int ndims;
  if (strcmp(type, "VALUE") == 0) {
    ndims = 0;
  } else if (strcmp(type, "VECTOR") == 0) {
    ndims = 1;
  } else if (strcmp(type, "TABLE") == 0) {
    ndims = 2;
  } else if (strcmp(type, "TABLE3D") == 0) {
    ndims = 3;
  } else {
    cerr << "Unknown TYPE attribute \"" << type
         << "\" for COEFFICIENT tag! Ignoring!" << endl;
    return TypedProperty<real_type>(prod);
  }

  std::string token;
  // The fist token is some useless string.
  data >> token;

  // The number of table entries 
  int n[3] = { 0 };
  for (int i = 0; i < ndims; ++i)
    data >> n[i];

  // The table lookup values.
  Property inVal[3];
  for (int i = 0; i < ndims; ++i) {
    data >> token;
    inVal[i] = lookupJSBExpression(token);
  }

  // The other factors in this product.
  std::string line;
  std::getline(data, line, '\n');
  std::getline(data, line, '\n');

  std::string::size_type pos;
  while ((pos = line.find('|')) != std::string::npos)
    line.replace(pos, 1, " ");
  std::stringstream linestream(line);
  while (linestream >> token) {
    if (token.empty() || token == "none")
      break;
    prod->addInputProperty(lookupJSBExpression(token));
  }
 
  // The lookup table values.
  if (ndims == 0) {
    real_type value;
    data >> value;
    prod->addInputProperty(Property(new ConstExpressionPropertyImpl<real_type>(value)));
  }
  else if (ndims == 1) {
    TableData<1>::SizeVector size;
    size(1) = n[0];
    TableData<1> table(size);
    TableLookup lookup;
    parseTable1D(data, table, lookup);
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
    parseTable2D(data, table, lookup);
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
    parseTable3D(data, table, lookup);
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
JSBReader::parseTank(istream& data, const char* type, const char* number)
{
  Vector3 loc = Vector3::zeros();
  real_type radius = 0;
  real_type capacity = 0;
  real_type contents = 0;
  real_type temperature = 0;

  for (;;) {
    std::string token;
    data >> token;
    if (!data)
      break;

    if (token == "XLOC") {
      data >> loc(1);
      
    } else if (token == "YLOC") {
      data >> loc(2);
      
    } else if (token == "ZLOC") {
      data >> loc(3);
      
    } else if (token == "RADIUS") {
      data >> radius;
      
    } else if (token == "CAPACITY") {
      data >> capacity;
      
    } else if (token == "CONTENTS") {
      data >> contents;
      
    } else if (token == "TEMPERATURE") {
      data >> temperature;
      
    } else {
      cerr << "Unknown parameter in tank configuration! Ignoring!" << endl;
      return;
    }
  }
}

void
JSBReader::parseThruster(istream& data, const char* file)
{
  Vector3 loc = Vector3::zeros();
  real_type pitch = 0;
  real_type yaw = 0;

  for (;;) {
    std::string token;
    data >> token;
    if (!data)
      break;

    if (token == "XLOC") {
      data >> loc(1);
      
    } else if (token == "YLOC") {
      data >> loc(2);
      
    } else if (token == "ZLOC") {
      data >> loc(3);
      
    } else if (token == "PITCH") {
      data >> pitch;
      
    } else if (token == "YAW") {
      data >> yaw;
      
    } else if (token == "P_FACTOR") {
      double d;
      data >> d;
      
    } else if (token == "SENSE") {
      double d;
      data >> d;
      
    } else {
      cerr << "Unknown parameter in tank configuration! Ignoring!" << endl;
      return;
    }
  }
}

void
JSBReader::parseEngine(istream& data, const char* file)
{
  Vector3 loc = Vector3::zeros();
  real_type pitch = 0;
  real_type yaw = 0;

  for (;;) {
    std::string token;
    data >> token;
    if (!data)
      break;

    if (token == "XLOC") {
      data >> loc(1);
      
    } else if (token == "YLOC") {
      data >> loc(2);
      
    } else if (token == "ZLOC") {
      data >> loc(3);
      
    } else if (token == "PITCH") {
      data >> pitch;
      
    } else if (token == "YAW") {
      data >> yaw;
      
    } else if (token == "FEED") {
      double d;
      data >> d;
      
    } else {
      cerr << "Unknown parameter in engine configuration! Ignoring!" << endl;
      return;
    }
  }

  std::stringstream namestr;
  namestr << "Engine<" << mEngineNumber << ">";
  DirectForce* engineForce = new DirectForce(namestr.str());
  engineForce->setDirection(Vector6(0, 0, 0, 4.4*1.5e4, 0, 0));
  engineForce->setPosition(structToBody(loc));
  engineForce->setOrientation(Quaternion::fromHeadAttBank(pitch, 0, yaw));

  std::stringstream throttlename;
  throttlename << "fcs/throttle-cmd-norm[" << mEngineNumber << "]";
  engineForce->setInputPort(0, lookupJSBExpression(throttlename.str()));

  mVehicle->mTopBody->addMultiBodyModel(engineForce);

  mEngineNumber++;
}

void
JSBReader::makeAeroprops(void)
{
  Property e = mAeroForce->getProperty("trueSpeed");
  registerJSBExpression("velocities/vt-mps", e);
  SiToUnitExpressionImpl* c = new SiToUnitExpressionImpl(uFeetPSecond);
  c->setInputProperty(e);
  registerJSBExpression("velocities/vt-fps", Property(c));
  c = new SiToUnitExpressionImpl(uKnots);
  c->setInputProperty(e);
  registerJSBExpression("velocities/vt-kts", Property(c));

  // Mach numbers, are unitless.
  e = mAeroForce->getProperty("mach");
  registerJSBExpression("velocities/mach-norm", e);

  // Rotational rates wrt air.
  e = mAeroForce->getProperty("p");
  registerJSBExpression("velocities/p-rad_sec", e);
  registerJSBExpression("velocities/p-aero-rad_sec", e);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  registerJSBExpression("velocities/p-aero-deg_sec", Property(c));
  e = mAeroForce->getProperty("q");
  registerJSBExpression("velocities/q-rad_sec", e);
  registerJSBExpression("velocities/q-aero-rad_sec", e);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  registerJSBExpression("velocities/q-aero-deg_sec", Property(c));
  e = mAeroForce->getProperty("r");
  registerJSBExpression("velocities/r-rad_sec", e);
  registerJSBExpression("velocities/r-aero-rad_sec", e);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  registerJSBExpression("velocities/r-aero-deg_sec", Property(c));


  e = mAeroForce->getProperty("u");
  registerJSBExpression("velocities/u-aero-mps", e);
  c = new SiToUnitExpressionImpl(uFeetPSecond);
  c->setInputProperty(e);
  registerJSBExpression("velocities/u-aero-fps", Property(c));
  e = mAeroForce->getProperty("v");
  registerJSBExpression("velocities/v-aero-mps", e);
  c = new SiToUnitExpressionImpl(uFeetPSecond);
  c->setInputProperty(e);
  registerJSBExpression("velocities/v-aero-fps", Property(c));
  e = mAeroForce->getProperty("w");
  registerJSBExpression("velocities/w-aero-mps", e);
  c = new SiToUnitExpressionImpl(uFeetPSecond);
  c->setInputProperty(e);
  registerJSBExpression("velocities/w-aero-fps", Property(c));


  // Dynamic pressure values.
  e = mAeroForce->getProperty("dynamicPressure");
  registerJSBExpression("aero/qbar-pa", e);
  c = new SiToUnitExpressionImpl(uPoundPFt2);
  c->setInputProperty(e);
  registerJSBExpression("aero/qbar-psf", Property(c));

  // Temperature.
  e = mAeroForce->getProperty("temperature");
  c = new SiToUnitExpressionImpl(uRankine);
  c->setInputProperty(e);
  registerJSBExpression("velocities/tat-r", Property(c));
  c = new SiToUnitExpressionImpl(uFahrenheit);
  c->setInputProperty(e);
  registerJSBExpression("velocities/tat-f", Property(c));

  // Braindead: a pressure value in velocities ...
  e = mAeroForce->getProperty("pressure");
  registerJSBExpression("velocities/pt-pascal", e);
  c = new SiToUnitExpressionImpl(uPoundPFt2);
  c->setInputProperty(e);
  registerJSBExpression("velocities/pt-lbs_sqft", Property(c));



  e = mAeroForce->getProperty("wingSpan");
  c = new SiToUnitExpressionImpl(uFoot);
  c->setInputProperty(e);
  registerJSBExpression("metrics/bw-ft", Property(c));

  e = mAeroForce->getProperty("wingArea");
  c = new SiToUnitExpressionImpl(uFoot2);
  c->setInputProperty(e);
  registerJSBExpression("metrics/Sw-sqft", Property(c));

  e = mAeroForce->getProperty("coord");
  c = new SiToUnitExpressionImpl(uFoot);
  c->setInputProperty(e);
  registerJSBExpression("metrics/cbarw-ft", Property(c));

  e = mAeroForce->getProperty("wingSpanOver2Speed");
  registerJSBExpression("aero/bi2vel", e);
  e = mAeroForce->getProperty("coordOver2Speed");
  registerJSBExpression("aero/ci2vel", e);

  // Angle of attack.
  e = mAeroForce->getProperty("alpha");
  registerJSBExpression("aero/alpha-rad", e);
  AbsExpressionImpl* a = new AbsExpressionImpl();
  a->setInputProperty(e);
  registerJSBExpression("aero/mag-alpha-rad", Property(a));
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  registerJSBExpression("aero/alpha-deg", Property(c));
  a = new AbsExpressionImpl();
  a->setInputProperty(Property(c));
  registerJSBExpression("aero/mag-alpha-deg", Property(a));

  // Angle of sideslip.
  e = mAeroForce->getProperty("beta");
  registerJSBExpression("aero/beta-rad", e);
  a = new AbsExpressionImpl();
  a->setInputProperty(e);
  registerJSBExpression("aero/mag-beta-rad", Property(a));
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  registerJSBExpression("aero/beta-deg", Property(c));
  a = new AbsExpressionImpl();
  a->setInputProperty(Property(c));
  registerJSBExpression("aero/mag-beta-deg", Property(a));


  // Time derivative of alpha.
  e = mAeroForce->getProperty("alphaDot");
  registerJSBExpression("aero/alphadot-rad_sec", e);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  registerJSBExpression("aero/alphadot-deg", Property(c));

  // Time derivative of beta.
  e = mAeroForce->getProperty("betaDot");
  registerJSBExpression("aero/betadot-rad_sec", e);
  c = new SiToUnitExpressionImpl(uDegree);
  c->setInputProperty(e);
  registerJSBExpression("aero/betadot-deg", Property(c));

  // The quotient agl/wingspan
  e = mAeroForce->getProperty("hOverWingSpan");
  c = new SiToUnitExpressionImpl(uFoot);
  c->setInputProperty(e);
  registerJSBExpression("aero/h_b-cg-ft", Property(c));
  registerJSBExpression("aero/h_b-mac-ft", Property(c));
}

} // namespace OpenFDM
