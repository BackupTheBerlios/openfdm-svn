/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <simgear/misc/sg_path.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/props/props.hxx>

#include <Aircraft/controls.hxx>
#include <Main/globals.hxx>
#include <Main/fg_props.hxx>

#include <JSBSim/LegacyJSBSimReader.h>
#include <OpenFDM/Units.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/Input.h>
#include <OpenFDM/Ground.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/Model.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/XMLDumpModelVisitor.h>

#include "FGPropertyAdapter.h"

#include "FGOpenFDM.h"

namespace OpenFDM {

class FGGround
  : public Ground {
public:
  FGGround(FGInterface *ifce)
    : mIfce(ifce)
  { }
  virtual ~FGGround(void)
  { }

  virtual GroundValues
  getGroundPlane(real_type t, const Vector3& refPos) const
  {
    double loc_cart[3] = { refPos(1), refPos(2), refPos(3) };
    double contact[3], normal[3], vel[3], lc, ff, agl;
    int groundtype;
    // FIXME!!!!!!!
    bool ok = false;
    if (mIfce)
      ok = mIfce->get_agl_m(t, loc_cart, contact, normal, vel,
                            &groundtype, &lc, &ff, &agl);
    Vector3 unitDown(-normal[0], -normal[1], -normal[2]);
    Vector3 groundOff(contact[0], contact[1], contact[2]);
    // FIXME: why is this not normalized?? ...
    Plane p(normalize(unitDown), groundOff);
    Vector6 v(Vector3::zeros(), Vector3(vel[0], vel[1], vel[2]));
    return GroundValues(p, v, ff);
  }

  void setInterface(FGInterface *ifce)
  { mIfce = ifce; }
  
private:
  FGInterface *mIfce;
};


class FGPlanet
  : public Planet {
public:
  /** Default constructor.
   */
  FGPlanet(void) {}

  /** Default destructor.
   */
  virtual ~FGPlanet(void) {}

  /** Gravity acceleration at the cartesion position cart.
   */
  virtual Vector3 gravityAccel(const Vector3& cart) const
  {
    real_type planetMass = 5.9742e24;
    real_type dist = norm(cart);
    return (-planetMass*gravity_constant/(dist*dist*dist))*cart;
  }

  /** Transform cartesian coordinates to geodetic coordinates.
   */
  virtual Geodetic toGeod(const Vector3& cart) const
  {
    double xyz[3] = { cart(1), cart(2), cart(3) };
    double lat, lon, alt;
    sgCartToGeod(xyz, &lat, &lon, &alt);
    return Geodetic(lat, lon, alt);
  }

  /** Transform geodetic coordinates to cartesian coordinates.
   */
  virtual Vector3 toCart(const Geodetic& geod) const
  {
    double xyz[3];
    sgGeodToCart(geod.latitude, geod.longitude, geod.altitude, xyz);
    return Vector3(xyz[0], xyz[1], xyz[2]);
  }
};

class FGWind
  : public Wind {
public:
  FGWind(void) {}
  virtual ~FGWind(void) {}

  // Return the wind velocity in the global coordinate frame.
  // ????
  // FIXME: make pure virtual
  virtual Vector3 getWindVel(const Vector3& pos) const
  {
    double xyz[3] = { pos(1), pos(2), pos(3) };
    double lat, lon, alt;
    sgCartToGeod(xyz, &lat, &lon, &alt);

    real_type cosLat = cos(lat);
    real_type sinLat = sin(lat);
    real_type cosLon = cos(lon);
    real_type sinLon = sin(lon);

    Matrix33 T(-cosLon*sinLat, -sinLon, -cosLon*cosLat,
               -sinLon*sinLat, cosLon , -sinLon*cosLat,
               cosLat        , 0.0    , -sinLat);

    double north = fgGetDouble("/environment/wind-from-north-fps");
    double east = fgGetDouble("/environment/wind-from-east-fps");
    double down = fgGetDouble("/environment/wind-from-down-fps");

    return convertFrom(uFeetPSecond, T*Vector3(north, east, down));
  }
};


//     temperature = fgGetNode("/environment/temperature-degc",true);
//     pressure = fgGetNode("/environment/pressure-inhg",true);
//     density = fgGetNode("/environment/density-slugft3",true);
//     turbulence_gain = fgGetNode("/environment/turbulence/magnitude-norm",true);
//     turbulence_rate = fgGetNode("/environment/turbulence/rate-hz",true);


void printVehicle(Vehicle* vehicle)
{
  cout << "T = " << vehicle->getTime()
       << ", Pos: " << vehicle->getGeodPosition()
//        << ", Or: " << vehicle->getGeodOrientation()
       << endl;
}


// Our local storage covers the pointer to our vehicle.
// A list of the property to expression adaptors.
struct FGOpenFDMData {
  OpenFDM::SharedPtr<OpenFDM::Vehicle> vehicle;
  OpenFDM::SharedPtr<OpenFDM::FGGround> ground;
};

FGOpenFDM::FGOpenFDM(SGPropertyNode* fdmRootNode) :
  mAircraftRootNode(fdmRootNode)
{
  SG_LOG(SG_FLIGHT, SG_INFO, "FGOpenFDM::FGOpenFDM(...)");

  mData = new FGOpenFDMData;

  // FIXME: this is the place where we can plug in our stuff into an other
  // base directory. For example if this will get an AI aircraft or what ever.
  if (!mAircraftRootNode)
    mAircraftRootNode = SGPropertyNode_ptr(fgGetNode("/", true));
}

FGOpenFDM::~FGOpenFDM(void)
{
  SG_LOG(SG_FLIGHT, SG_INFO, "FGOpenFDM::~FGOpenFDM()");
  mData->ground->setInterface(0);
  delete mData;
}

void FGOpenFDM::init()
{
  SG_LOG(SG_FLIGHT, SG_INFO, "FGOpenFDM::init()");

  // Try to read JSBSim legacy files.
  LegacyJSBSimReader reader;
  reader.addAircraftPath(fgGetString("/sim/aircraft-dir"));
  reader.addEnginePath(std::string(fgGetString("/sim/aircraft-dir"))
                       + "/Engines");
  reader.loadAircraft(std::string(fgGetString("/sim/aero")) + ".xml");
  if (reader.getErrorState()) {
    SG_LOG(SG_FLIGHT, SG_ALERT, "FGOpenFDM::init() cannot read aircraft!");
    const ReaderWriter::StringList errors = reader.getErrors();
    ReaderWriter::StringList::const_iterator it;
    for (it = errors.begin(); it != errors.end(); ++it) {
      SG_LOG(SG_FLIGHT, SG_ALERT, (*it));
    }
    return;
  }

  mData->vehicle = reader.getVehicle();
  Vehicle* vehicle = mData->vehicle;
  mData->ground = new FGGround(this);
  vehicle->setGround(mData->ground);
  vehicle->setPlanet(new FGPlanet);
  vehicle->setWind(new FGWind);

  // Call what needs to be done ... ;-)
  common_init();

  // Hmm, twice ??
  vehicle->init();

  MobileRootJoint* mobileRootJoint = vehicle->getMobileRootJoint();
  // Check the position
  Geodetic gp = Geodetic(get_Latitude(),
                         get_Longitude(),
                         convertFrom(uFoot, get_Altitude()));
  SG_LOG(SG_FLIGHT, SG_INFO, "Geod Pos " << gp << " set.");
  mobileRootJoint->setGeodPosition(gp);

  // Orientation
  Quaternion go = Quaternion::fromHeadAttBank(get_Psi(),
                                              get_Theta(),
                                              get_Phi());
  SG_LOG(SG_FLIGHT, SG_INFO, "Geod Orientation " << go << " set!");
  vehicle->setGeodOrientation(go);

  // Velocity
  std::string speed_set = fgGetString("/sim/presets/speed-set", "UVW");
  if (speed_set == "UVW") {
    Vector3 bodyVel(get_uBody(), get_vBody(), get_wBody());
    mobileRootJoint->setAngularRelVel(Vector3::zeros());
    mobileRootJoint->setLinearRelVel(convertFrom(uFeetPSecond, bodyVel));
//   } else if (speed_set == "NED") {
//   } else if (speed_set == "KNOTS") {
//   } else if (speed_set == "MACH") {
  } else {
    mobileRootJoint->setRelVel(Vector6::zeros());
  }

  // Try to find a stable set of states
  if (!vehicle->trim())
    SG_LOG(SG_FLIGHT, SG_WARN, "Trimming failed!");

  // Copy the trim results back
  gp = vehicle->getGeodPosition();
  _updateGeodeticPosition(gp.latitude, gp.longitude,
                          convertTo(uFoot, gp.altitude));

  Rotation geodOr = vehicle->getGeodOrientation();
  Vector3 euler = geodOr.getEuler();
  _set_Euler_Angles(euler(1), euler(2), euler(3));

  set_inited(true);
}

void FGOpenFDM::bind()
{
  SG_LOG(SG_FLIGHT, SG_INFO, "FGOpenFDM::bind()");

  FGInterface::bind();

  SGPropertyNode* sgProp = mAircraftRootNode->getChild("fdm", 0, true);
  sgProp = sgProp->getChild("vehicle", 0, true);
  sgProp = sgProp->getChild("system", 0, true);
  tieModelGroup(sgProp, mData->vehicle->getSystem());
}

void FGOpenFDM::unbind()
{
  SG_LOG(SG_FLIGHT, SG_INFO, "FGOpenFDM::unbind()");

  untieNamed(mAircraftRootNode, "fdm/vehicle");

  FGInterface::unbind();
}

void
FGOpenFDM::update(double dt)
{
  if (is_suspended() || dt == 0)
    return;

  // Get a local vehicle pointer
  Vehicle* vehicle = mData->vehicle;
  if (!vehicle) {
    SG_LOG(SG_FLIGHT, SG_ALERT,
           "FGOpenFDM::update(double) is called without an aircraft loaded!");
    return;
  }

  // Check the position
  Geodetic gp = Geodetic(get_Latitude(),
                         get_Longitude(),
                         convertFrom(uFoot, get_Altitude()));
  if (!equal(vehicle->getPlanet()->toCart(gp), vehicle->getCartPosition())) {
    SG_LOG(SG_FLIGHT, SG_INFO,
           "Geod Pos set, error = "
           << norm(vehicle->getPlanet()->toCart(gp)-vehicle->getCartPosition())
           << "\n\tprevous: " << vehicle->getGeodPosition()
           << "\n\tcurrent: "<< gp );
    vehicle->setGeodPosition(gp);
  }

  // Check the orientation 
  Vector3 euler = vehicle->getGeodOrientation().getEuler();
  if (!equal(euler, Vector3(get_Phi(), get_Theta(), get_Psi()))) {
    Quaternion
      go = Quaternion::fromHeadAttBank(get_Psi(), get_Theta(), get_Phi());
    SG_LOG(SG_FLIGHT, SG_INFO,
           "Geod Or set, error = "
           << norm(vehicle->getGeodOrientation() - go)
           << "\n\tprevous: " << vehicle->getGeodOrientation()
           << "\n\tcurrent: "<< go );
    vehicle->setGeodOrientation(go);
  }
  
  double acrad = vehicle->getRadius();
  double groundCacheRadius = acrad
    + 2*dt*norm(vehicle->getVelocity().getLinear());
  Vector3 cart = vehicle->getCartPosition();
  double cart_pos[3] = { cart(1), cart(2), cart(3) };
  double t = vehicle->getTime();
  if (!prepare_ground_cache_m(t, cart_pos, groundCacheRadius))
    SG_LOG(SG_FLIGHT, SG_WARN,
           "FGInterface is beeing called without scenery below the aircraft!");

  // Here a miracle occures :)
  vehicle->output();
  vehicle->update(dt);

  // Now write the newly computed values into the interface class.
  gp = vehicle->getGeodPosition();
  _updateGeodeticPosition(gp.latitude, gp.longitude,
                          convertTo(uFoot, gp.altitude));
//   _set_Altitude_AGL(model->getAGL() * M2FT);

  Rotation geodOr = vehicle->getGeodOrientation();
  euler = geodOr.getEuler();
  _set_Euler_Angles(euler(1), euler(2), euler(3));

  // FIXME: wrong velocities are set here ...
  Vector3 velWrtWind = vehicle->getVelocity().getLinear();
  _set_V_rel_wind(convertTo(uFeetPSecond, norm(velWrtWind)));
  _set_Velocities_Wind_Body(convertTo(uFeetPSecond, velWrtWind(1)),
                            convertTo(uFeetPSecond, velWrtWind(2)),
                            convertTo(uFeetPSecond, velWrtWind(3)));
  _set_V_equiv_kts(convertTo(uKnots, norm(velWrtWind)));
  _set_V_calibrated_kts(convertTo(uKnots, norm(velWrtWind)));
  _set_Mach_number(norm(velWrtWind)/340);

  Vector3 localVel = convertTo(uFeetPSecond, geodOr.backTransform(velWrtWind));
  _set_Velocities_Local(localVel(1),localVel(2), localVel(3));
  _set_V_ground_speed(convertTo(uFeetPSecond,
             sqrt(localVel(1)*localVel(1) + localVel(2)*localVel(2))));
  _set_Velocities_Ground(localVel(1),localVel(2), -localVel(3));;
  _set_Climb_Rate(-localVel(3));

  const RigidBody* topBody = vehicle->getTopBody();
  Vector3 bodyAccel = convertTo(uFeetPSec2, topBody->getFrame()->getClassicAccel().getLinear());
  _set_Accels_Body(bodyAccel(1), bodyAccel(2), bodyAccel(3));
  _set_Accels_Pilot_Body(bodyAccel(1), bodyAccel(2), bodyAccel(3));
  _set_Accels_CG_Body(bodyAccel(1), bodyAccel(2), bodyAccel(3));
  // It is not clear in any way how this 'local acceleration is meant'
  // Just provide one possible interpretation
  Vector3 localAccel = geodOr.backTransform(bodyAccel);
  _set_Accels_Local(localAccel(1), localAccel(2), localAccel(3));

  Vector3 nAccel = 1/convertTo(uFeetPSec2, 9.81) * bodyAccel;
  _set_Accels_CG_Body_N(nAccel(1), nAccel(2), nAccel(3));
  _set_Nlf(-nAccel(3));

  
  Vector3 angVel = topBody->getFrame()->getRelVel().getAngular();
  _set_Omega_Body(angVel(1), angVel(2), angVel(3));
//   _set_Euler_Rates(roll, pitch, hdg);

//   _set_Alpha( Auxiliary->Getalpha() );
//   _set_Beta( Auxiliary->Getbeta() );
//   _set_Gamma_vert_rad( Auxiliary->GetGamma() );
}


void
FGOpenFDM::tieObject(SGPropertyNode* base, Object* object)
{
  // Check for output models
  // If so, we want to reflect that in flightgears property tree
  Output* outputModel = dynamic_cast<Output*>(object);
  if (outputModel) {
    std::string pName = outputModel->getOutputName();
    SGPropertyNode* sgProp = mAircraftRootNode->getNode(pName.c_str(), true);
    sgProp->tie(FGOutputReflector(outputModel));
  }

  // Check for input models
  // If so, we need to register a change notifier in flightgears property tree
  Input* inputModel = dynamic_cast<Input*>(object);
  if (inputModel) {
    SG_LOG(SG_FLIGHT, SG_INFO,
           "Registering input for \"" << inputModel->getName() << "\"");
    std::string pName = inputModel->getInputName();
    SGPropertyNode* sgProp = mAircraftRootNode->getNode(pName.c_str(), true);
    // That adds a change listener to the property node which in turn
    // writes changes to the property back to the input model.
    inputModel->setUserData(new InputChangeUserData(inputModel, sgProp));
  }

  // Reflect all output ports
  Model* model = dynamic_cast<Model*>(object);
  if (model && model->getNumOutputPorts()) {
    SGPropertyNode* outputBase = base->getNode("outputs", true);
    for (unsigned i = 0; i < model->getNumOutputPorts(); ++i) {
      std::string name = model->getOutputPortName(i);
      SGPropertyNode* sgProp = outputBase->getNode(name.c_str(), true);
      sgProp->tie(FGRealPortReflector(model->getOutputPort(i)));
    }
  }

  // The usual, whole object reflection so that one can take a look into
  // OpenFDM's internal modules ...
  std::list<std::string> propertyList = object->listProperties();
  std::list<std::string>::const_iterator it = propertyList.begin();
  while (it != propertyList.end()) {
    // ... well, FIXME cleanup ...
    std::string pName = toPropname(*it);
    SGPropertyNode* sgProp = base->getChild(pName.c_str(), 0, true);
    Variant value = object->getPropertyValue(*it);

    if (value.isString())
      sgProp->tie(FGStringPropertyAdapter(object, *it));
    else if (value.isReal())
      sgProp->tie(FGRealPropertyAdapter(object, *it));
    else if (value.isInteger())
      sgProp->tie(FGIntegerPropertyAdapter(object, *it));
    else if (value.isUnsigned())
      sgProp->tie(FGIntegerPropertyAdapter(object, *it));

    else if (value.isMatrix()) {
      Matrix m = value.toMatrix();
      unsigned reshapeSize = rows(m) * cols(m);

      sgProp->tie(FGRealPropertyAdapter(object, *it));
      for (unsigned i = 2; i <= reshapeSize; ++i) {
        sgProp = base->getChild(pName.c_str(), i-1, true);
        sgProp->tie(FGRealPropertyAdapter(object, *it, i));
      }
    }
    else if (value.isValid()) {
      SG_LOG(SG_FLIGHT, SG_WARN,
             "Found unexpected property type with property named \""
             << *it << "\"");
    }
    ++it;
  }
}

void
FGOpenFDM::tieModelGroup(SGPropertyNode* base, ModelGroup* modelGroup)
{
  unsigned numModels = modelGroup->getNumModels();
  for (unsigned i = 0; i < numModels; ++i) {
    Model* model = modelGroup->getModel(i);
    std::string pName = toPropname(model->getName());
    SGPropertyNode* sgProp = base->getNode(pName.c_str(), true);
    tieObject(sgProp, model);

    ModelGroup* nestedGroup = model->toModelGroup();
    if (nestedGroup) {
      tieModelGroup(sgProp, nestedGroup);
    }
  }
}

void
FGOpenFDM::untieRecursive(SGPropertyNode* base)
{
  // Unties and removes a whole tree.
  // Sadly, it is not possible to remove the properties, so following models
  // will see the properties even if they are no longer usefull.
  int nChildren = base->nChildren();
  for (int i = nChildren-1; 0 <= i; --i) {
    SGPropertyNode* child = base->getChild(i);
    if (!child)
      continue;

    child->clearValue();
    untieRecursive(child);
  }
}

void
FGOpenFDM::untieNamed(SGPropertyNode* base, const char* name)
{
  int sz = base->getChildren(name).size();
  for (int j = sz - 1; 0 <= j; --j) {
    SGPropertyNode* tmpChild = base->getChild(name, j);
    if (!tmpChild)
      continue;
    untieRecursive(tmpChild);
    tmpChild->clearValue();
  }
}

std::string
FGOpenFDM::toPropname(const std::string& name)
{
  std::string pName = name;
  std::string::size_type pos;
  while ((pos = pName.find(' ')) != std::string::npos) {
    pName.replace(pos, 1, 1, '_');
  }
  while ((pos = pName.find('<')) != std::string::npos) {
    pName.replace(pos, 1, 1, '[');
  }
  while ((pos = pName.find('>')) != std::string::npos) {
    pName.replace(pos, 1, 1, ']');
  }
  while ((pos = pName.find('(')) != std::string::npos) {
    pName.replace(pos, 1, 1, '_');
  }
  while ((pos = pName.find(')')) != std::string::npos) {
    pName.replace(pos, 1, 1, '_');
  }
  return pName;
}

} // namespace OpenFDM
