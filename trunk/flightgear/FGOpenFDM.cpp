/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <simgear/misc/sg_path.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/props/props.hxx>

#include <Aircraft/controls.hxx>
#include <Main/globals.hxx>
#include <Main/fg_props.hxx>

#include <JSBSim/JSBSimReader.h>
#include <OpenFDM/Unit.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/Input.h>
#include <OpenFDM/Output.h>
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
    double loc_cart[3] = { refPos(0), refPos(1), refPos(2) };
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

  virtual bool
  getCatapultValues(real_type t, const Vector3& refPos,
                    CatapultValues& catVal) const
  {
    if (!mIfce)
      return Limits<real_type>::max();

    double pt[3] = { refPos(0), refPos(1), refPos(2) };
    double end[2][3];
    double vel[2][3];
    real_type dist = mIfce->get_cat_m(t, pt, end, vel);

    Vector3 p0 = Vector3(end[0][0], end[0][1], end[0][2]);
    Vector3 p1 = Vector3(end[1][0], end[1][1], end[1][2]);

    catVal.position = p0;
    catVal.orientation = Quaternion::fromRotateTo(p1 - p0, Vector3::unit(0));
    /// FIXME: wrong ...
    catVal.velocity = Vector6(Vector3::zeros(),
                              Vector3(vel[0][0], vel[0][1], vel[0][2]));
    catVal.length = norm(p1 - p0);

    return dist < 5;
  }

  virtual bool
  caughtWire(const HookPosition& old, const HookPosition& current) const
  {
    if (!mIfce)
      return false;

    Vector3 oldTip = old.basePosition + old.hookVector;
    Vector3 currentTip = current.basePosition + current.hookVector;
    double pt[4][3] = {
      { old.basePosition(0), old.basePosition(1), old.basePosition(2) },
      { oldTip(0), oldTip(1), oldTip(2) },
      { currentTip(0), currentTip(1), currentTip(2) },
      { current.basePosition(0), current.basePosition(1), current.basePosition(2) },
    };
    return mIfce->caught_wire_m(old.t, pt);
  }

  virtual bool
  getWireEnds(real_type t, WireValues& wireVal) const
  {
    if (!mIfce)
      return false;

    double end[2][3];
    double vel[2][3];
    if (!mIfce->get_wire_ends_m(t, end, vel))
      return false;

    Vector3 p0 = Vector3(end[0][0], end[0][1], end[0][2]);
    Vector3 p1 = Vector3(end[1][0], end[1][1], end[1][2]);

    wireVal.position = 0.5*(p0 + p1);
    wireVal.orientation = Quaternion::fromRotateTo(p1 - p0, Vector3::unit(2));

    Vector3 v0 = Vector3(vel[0][0], vel[0][1], vel[0][2]);
    Vector3 v1 = Vector3(vel[1][0], vel[1][1], vel[1][2]);
    wireVal.velocity = Vector6(Vector3::zeros(), 0.5*(v0 + v1));
    wireVal.width = norm(p1 - p0);

    return true;
  }
  
  virtual void
  releaseWire(void) const
  {
    if (!mIfce)
      return;
    mIfce->release_wire();
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
    double xyz[3] = { cart(0), cart(1), cart(2) };
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
    double xyz[3] = { pos(0), pos(1), pos(2) };
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

    return Unit::footPerSecond().convertFrom(T*Vector3(north, east, down));
  }
};


//     temperature = fgGetNode("/environment/temperature-degc",true);
//     pressure = fgGetNode("/environment/pressure-inhg",true);
//     density = fgGetNode("/environment/density-slugft3",true);
//     turbulence_gain = fgGetNode("/environment/turbulence/magnitude-norm",true);
//     turbulence_rate = fgGetNode("/environment/turbulence/rate-hz",true);



class InputConnectModelVisitor :
    public ModelVisitor {
public:
  InputConnectModelVisitor(const SGSharedPtr<SGPropertyNode>& acRootNode) :
    mAircraftRootNode(acRootNode)
  { }
  virtual void apply(Model& model)
  {
    // Check for input models
    // If so, we need to register a change notifier in flightgears properties
    Input* inputModel = model.toInput();
    if (!inputModel)
      return;

    SG_LOG(SG_FLIGHT, SG_INFO,
           "Registering input for \"" << inputModel->getName() << "\"");
    std::string pName = inputModel->getInputName();
    SGPropertyNode* sgProp = mAircraftRootNode->getNode(pName.c_str(), true);
    inputModel->setCallback(new FGInputCallback(sgProp));
  }
  virtual void apply(ModelGroup& modelGroup)
  { traverse(modelGroup); }
private:
  SGSharedPtr<SGPropertyNode> mAircraftRootNode;
};



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

  // FIXME: this is clearly wrong, but avoids ending in an endless init loop
  set_inited(true);

  std::string aircraftDir = fgGetString("/sim/aircraft-dir");
  std::string engineDir = aircraftDir + "/Engines";
  std::string aircraftFile = std::string(fgGetString("/sim/aero")) + ".xml";

  JSBSimReader reader;
  reader.addAircraftPath(aircraftDir);
  reader.addEnginePath(engineDir);
  reader.loadAircraft(aircraftFile);
  if (reader.getErrorState()) {
    SG_LOG(SG_FLIGHT, SG_ALERT, "FGOpenFDM::init() cannot read aircraft!");
    
    SG_LOG(SG_FLIGHT, SG_ALERT, "FGOpenFDM::init() Error messages from JSBSim reader:");
    ReaderWriter::StringList errors = reader.getErrors();
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

  // connect the input models with the input properties
  InputConnectModelVisitor icmv(mAircraftRootNode);
  vehicle->getSystem()->accept(icmv);

  // Hmm, twice ??
  if (!vehicle->init()) {
    mData->vehicle = 0;
    return;
  }

  MobileRootJoint* mobileRootJoint = vehicle->getMobileRootJoint();
  // Check the position
  Geodetic gp = Geodetic(get_Latitude(),
                         get_Longitude(),
                         Unit::foot().convertFrom(get_Altitude()));
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
    mobileRootJoint->setLinearRelVel(Unit::footPerSecond().convertFrom(bodyVel));
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
                          Unit::foot().convertTo(gp.altitude));

  Rotation geodOr = vehicle->getGeodOrientation();
  Vector3 euler = geodOr.getEuler();
  _set_Euler_Angles(euler(1), euler(2), euler(3));
}

void FGOpenFDM::bind()
{
  SG_LOG(SG_FLIGHT, SG_INFO, "FGOpenFDM::bind()");

  FGInterface::bind();

  if (!mData->vehicle)
    return;

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
    SG_LOG(SG_FLIGHT, SG_WARN,
           "FGOpenFDM::update(double) is called without an aircraft loaded!");
    return;
  }

  // Check the position
  Geodetic gp = Geodetic(get_Latitude(),
                         get_Longitude(),
                         Unit::foot().convertFrom(get_Altitude()));
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
  double cart_pos[3] = { cart(0), cart(1), cart(2) };
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
                          Unit::foot().convertTo(gp.altitude));
//   _set_Altitude_AGL(model->getAGL() * M2FT);

  Rotation geodOr = vehicle->getGeodOrientation();
  euler = geodOr.getEuler();
  _set_Euler_Angles(euler(0), euler(1), euler(2));

  // FIXME: wrong velocities are set here ...
  Vector3 velWrtWind = vehicle->getVelocity().getLinear();
  _set_V_rel_wind(Unit::footPerSecond().convertTo(norm(velWrtWind)));
  _set_Velocities_Wind_Body(Unit::footPerSecond().convertTo(velWrtWind(0)),
                            Unit::footPerSecond().convertTo(velWrtWind(1)),
                            Unit::footPerSecond().convertTo(velWrtWind(2)));
  _set_V_equiv_kts(Unit::knots().convertTo(norm(velWrtWind)));
  _set_V_calibrated_kts(Unit::knots().convertTo(norm(velWrtWind)));
  _set_Mach_number(norm(velWrtWind)/340);

  Vector3 localVel = Unit::footPerSecond().convertTo(geodOr.backTransform(velWrtWind));
  _set_Velocities_Local(localVel(0),localVel(1), localVel(2));
  _set_V_ground_speed(Unit::footPerSecond().convertTo(
             sqrt(localVel(0)*localVel(0) + localVel(1)*localVel(1))));
  _set_Velocities_Ground(localVel(0), localVel(1), -localVel(2));;
  _set_Climb_Rate(-localVel(2));

  const RigidBody* topBody = vehicle->getTopBody();
  Vector3 bodyAccel = Unit::footPerSecond2().convertTo(topBody->getFrame()->getClassicAccel().getLinear());
  _set_Accels_Body(bodyAccel(0), bodyAccel(1), bodyAccel(2));
  _set_Accels_Pilot_Body(bodyAccel(0), bodyAccel(1), bodyAccel(2));
  _set_Accels_CG_Body(bodyAccel(0), bodyAccel(1), bodyAccel(2));
  // It is not clear in any way how this 'local acceleration is meant'
  // Just provide one possible interpretation
  Vector3 localAccel = geodOr.backTransform(bodyAccel);
  _set_Accels_Local(localAccel(0), localAccel(1), localAccel(2));

  Vector3 nAccel = 1/Unit::footPerSecond2().convertTo(9.81) * bodyAccel;
  _set_Accels_CG_Body_N(nAccel(0), nAccel(1), nAccel(2));


  SGPropertyNode* sgProp;
  // is already in the property tree, but fool the HUD now
  sgProp = mAircraftRootNode->getNode("accelerations/nlf", false);
  if (sgProp)
    _set_Nlf( sgProp->getDoubleValue() );

  Vector3 angVel = topBody->getFrame()->getRelVel().getAngular();
  _set_Omega_Body(angVel(0), angVel(1), angVel(2));

  // is already in the property tree, but fool the HUD now
  sgProp = mAircraftRootNode->getNode("orientation/alpha-deg", false);
  if (sgProp)
    _set_Alpha( sgProp->getDoubleValue() );
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
    if (sgProp->isTied())
      sgProp->untie();
    sgProp->tie(FGOutputReflector(outputModel));
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
  std::vector<PropertyInfo> props;
  object->getPropertyInfoList(props);
  std::vector<PropertyInfo>::iterator it = props.begin();
  while (it != props.end()) {
    // ... well, FIXME cleanup ...
    std::string pName = toPropname(it->getName());
    SGPropertyNode* sgProp = base->getChild(pName.c_str(), 0, true);
    Variant value;
    object->getPropertyValue(it->getName(), value);

    if (value.isString())
      sgProp->tie(FGStringPropertyAdapter(object, it->getName()));
    else if (value.isReal())
      sgProp->tie(FGRealPropertyAdapter(object, it->getName()));
    else if (value.isInteger())
      sgProp->tie(FGIntegerPropertyAdapter(object, it->getName()));
    else if (value.isUnsigned())
      sgProp->tie(FGIntegerPropertyAdapter(object, it->getName()));

    else if (value.isMatrix()) {
      Matrix m = value.toMatrix();
      unsigned reshapeSize = rows(m) * cols(m);

      sgProp->tie(FGRealPropertyAdapter(object, it->getName()));
      for (unsigned i = 2; i <= reshapeSize; ++i) {
        sgProp = base->getChild(pName.c_str(), i-1, true);
        sgProp->tie(FGRealPropertyAdapter(object, it->getName(), i));
      }
    }
    else if (value.isValid()) {
      SG_LOG(SG_FLIGHT, SG_WARN,
             "Found unexpected property type with property named \""
             << it->getName() << "\"");
    }
    ++it;
  }
}

void
FGOpenFDM::tieModelGroup(SGPropertyNode* base, ModelGroup* modelGroup)
{
  unsigned numModels = modelGroup->getNumModels();
  for (unsigned i = 0; i < numModels; ++i) {
    Node* model = modelGroup->getModel(i);
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
