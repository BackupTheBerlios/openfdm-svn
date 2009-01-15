/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "PacejkaTire.h"

#include "LogStream.h"
#include "PortValueList.h"
#include "Task.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(PacejkaTire, SingleLinkInteract)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  DEF_OPENFDM_PROPERTY(Real, WheelRadius, Serialized)
  END_OPENFDM_OBJECT_DEF

class PacejkaTire::Context : public SingleLinkInteract::Context {
public:
  Context(const PacejkaTire* pacejkaTire, const Environment* environment,
          PortValueList& portValueList) :
    SingleLinkInteract::Context(pacejkaTire, environment, portValueList),
    mPacejkaTire(pacejkaTire),
    mSideSlipPort(portValueList.getPortValue(pacejkaTire->mSideSlipPort)),
    mLongitudinalSlipPort(portValueList.getPortValue(pacejkaTire->mLongitudinalSlipPort)),
    mCamberAnglePort(portValueList.getPortValue(pacejkaTire->mCamberAnglePort)),
    mNormalForcePort(portValueList.getPortValue(pacejkaTire->mNormalForcePort)),
    mLateralForcePort(portValueList.getPortValue(pacejkaTire->mLateralForcePort)),
    mLongitudinalForcePort(portValueList.getPortValue(pacejkaTire->mLongitudinalForcePort))
  { }
  virtual ~Context() {}
    
  virtual const PacejkaTire& getNode() const
  { return *mPacejkaTire; }

  virtual void articulation(const Task& task)
  {
    if (mSideSlipPort.isConnected())
      mSideSlipPort = Limits<real_type>::quiet_NaN();
    if (mLongitudinalSlipPort.isConnected())
      mLongitudinalSlipPort = Limits<real_type>::quiet_NaN();
    if (mCamberAnglePort.isConnected())
      mCamberAnglePort = Limits<real_type>::quiet_NaN();
    if (mNormalForcePort.isConnected())
      mNormalForcePort = 0;
    if (mLateralForcePort.isConnected())
      mLateralForcePort = 0;
    if (mLongitudinalForcePort.isConnected())
      mLongitudinalForcePort = 0;

    // Pacejka suggests this correction to avoid singularities at zero speed
    const real_type e_v = 1e-2;

    // The coordinate system at the hub.
    CoordinateSystem hubCoordinateSystem(getLink().getCoordinateSystem());

    // Get the ground values in the hub coordinate system.
    GroundValues groundValues =
      getEnvironment().getGroundPlane(hubCoordinateSystem, task.getTime());
    // The plane equation in the hub coordinate system
    Plane lp = groundValues.plane;


    // The wheel axis vector, named s as in Pacjeka, Fig. 2.3
    Vector3 s = mPacejkaTire->getAxis();

    // Normal up vector as defined in Pacejka [1]
    Vector3 n = copysign(1, lp.getDist())*lp.getNormal();

    // Wheel forward vector as defined in (2.23), Pacejka [1]
    Vector3 l = normalize(cross(n, s));

//     // The side direction in the ground plane
//     Vector3 t = normalize(cross(l, n));

//     // The transform 
//     Matrix33 T(l(0), l(1), l(2),
//                t(0), t(1), t(2),
//                s(0), s(1), s(2));
    
    // Compute the contact point of the wheel with the ground plane
    Vector3 c;
    if (!lp.intersectLine(Vector3::zeros(), cross(l, s), c))
      return;

    // Since we live in our local coordinate system, this is just to be clear
    Vector3 r = c - Vector3::zeros();
    
    // Get the tire deflection.
    real_type rho = mPacejkaTire->getWheelRadius() - norm(r);

    // Don't bother if we do not intersect the ground.
    if (rho < 0)
      return;

    // The relative velocity of the hub wrt the contact point
    // measured in the hubs coordinate system
    Vector6 relVel = getLink().getRefVel() - groundValues.vel;

    Vector6 relVel2 = motionTo(r, relVel);

//     std::cout << trans(r) << std::endl;
//     std::cout << trans(relVel.getLinear()) << " " << 
//       trans(relVel2.getLinear()) << std::endl;
    
    // The compression velocity.
    // Positive when the contact spring is compressed,
    // negative when decompressed.
    real_type rhoDot = dot(relVel.getLinear(), normalize(r));

    // The side direction in the ground plane
    Vector3 t = normalize(cross(l, n));

    // The speed at the contact center
    Vector3 V_c = relVel.getLinear(); // + r_dot; ???
//     Vector3 V_cS = relVel.getLinear(); // + r_dot; ???
    Vector3 V_s = relVel2.getLinear();

    // FIXME ????
    real_type V_cx = dot(V_c, l);
    real_type V_cx_eps = fabs(V_cx) + e_v;
    real_type V_cy = dot(V_c, t);
    real_type V_sx = dot(V_s, l);

    // Note that internally computations are done using derived parameters,
    // thus compute different values for the output ports

    // The sideslip angle (4.E3)
    real_type alpha = atan2(-V_cy, V_cx_eps);
    if (mSideSlipPort.isConnected())
      mSideSlipPort = alpha;

    // The camber angle (4.E4)
    real_type gamma = asin(saturate(-dot(n, s), real_type(1)));
    if (mCamberAnglePort.isConnected())
      mCamberAnglePort = gamma;
    
    // The longitudinal slip (4.E5)
    real_type kappa = -V_sx/V_cx_eps;
    if (mLongitudinalSlipPort.isConnected())
      mLongitudinalSlipPort = kappa;

    Vector6 f = mPacejkaTire->getForce(rho, rhoDot, alpha, kappa, gamma);

    if (mLongitudinalForcePort.isConnected())
      mLongitudinalForcePort = f.getLinear()(0);
    if (mLateralForcePort.isConnected())
      mLateralForcePort = f.getLinear()(1);
    if (mNormalForcePort.isConnected())
      mNormalForcePort = f.getLinear()(2);

    // For now just pure side force and slip
    Vector3 force = f.getLinear()(0)*l + f.getLinear()(1)*t
      + f.getLinear()(2)*n;
    Vector3 contactPoint = c;
    getLink().applyBodyForce(contactPoint, force);
  }

private:
  SharedPtr<const PacejkaTire> mPacejkaTire;

  RealOutputPortHandle mSideSlipPort;
  RealOutputPortHandle mLongitudinalSlipPort;
  RealOutputPortHandle mCamberAnglePort;
  RealOutputPortHandle mNormalForcePort;
  RealOutputPortHandle mLateralForcePort;
  RealOutputPortHandle mLongitudinalForcePort;
};

PacejkaTire::PacejkaTire(const std::string& name) :
  SingleLinkInteract(name),
  mSideSlipPort(this, "sideSlip"),
  mLongitudinalSlipPort(this, "longitudinalSlip"),
  mCamberAnglePort(this, "camberAngle"),
  mNormalForcePort(this, "normalForce"),
  mLateralForcePort(this, "lateralForce"),
  mLongitudinalForcePort(this, "longitudinalForce"),
  mAxis(0, 1, 0),
  mWheelRadius(0.3)
{
}

PacejkaTire::~PacejkaTire(void)
{
}

MechanicContext*
PacejkaTire::newMechanicContext(const Environment* environment,
                                 PortValueList& portValueList) const
{
  return new Context(this, environment, portValueList);
}

const Vector3&
PacejkaTire::getAxis(void) const
{
  return mAxis;
}

void
PacejkaTire::setAxis(const Vector3& axis)
{
  if (norm(axis) <= Limits<real_type>::safe_min())
    return;
  mAxis = normalize(axis);
}

void
PacejkaTire::setWheelRadius(const real_type& wheelRadius)
{
  mWheelRadius = wheelRadius;
}

const real_type&
PacejkaTire::getWheelRadius(void) const
{
  return mWheelRadius;
}

} // namespace OpenFDM
