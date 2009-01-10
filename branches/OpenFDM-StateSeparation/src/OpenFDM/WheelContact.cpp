/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "WheelContact.h"

#include "LogStream.h"
#include "PortValueList.h"
#include "Task.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(WheelContact, SingleLinkInteract)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  DEF_OPENFDM_PROPERTY(Real, WheelRadius, Serialized)
  DEF_OPENFDM_PROPERTY(Real, SpringConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DampingConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, FrictionCoeficient, Serialized)
  END_OPENFDM_OBJECT_DEF

class WheelContact::Context : public SingleLinkInteract::Context {
public:
  Context(const WheelContact* wheelContact,
          const Environment* environment, PortValueList& portValueList) :
    SingleLinkInteract::Context(wheelContact, environment, portValueList),
    mWheelContact(wheelContact)
  { }
  virtual ~Context() {}
    
  virtual const WheelContact& getNode() const
  { return *mWheelContact; }

  virtual void articulation(const Task& task)
  {
    // The coordinate system at the hub.
    CoordinateSystem hubCoordinateSystem(getLink().getCoordinateSystem());

    // Get the ground values in the hub coordinate system.
    GroundValues groundValues =
      getEnvironment().getGroundPlane(hubCoordinateSystem, task.getTime());
    
    // Transform the plane equation to the local frame.
    Plane lp = groundValues.plane;
    
    // Get the intersection length.
    real_type distHubGround = fabs(lp.getDist());
    Vector3 down = -copysign(1, lp.getDist())*lp.getNormal();
    real_type compressLength = mWheelContact->getWheelRadius() - distHubGround;
    
    // Don't bother if we do not intersect the ground.
    if (compressLength < 0)
      return;
    
    Vector3 contactPoint = distHubGround*down;
    
    // The relative velocity of the ground wrt the contact point
    Vector6 relVel = getLink().getRefVel() - groundValues.vel;
    
    // The velocity perpandicular to the plane.
    // Positive when the contact spring is compressed,
    // negative when decompressed.
    real_type compressVel = - lp.scalarProjectToNormal(relVel.getLinear());
    
    // Get a transform from the current frames coordinates into
    // wheel coordinates.
    // The wheel coordinates x axis is defined by the forward orientation
    // of the wheel, the z axis points perpandicular to the ground
    // plane downwards.
    Vector3 forward = normalize(cross(mWheelContact->getAxis(), down));
    Vector3 side = normalize(cross(down, forward));
    
    // Transformed velocity to the ground plane
    Vector2 wheelVel(dot(forward, relVel.getLinear()),
                     dot(side, relVel.getLinear()));
    
    // The wheel rotation speed wrt ground
    Vector3 rotVel = relVel.getAngular();
    real_type omegaR = dot(rotVel, mWheelContact->getAxis()) * distHubGround;
    
    //   Log(Model,Error) << trans(groundValues.vel) << " "
    //                    << trans(wheelVel) << " "
    //                    << omegaR << " "
    //                    << compressLength << " "
    //                    << distHubGround << endl;
    
    
    // Get the plane normal force.
    real_type normForce = mWheelContact->computeNormalForce(compressLength,
                                                            compressVel,
                                                            mPortValueList);
    // The normal force cannot get negative here.
    normForce = max(static_cast<real_type>(0), normForce);
    
    // Get the friction force.
    Vector2 fricForce = mWheelContact->computeFrictionForce(normForce, wheelVel,
                                 omegaR, groundValues.friction, mPortValueList);
    
    // The resulting force is the sum of both.
    // The minus sign is because of the direction of the surface normal.
    Vector3 force = fricForce(0)*forward + fricForce(1)*side - normForce*down;
    
    // We don't have an angular moment.
    getLink().applyBodyForce(contactPoint, force);
  }

private:
  SharedPtr<const WheelContact> mWheelContact;
};

WheelContact::WheelContact(const std::string& name) :
  SingleLinkInteract(name),
  mAxis(0, 1, 0)
{
  mWheelRadius = 0.3;
  mSpringConstant = 0;
  mDampingConstant = 0;
  mFrictionCoeficient = 0.8;
}

WheelContact::~WheelContact(void)
{
}

MechanicContext*
WheelContact::newMechanicContext(const Environment* environment,
                                 PortValueList& portValueList) const
{
  return new Context(this, environment, portValueList);
}

real_type
WheelContact::computeNormalForce(real_type compressLen,
                                 real_type compressVel,
                                 PortValueList&) const
{
  return compressLen*mSpringConstant + mDampingConstant*compressVel;
}

Vector2
WheelContact::computeFrictionForce(real_type normForce, const Vector2& vel,
                                   real_type omegaR, real_type friction,
                                   PortValueList&) const
{
  // We just get the wheel slip directly here
  real_type wheelSlip = vel(0)+omegaR;
  
  // The slip angle is the angle between the 'velocity vector' and 
  // the wheel forward direction.
  real_type slipAngle = rad2deg*atan2(vel(1), fabs(vel(0)));
//   slipAngle = saturate(slipAngle, 10*fabs(vel(1)));
  slipAngle = smoothSaturate(slipAngle, 10*fabs(vel(1)));
  
//   Vector2 slip(wheelSlip, slipAngle);
//   if (1 < norm(slip))
//     slip = normalize(slip);
  Vector2 slip(smoothSaturate(wheelSlip, real_type(1)),
               smoothSaturate(slipAngle, real_type(1)));
  
  // The friction force for fast movement.
  return (-friction*mFrictionCoeficient*normForce)*slip;
}

const Vector3&
WheelContact::getAxis(void) const
{
  return mAxis;
}

void
WheelContact::setAxis(const Vector3& axis)
{
  if (norm(axis) <= Limits<real_type>::safe_min())
    return;
  mAxis = normalize(axis);
}

} // namespace OpenFDM
