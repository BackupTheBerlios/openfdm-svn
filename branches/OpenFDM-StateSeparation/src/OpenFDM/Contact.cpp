/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Contact.h"

#include "Assert.h"
#include "LogStream.h"
#include "Task.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Contact, SingleLinkInteract)
  END_OPENFDM_OBJECT_DEF

class Contact::Context : public SingleLinkInteract::Context {
public:
  Context(const Contact* wheelContact,
          const Environment* environment, PortValueList& portValueList) :
    SingleLinkInteract::Context(wheelContact, environment, portValueList),
    mContact(wheelContact)
  { }
  virtual ~Context() {}
    
  virtual const Contact& getNode() const
  { return *mContact; }

  virtual void articulation(const Task& task)
  {
    // The coordinate system at the body fixed contact point.
    CoordinateSystem localCoordSys(getLink().getCoordinateSystem());
    
    // Get the ground values in the hub coordinate system.
    GroundValues groundValues =
      getEnvironment().getGroundPlane(localCoordSys, task.getTime());
    
    // Transform the plane equation to the local frame.
    Plane lp = groundValues.plane;
    
    // Get the intersection length.
    real_type compressLength = lp.getDist(Vector3::zeros());
    
    // Don't bother if we do not intersect the ground.
    if (compressLength < 0)
      return;
    
    // The velocity of the ground patch in the current frame.
    Vector3 relVel = groundValues.vel.getLinear();
    // Now get the relative velocity of the ground wrt the contact point
    relVel -= getLink().getLocalVelocity().getLinear();

    
    // The velocity perpandicular to the plane.
    // Positive when the contact spring is compressed,
    // negative when decompressed.
    real_type compressVel = - lp.scalarProjectToNormal(relVel);
    
    // The in plane velocity.
    Vector3 sVel = lp.projectToPlane(relVel);
    
    // Get the plane normal force.
    real_type normForce = mContact->computeNormalForce(compressLength,
                                                       compressVel,
                                                       mPortValueList);
    // The normal force cannot get negative here.
    normForce = max(static_cast<real_type>(0), normForce);
    
    // Get the friction force.
    Vector3 fricForce = mContact->computeFrictionForce(normForce, sVel,
                                                       lp.getNormal(),
                                                       groundValues.friction,
                                                       mPortValueList);
    
    // The resulting force is the sum of both.
    // The minus sign is because of the direction of the surface normal.
    Vector3 force = fricForce - normForce*lp.getNormal();
    
    // We don't have an angular moment.
    getLink().applyBodyForce(force);
  }

private:
  SharedPtr<const Contact> mContact;
};

Contact::Contact(const std::string& name) :
  SingleLinkInteract(name)
{
}

Contact::~Contact(void)
{
}

MechanicContext*
Contact::newMechanicContext(const Environment* environment,
                            PortValueList& portValueList) const
{
  return new Context(this, environment, portValueList);
}

real_type
Contact::computeNormalForce(real_type, real_type, PortValueList&) const
{
  return 0;
}

Vector3
Contact::computeFrictionForce(real_type, const Vector3&, const Vector3&,
                              real_type, PortValueList&) const
{
  return Vector3::zeros();
}

} // namespace OpenFDM
