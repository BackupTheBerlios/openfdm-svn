/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Mass.h"

#include "PortValueList.h"
#include "Transform.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Mass, SingleLinkInteract)
  DEF_OPENFDM_PROPERTY(Inertia, Inertia, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Mass, Serialized)
  END_OPENFDM_OBJECT_DEF

class Mass::Context : public SingleLinkInteract::Context {
public:
  Context(const Mass* mass,
          const Environment* environment, PortValueList& portValueList) :
    SingleLinkInteract::Context(mass, environment, portValueList),
    mMass(mass)
  { }
  virtual ~Context() {}
    
  virtual const Mass& getNode() const
  { return *mMass; }

  virtual void initDesignPosition()
  {
    SingleLinkInteract::Context::initDesignPosition();
  }
  virtual void articulation(const Task&)
  {
    CoordinateSystem cs = getLink().getCoordinateSystem();

    SpatialInertia I(cs.rotToReference(mMass->getInertia()), mMass->getMass());

    // Contribute the inerita
    getLink().addSpatialInertia(cs.getPosition(), I);

    // Each inertia has a contribution to the spatial bias force.
    // This part is handled here.
    Vector6 v = getLink().getVelocity(cs.getPosition());
    Vector6 Iv = I*v;
    Vector6 vIv = Vector6(cross(v.getAngular(), Iv.getAngular()) +
                          cross(v.getLinear(), Iv.getLinear()),
                          cross(v.getAngular(), Iv.getLinear()));
    getLink().addSpatialForce(cs.getPosition(), vIv);

    // Now the gravity part
    Vector3 gravity = getEnvironment().getGravityAcceleration(cs.getPosition());
    gravity *= mMass->getMass();
    getLink().applyForce(cs.getPosition(), gravity);
  }

private:
  SharedPtr<const Mass> mMass;
};

Mass::Mass(const std::string& name, const real_type& mass,
           const InertiaMatrix& inertia, const Vector3& position) :
  SingleLinkInteract(name),
  mMass(mass),
  mInertia(inertia)
{
}

Mass::~Mass(void)
{
}

MechanicContext*
Mass::newMechanicContext(const Environment* environment,
                         PortValueList& portValueList) const
{
  return new Context(this, environment, portValueList);
}

const InertiaMatrix&
Mass::getInertia(void) const
{
  return mInertia;
}

void
Mass::setInertia(const InertiaMatrix& inertia)
{
  mInertia = inertia;
}

const real_type&
Mass::getMass() const
{
  return mMass;
}

void
Mass::setMass(const real_type& mass)
{
  mMass = mass;
}

} // namespace OpenFDM
