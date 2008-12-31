/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Mass.h"

#include "PortValueList.h"
#include "Transform.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Mass, SingleLinkInteract)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  DEF_OPENFDM_PROPERTY(Inertia, Inertia, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Mass, Serialized)
  END_OPENFDM_OBJECT_DEF

class Mass::Context : public SingleLinkInteract::Context {
public:
  Context(const Mass* mass,
          const Environment* environment, PortValueList& portValueList) :
    SingleLinkInteract::Context(mass, environment, portValueList),
    mMass(mass),
    mLinkRelPos(Vector3::zeros()),
    mSpatialInertia(SpatialInertia::zeros())
  { }
  virtual ~Context() {}
    
  virtual const Mass& getNode() const
  { return *mMass; }

  virtual void initDesignPosition()
  {
    mLinkRelPos = mMass->getPosition() - getLink().getDesignPosition();
    mSpatialInertia = SpatialInertia(mMass->getInertia(), mMass->getMass());
    mSpatialInertia = inertiaFrom(mLinkRelPos, mSpatialInertia);
  }
  virtual void articulation(const Task&)
  {
    // Contribute the inerita
    getLink().addInertia(mSpatialInertia);

    // Each inertia has a contribution to the spatial bias force.
    // This part is handled here.
    Vector6 v = getLink().getSpVel();
    Vector6 Iv = mSpatialInertia*v;
    Vector6 vIv = Vector6(cross(v.getAngular(), Iv.getAngular()) +
                          cross(v.getLinear(), Iv.getLinear()),
                          cross(v.getAngular(), Iv.getLinear()));
    getLink().addForce(vIv);

    // Now the gravity part
    Vector3 refPos = getLink().getCoordinateSystem().toReference(mLinkRelPos);
    Vector3 gravity = getEnvironment().getGravityAcceleration(refPos);
    gravity = getLink().getCoordinateSystem().rotToLocal(gravity);
    gravity *= mMass->getMass();
    getLink().applyForce(mLinkRelPos, gravity);
  }

private:
  SharedPtr<const Mass> mMass;
  Vector3 mLinkRelPos;
  SpatialInertia mSpatialInertia;
};

Mass::Mass(const std::string& name, const real_type& mass,
           const InertiaMatrix& inertia, const Vector3& position) :
  SingleLinkInteract(name),
  mMass(mass),
  mInertia(inertia),
  mPosition(position)
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

const Vector3&
Mass::getPosition(void) const
{
  return mPosition;
}

void
Mass::setPosition(const Vector3& position)
{
  mPosition = position;
}

} // namespace OpenFDM
