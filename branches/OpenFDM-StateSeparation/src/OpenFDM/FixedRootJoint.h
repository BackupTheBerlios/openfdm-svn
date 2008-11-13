/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_FixedRootJoint_H
#define OpenFDM_FixedRootJoint_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "MatrixStateInfo.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "RootJoint.h"

namespace OpenFDM {

class FixedRootJoint : public RootJoint {
  OPENFDM_OBJECT(FixedRootJoint, RootJoint);
public:
  FixedRootJoint(const std::string& name);
  virtual ~FixedRootJoint();

  /// The position is global coordinates
  const Vector3& getPosition() const;
  void setPosition(const Vector3& position);

  /// The orientation is global coordinates
  const Quaternion& getOrientation() const;
  void setOrientation(const Quaternion& orientation);

  virtual void init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector&,
                    const PortValueList&) const;
  virtual void initDesignPosition(PortValueList&) const;
  virtual void velocity(const Task&, const ContinousStateValueVector& states,
                        PortValueList& portValues) const;
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues) const;
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues) const;
private:
  MechanicLink mMechanicLink;

  Vector3 mPosition;
  Quaternion mOrientation;
};

} // namespace OpenFDM

#endif
