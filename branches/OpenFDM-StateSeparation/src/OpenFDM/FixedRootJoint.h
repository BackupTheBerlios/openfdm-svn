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
  const Vector3& getRootPosition() const;
  void setRootPosition(const Vector3& rootRosition);

  /// The orientation is global coordinates
  const Quaternion& getRootOrientation() const;
  void setRootOrientation(const Quaternion& rootOrientation);

  virtual void init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector&,
                    const PortValueList&) const;
  virtual void initDesignPosition(PortValueList&) const;
  virtual void velocity(const Task&, const Environment& environment,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues) const;
  virtual void articulation(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
                            PortValueList& portValues) const;
  virtual void acceleration(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
                            PortValueList& portValues) const;
private:
  MechanicLink mMechanicLink;

  Vector3 mRootPosition;
  Quaternion mRootOrientation;
};

} // namespace OpenFDM

#endif
