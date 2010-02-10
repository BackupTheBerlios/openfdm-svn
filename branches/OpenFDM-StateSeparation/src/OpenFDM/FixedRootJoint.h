/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
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

  virtual JointContext*
  newJointContext(const Environment* environment,
                  MechanicLinkValue* parentLink,
                  MechanicLinkValue* childLink,
                  PortValueList& portValueList) const;

  /// The position is global coordinates
  const Vector3& getRootPosition() const;
  void setRootPosition(const Vector3& rootRosition);

  /// The orientation is global coordinates
  const Quaternion& getRootOrientation() const;
  void setRootOrientation(const Quaternion& rootOrientation);

private:
  class Context;

  SharedPtr<MechanicLink> mMechanicLink;

  Vector3 mRootPosition;
  Quaternion mRootOrientation;
};

} // namespace OpenFDM

#endif
