/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Joint_H
#define OpenFDM_Joint_H

#include "Assert.h"
#include "Object.h"
#include "Frame.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Interact.h"
#include "Frame.h"
#include "LogStream.h"

namespace OpenFDM {

/// FIXME: joint's should be lockable, which means trylock == true and
/// velocity small enough - keep position ...

class Joint : public Interact {
  OPENFDM_OBJECT(Joint, Interact);
public:
  Joint(const std::string& name);
  virtual ~Joint(void);

  // Joints cannot do something different than apply forces and inertia to
  // its parent.
  virtual void velocity(const MechanicLinkValue& parentLink,
                        MechanicLinkValue& childLink,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues,
                        FrameData& frameData) const = 0;
  virtual void articulation(MechanicLinkValue& parentLink,
                            const MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            FrameData& frameData) const = 0;
  virtual void acceleration(const MechanicLinkValue& parentLink,
                            MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            FrameData& frameData) const = 0;

  /// They implement the mechanic stuff
  virtual void velocity(const Task&, const ContinousStateValueVector&,
                        PortValueList&, FrameData&) const;
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList&, FrameData&) const;
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList&, FrameData&) const;
private:
  MechanicLink mParentLink;
  MechanicLink mChildLink;
};

} // namespace OpenFDM

#endif
