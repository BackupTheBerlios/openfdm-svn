/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicInstance_H
#define OpenFDM_MechanicInstance_H

#include <list>
#include "AbstractNodeInstance.h"
#include "MechanicContext.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MechanicInstance : public AbstractNodeInstance {
public:
  MechanicInstance(const NodePath& nodePath, const SampleTime& sampleTime,
                   const MechanicNode* mechanicNode);
  virtual ~MechanicInstance();

  bool isConnectedTo(const MechanicInstance& mechanicInstance) const;

// protected:
  virtual MechanicContext& getNodeContext();
  virtual const MechanicContext& getNodeContext() const;

private:
  SharedPtr<MechanicContext> mMechanicContext;
};

typedef std::list<SharedPtr<MechanicInstance> > MechanicInstanceList;

} // namespace OpenFDM

#endif
