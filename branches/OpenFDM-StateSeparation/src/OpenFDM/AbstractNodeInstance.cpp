/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "AbstractNodeInstance.h"

#include "Assert.h"
#include "Node.h"
#include "SharedPtr.h"
#include "WeakReferenced.h"

namespace OpenFDM {

AbstractNodeInstance::AbstractNodeInstance(const NodePath& nodePath,
                                           const SampleTime& sampleTime) :
  mNodePath(nodePath),
  mSampleTime(sampleTime)
{
  OpenFDMAssert(!nodePath.empty());
}

AbstractNodeInstance::~AbstractNodeInstance()
{
}

std::string
AbstractNodeInstance::getNodeNamePath() const
{
  if (mNodePath.empty())
    return std::string();
  std::string path = mNodePath.front()->getName();
  NodePath::const_iterator i = mNodePath.begin();
  if (i != mNodePath.end()) {
    for (++i; i != mNodePath.end(); ++i) {
      path += '/';
      path += (*i)->getName();
    }
  }
  return path;
}

} // namespace OpenFDM
