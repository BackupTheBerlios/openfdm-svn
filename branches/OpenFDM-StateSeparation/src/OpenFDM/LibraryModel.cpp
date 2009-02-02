/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include "LibraryModel.h"

#include "LibraryNode.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(LibraryModel, Object)
  END_OPENFDM_OBJECT_DEF

LibraryModel::LibraryModel(const std::string& name, Node* node) :
  Object(name),
  mNode(node)
{
}

LibraryModel::~LibraryModel()
{
}

unsigned
LibraryModel::getNumParentNodes() const
{
  return mParentNodes.size();
}
  
WeakPtr<LibraryNode>
LibraryModel::getParent(unsigned i)
{
  if (mParentNodes.size() <= i)
    return 0;
  return mParentNodes[i];
}

WeakPtr<const LibraryNode>
LibraryModel::getParent(unsigned i) const
{
  if (mParentNodes.size() <= i)
    return 0;
  return mParentNodes[i];
}

SharedPtr<Node>
LibraryModel::getNode()
{
  return mNode;
}

SharedPtr<const Node>
LibraryModel::getNode() const
{
  return mNode;
}

void
LibraryModel::setNode(Node* node)
{
  mNode = node;
}

} // namespace OpenFDM
