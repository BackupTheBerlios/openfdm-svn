/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#include "LibraryNode.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(LibraryNode, Node)
  END_OPENFDM_OBJECT_DEF

LibraryNode::LibraryNode(const std::string& name, LibraryModel* libraryModel) :
  Node(name),
  mLibraryModel(libraryModel)
{
}

LibraryNode::~LibraryNode()
{
}

void
LibraryNode::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
LibraryNode::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

SharedPtr<LibraryModel>
LibraryNode::getLibraryModel()
{
  return mLibraryModel;
}

SharedPtr<const LibraryModel>
LibraryNode::getLibraryModel() const
{
  return mLibraryModel;
}

void
LibraryNode::setLibraryModel(LibraryModel* libraryModel)
{
  mLibraryModel = libraryModel;
}

} // namespace OpenFDM
